// Use PoseTracking graph to get landmarks and OpenCV to draw on original image

#include <cstdlib>

#include "mediapipe/framework/calculator_framework.h"
#include "mediapipe/framework/formats/image_frame.h"
#include "mediapipe/framework/formats/image_frame_opencv.h"
#include "mediapipe/framework/formats/landmark.pb.h"
#include "mediapipe/framework/port/commandlineflags.h"
#include "mediapipe/framework/port/file_helpers.h"
#include "mediapipe/framework/port/opencv_highgui_inc.h"
#include "mediapipe/framework/port/opencv_imgproc_inc.h"
#include "mediapipe/framework/port/opencv_video_inc.h"
#include "mediapipe/framework/port/parse_text_proto.h"
#include "mediapipe/framework/port/status.h"
#include "mediapipe/gpu/gl_calculator_helper.h"
#include "mediapipe/gpu/gpu_buffer.h"
#include "mediapipe/gpu/gpu_shared_data_internal.h"

constexpr char kInputStream[] = "input_video";
constexpr char kOutputStream_Video[] = "output_video";
constexpr char kOutputStream_Landmarks[] = "pose_landmarks";
constexpr char kWindowName[] = "Pose Tracking Data";

mediapipe::Status RunMPPGraph() {
  std::string calculator_graph_config_contents = R"(
    input_stream: "input_video"
    output_stream: "output_video"
    output_stream: "pose_landmarks"

    node {
      calculator: "FlowLimiterCalculator"
      input_stream: "input_video"
      input_stream: "FINISHED:output_video"
      input_stream_info: {
        tag_index: "FINISHED"
        back_edge: true
      }
      output_stream: "throttled_input_video"
    }

    node {
      calculator: "PoseLandmarkGpu"
      input_stream: "IMAGE:throttled_input_video"
      output_stream: "LANDMARKS:pose_landmarks"
      output_stream: "DETECTION:pose_detection"
      output_stream: "ROI_FROM_LANDMARKS:roi_from_landmarks"
    }

    node {
      calculator: "ImagePropertiesCalculator"
      input_stream: "IMAGE_GPU:throttled_input_video"
      output_stream: "SIZE:image_size"
    }

    node {
      calculator: "LandmarksSmoothingCalculator"
      input_stream: "NORM_LANDMARKS:pose_landmarks"
      input_stream: "IMAGE_SIZE:image_size"
      output_stream: "NORM_FILTERED_LANDMARKS:pose_landmarks_smoothed"
      node_options: {
        [type.googleapis.com/mediapipe.LandmarksSmoothingCalculatorOptions] {
          velocity_filter: {
            window_size: 5
            velocity_scale: 10.0
          }
        }
      }
    }

    node {
      calculator: "PoseRendererGpu"
      input_stream: "IMAGE:throttled_input_video"
      input_stream: "LANDMARKS:pose_landmarks_smoothed"
      input_stream: "ROI:roi_from_landmarks"
      input_stream: "DETECTION:pose_detection"
      output_stream: "IMAGE:output_video"
    }
  )";
  mediapipe::CalculatorGraphConfig config =
      mediapipe::ParseTextProtoOrDie<mediapipe::CalculatorGraphConfig>(calculator_graph_config_contents);

  LOG(INFO) << "Initialize the calculator graph...";
  mediapipe::CalculatorGraph graph;
  MP_RETURN_IF_ERROR(graph.Initialize(config));

  LOG(INFO) << "Initialize the GPU...";
  ASSIGN_OR_RETURN(auto gpu_resources, mediapipe::GpuResources::Create());
  MP_RETURN_IF_ERROR(graph.SetGpuResources(std::move(gpu_resources)));
  mediapipe::GlCalculatorHelper gpu_helper;
  gpu_helper.InitializeForTest(graph.GetGpuResources().get());

  LOG(INFO) << "Initialize camera..";
  cv::VideoCapture capture;
  capture.open(0);
  RET_CHECK(capture.isOpened());

  cv::namedWindow(kWindowName, cv::WINDOW_NORMAL);
#if (CV_MAJOR_VERSION >= 3) && (CV_MINOR_VERSION >= 2)
    capture.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    capture.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    capture.set(cv::CAP_PROP_FPS, 30);
#endif

  LOG(INFO) << "Start running the calculator graph.";
  ASSIGN_OR_RETURN(mediapipe::OutputStreamPoller poller_landmark,
                   graph.AddOutputStreamPoller(kOutputStream_Landmarks));
  ASSIGN_OR_RETURN(mediapipe::OutputStreamPoller poller_video,
                   graph.AddOutputStreamPoller(kOutputStream_Video));
  MP_RETURN_IF_ERROR(graph.StartRun({}));

  LOG(INFO) << "Start grabbing and processing frames.";
  bool grab_frames = true;
  double last_timestamp = 0.0;
  while (grab_frames) {
    // Capture opencv camera or video frame.
    cv::Mat camera_frame_raw;
    capture >> camera_frame_raw;
    if (camera_frame_raw.empty()) {
      LOG(INFO) << "Ignore empty frames from camera.";
      continue;
    }
    cv::Mat camera_frame;
    cv::cvtColor(camera_frame_raw, camera_frame, cv::COLOR_BGR2RGBA);
    cv::flip(camera_frame, camera_frame, /*flipcode=HORIZONTAL*/ 1);

    // Wrap Mat into an ImageFrame.
    auto input_frame = absl::make_unique<mediapipe::ImageFrame>(
        mediapipe::ImageFormat::SRGBA, camera_frame.cols, camera_frame.rows,
        mediapipe::ImageFrame::kGlDefaultAlignmentBoundary);
    cv::Mat input_frame_mat = mediapipe::formats::MatView(input_frame.get());
    camera_frame.copyTo(input_frame_mat);

    // Prepare and add graph input packet.
    size_t frame_timestamp_us =
        (double)cv::getTickCount() / (double)cv::getTickFrequency() * 1e6;
    MP_RETURN_IF_ERROR(
        gpu_helper.RunInGlContext([&input_frame, &frame_timestamp_us, &graph,
                                   &gpu_helper]() -> ::mediapipe::Status {
          // Convert ImageFrame to GpuBuffer.
          auto texture = gpu_helper.CreateSourceTexture(*input_frame.get());
          auto gpu_frame = texture.GetFrame<mediapipe::GpuBuffer>();
          glFlush();
          texture.Release();
          // Send GPU image packet into the graph.
          MP_RETURN_IF_ERROR(graph.AddPacketToInputStream(
              kInputStream, mediapipe::Adopt(gpu_frame.release())
                                .At(mediapipe::Timestamp(frame_timestamp_us))));
          return mediapipe::OkStatus();
        }));

    // Get the graph result packet, or stop if that fails.
    mediapipe::Packet packet_video;
    if (!poller_video.Next(&packet_video)) break;
    std::unique_ptr<mediapipe::ImageFrame> output_frame;
    // Convert GpuBuffer to ImageFrame.
    MP_RETURN_IF_ERROR(gpu_helper.RunInGlContext(
        [&packet_video, &output_frame, &gpu_helper]() -> ::mediapipe::Status {
          auto& gpu_frame = packet_video.Get<mediapipe::GpuBuffer>();
          auto texture = gpu_helper.CreateSourceTexture(gpu_frame);
          output_frame = absl::make_unique<mediapipe::ImageFrame>(
              mediapipe::ImageFormatForGpuBufferFormat(gpu_frame.format()),
              gpu_frame.width(), gpu_frame.height(),
              mediapipe::ImageFrame::kGlDefaultAlignmentBoundary);
          gpu_helper.BindFramebuffer(texture);
          const auto info =
              mediapipe::GlTextureInfoForGpuBufferFormat(gpu_frame.format(), 0);
          glReadPixels(0, 0, texture.width(), texture.height(), info.gl_format,
                       info.gl_type, output_frame->MutablePixelData());
          glFlush();
          texture.Release();
          return mediapipe::OkStatus();
        }));
    // Convert back to opencv.
    cv::Mat output_frame_mat = mediapipe::formats::MatView(output_frame.get());
    if (output_frame_mat.channels() == 4)
      cv::cvtColor(output_frame_mat, output_frame_mat, cv::COLOR_RGBA2BGR);
    else
      cv::cvtColor(output_frame_mat, output_frame_mat, cv::COLOR_RGB2BGR);

    // retrieve landmarks
    mediapipe::Packet packet_landmark;
    if (!poller_landmark.Next(&packet_landmark)) break;
    auto& output_list = packet_landmark.Get<mediapipe::NormalizedLandmarkList>();
    if (output_list.landmark_size() > 0) {
      // Show output list.
      auto& nose = output_list.landmark(0);
      int x_int = static_cast<int>(nose.x() * output_frame_mat.cols);
      int y_int = static_cast<int>(nose.y() * output_frame_mat.rows);
      cv::Point point_to_draw(x_int + 10, y_int);
      cv::putText(output_frame_mat, "nose!!!", point_to_draw, cv::FONT_HERSHEY_COMPLEX, 0.8, cv::Scalar(0, 0, 255), 2, cv::FILLED);
      // const cv::Scalar color = cv::Scalar(0, 255, 0);
      // cv::circle(camera_frame_raw, point_to_draw, 2, color, -1);
      // LOG(INFO) << "Nose Position: (" << x_int << ", " << y_int << ")";
    }

    double now = (double)cv::getTickCount() / (double)cv::getTickFrequency();
    if (last_timestamp > 0) {
      double fps = 1.0 / (now - last_timestamp);
      cv::Point text_org(0, 50);
      std::string text = "FPS: " + std::to_string(fps);
      cv::putText(output_frame_mat, text, text_org, cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 0, 0), 2, cv::FILLED);
    }
    last_timestamp = now;

    cv::imshow(kWindowName, output_frame_mat);
    // Press any key to exit.
    const int pressed_key = cv::waitKey(5);
    if (pressed_key >= 0 && pressed_key != 255) grab_frames = false;
  }

  LOG(INFO) << "Shutting down.";
  MP_RETURN_IF_ERROR(graph.CloseInputStream(kInputStream));
  return graph.WaitUntilDone();
}

int main(int argc, char** argv) {
  // google::InitGoogleLogging(argv[0]);
  // gflags::ParseCommandLineFlags(&argc, &argv, true);
  mediapipe::Status run_status = RunMPPGraph();
  if (!run_status.ok()) {
    LOG(ERROR) << "Failed to run the graph: " << run_status.message();
    return EXIT_FAILURE;
  } else {
    LOG(INFO) << "Success!";
  }
  return EXIT_SUCCESS;
}

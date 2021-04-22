// An example of sending Kinect Color frames into a MediaPipe graph.
#include <cstdlib>

#include "k4a/k4a.h"

#include "mediapipe/framework/calculator_framework.h"
#include "mediapipe/framework/formats/image_frame.h"
#include "mediapipe/framework/formats/image_frame_opencv.h"
#include "mediapipe/framework/port/commandlineflags.h"
#include "mediapipe/framework/port/file_helpers.h"
#include "mediapipe/framework/port/opencv_highgui_inc.h"
#include "mediapipe/framework/port/opencv_imgproc_inc.h"
#include "mediapipe/framework/port/opencv_video_inc.h"
#include "mediapipe/framework/port/parse_text_proto.h"
#include "mediapipe/framework/port/status.h"

constexpr char kWindowName[] = "KinectInput";
constexpr char kInputStream[] = "input_video";
constexpr char kOutputStream[] = "output_video";

DEFINE_string(
    calculator_graph_config_file, "",
    "Name of file containing text format CalculatorGraphConfig proto.");

mediapipe::Status RunMPPGraph() {
  std::string calculator_graph_config_contents;
  MP_RETURN_IF_ERROR(mediapipe::file::GetContents(
      FLAGS_calculator_graph_config_file, &calculator_graph_config_contents));
  LOG(INFO) << "Get calculator graph config contents: "
            << calculator_graph_config_contents;
  mediapipe::CalculatorGraphConfig config =
      mediapipe::ParseTextProtoOrDie<mediapipe::CalculatorGraphConfig>(
          calculator_graph_config_contents);

  LOG(INFO) << "Initialize the calculator graph.";
  mediapipe::CalculatorGraph graph;
  MP_RETURN_IF_ERROR(graph.Initialize(config));

  // Find Azure Kinect
  LOG(INFO) << "Going to Initialize Azure Kinect...";
  uint32_t count = k4a_device_get_installed_count();
  if (count == 0)
  {
      LOG(INFO) << "Cannot find Any Installed K4A device!";
      return mediapipe::OkStatus();
  }

  // Try to open the first device
  k4a_result_t ret;
  k4a_device_t kinect;
  ret = k4a_device_open(K4A_DEVICE_DEFAULT, &kinect);
  if (ret != K4A_RESULT_SUCCEEDED) {
      LOG(INFO) << "Fail to Open K4A device! Return Code: " << ret;
      return mediapipe::OkStatus();
  }

  // Config the device
  k4a_device_configuration_t k4a_config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
  k4a_config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32; 
  // k4a_config.color_resolution = K4A_COLOR_RESOLUTION_1080P;
  k4a_config.color_resolution = K4A_COLOR_RESOLUTION_2160P;
  k4a_config.camera_fps = K4A_FRAMES_PER_SECOND_30;
  k4a_config.depth_mode = K4A_DEPTH_MODE_OFF;

  // Start the camera with the given configuration
  ret = k4a_device_start_cameras(kinect, &k4a_config);
  if (ret != K4A_RESULT_SUCCEEDED)
  {
      LOG(INFO) << "Fail to start k4a camera! Return Code: " << ret;
      k4a_device_close(kinect);
      return mediapipe::OkStatus();
  }

  LOG(INFO) << "Start running the calculator graph.";
  ASSIGN_OR_RETURN(mediapipe::OutputStreamPoller poller,
                    graph.AddOutputStreamPoller(kOutputStream));
  MP_RETURN_IF_ERROR(graph.StartRun({}));

  cv::namedWindow(kWindowName, cv::WINDOW_NORMAL);
  k4a_image_t kinectColorFrame;
  cv::Mat imageRaw;
  k4a_capture_t sensor_capture;
  LOG(INFO) << "Start grabbing and processing frames...";
  while (true)
  {
      int32_t K4A_WAIT_TIMEOUT = 50;
      switch (k4a_device_get_capture(kinect, &sensor_capture, K4A_WAIT_TIMEOUT))
      {
          case K4A_WAIT_RESULT_SUCCEEDED:
            break;
          case K4A_WAIT_RESULT_TIMEOUT:
            LOG(INFO) << "Timed out waiting for a capture";
            continue;
            break;
          case K4A_WAIT_RESULT_FAILED:
            LOG(INFO) << "Failed to read a capture";
            goto Exit;
      }

      // Probe for a color image
      kinectColorFrame = k4a_capture_get_color_image(sensor_capture);
      if (kinectColorFrame)
      {
          int height = k4a_image_get_height_pixels(kinectColorFrame);
          int width = k4a_image_get_width_pixels(kinectColorFrame);
          int stride = k4a_image_get_stride_bytes(kinectColorFrame);
          
          uint8_t* imgData = k4a_image_get_buffer(kinectColorFrame);
          imageRaw = cv::Mat(height, width, CV_8UC4, imgData);
          if (imageRaw.empty()) {
              LOG(INFO) << "Transform Color Frame to empty cv::Mat";
              continue;
          }

          cv::Mat camera_frame;
          cv::cvtColor(imageRaw, camera_frame, cv::COLOR_BGRA2RGB);
          cv::flip(camera_frame, camera_frame, /*flipcode=HORIZONTAL*/ 1);
          k4a_image_release(kinectColorFrame);

          // Wrap Mat into an ImageFrame.
          auto input_frame = absl::make_unique<mediapipe::ImageFrame>(
              mediapipe::ImageFormat::SRGB, camera_frame.cols, camera_frame.rows,
              mediapipe::ImageFrame::kDefaultAlignmentBoundary);
          cv::Mat input_frame_mat = mediapipe::formats::MatView(input_frame.get());
          camera_frame.copyTo(input_frame_mat);

          // Send image packet into the graph.
          size_t frame_timestamp_us =
              (double)cv::getTickCount() / (double)cv::getTickFrequency() * 1e6;
          MP_RETURN_IF_ERROR(graph.AddPacketToInputStream(
              kInputStream, mediapipe::Adopt(input_frame.release())
                                .At(mediapipe::Timestamp(frame_timestamp_us))));

          // Get the graph result packet, or stop if that fails.
          mediapipe::Packet packet;
          if (!poller.Next(&packet)) break;
          auto& output_frame = packet.Get<mediapipe::ImageFrame>();

          // Convert back to opencv for display or saving.
          cv::Mat output_frame_mat = mediapipe::formats::MatView(&output_frame);
          cv::cvtColor(output_frame_mat, output_frame_mat, cv::COLOR_RGB2BGR);
          cv::imshow(kWindowName, output_frame_mat);

          // Press Esc or q to exit.
          const int pressed_key = cv::waitKey(5);
          if (pressed_key == 27 || pressed_key == 'q') {
              break;
          }
      }
      else
      {
          LOG(INFO) << "No Available Color Frame";
      }

      // release capture
      k4a_capture_release(sensor_capture);
  }

Exit:
  cv::destroyAllWindows();
  // Shut Down Device
  k4a_device_stop_cameras(kinect);
  k4a_device_close(kinect);

  LOG(INFO) << "Shutting down.";
  MP_RETURN_IF_ERROR(graph.CloseInputStream(kInputStream));
  return graph.WaitUntilDone();
}

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  mediapipe::Status run_status = RunMPPGraph();
  if (!run_status.ok()) {
    LOG(INFO) << "Failed to run the graph: " << run_status.message();
    return EXIT_FAILURE;
  } else {
    LOG(INFO) << "Success!";
  }
  return EXIT_SUCCESS;
}

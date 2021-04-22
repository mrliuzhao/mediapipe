#include <cstdlib>
#include <ctime>
#include <fstream>
#include <corecrt.h>
#include <corecrt_io.h>
#include <string>
#include <direct.h>
#include <sys/timeb.h>
#include <iomanip>

#include <windows.h>

#include "mediapipe/framework/calculator_framework.h"
#include "mediapipe/framework/formats/image_frame.h"
#include "mediapipe/framework/formats/image_frame_opencv.h"
#include "mediapipe/framework/formats/landmark.pb.h"
#include "mediapipe/framework/port/file_helpers.h"
#include "mediapipe/framework/port/opencv_core_inc.h"
#include "mediapipe/framework/port/opencv_highgui_inc.h"
#include "mediapipe/framework/port/opencv_imgproc_inc.h"
#include "mediapipe/framework/port/opencv_video_inc.h"
#include "mediapipe/framework/port/parse_text_proto.h"
#include "mediapipe/framework/port/status.h"

#include "k4a/k4a.h"

constexpr char kInputStream[] = "input_video";
constexpr char kOutputStream[] = "body_landmark_list";
constexpr char kWindowName[] = "BodyTracking";
std::ofstream logFile;

std::string getCurrentDir() {
  char szModuleFilePath[MAX_PATH];
  char SaveResult[MAX_PATH];
  int n = GetModuleFileNameA(0, szModuleFilePath, MAX_PATH);
  szModuleFilePath[ strrchr(szModuleFilePath, '\\') - szModuleFilePath + 1 ] = 0;
  strcpy(SaveResult, szModuleFilePath);
  return SaveResult;
}

UINT64 getCurrentTimeMillis()
{
    using namespace std;

    timeb now;
    ftime(&now);
    std::stringstream milliStream;
    milliStream << setw(3) << setfill('0') << right << now.millitm;

    stringstream secStream;
    secStream << now.time;
    string timeStr(secStream.str());
    timeStr.append(milliStream.str());

    UINT64 timeLong;
    stringstream transStream(timeStr);
    transStream >> timeLong;

    return timeLong;
}

std::string getNowTimeString(std::string fmtStr) {
    time_t timep;
    time(&timep);
    char tmp[256];
    strftime(tmp, sizeof(tmp), fmtStr.c_str(), localtime(&timep));
    return tmp;
}

void DrawPoint(int x, int y, uint16_t z, cv::Mat img){
  cv::Point point_to_draw(x, y);
  const cv::Scalar color = cv::Scalar(0, 255, 0);
  cv::circle(img, point_to_draw, 2, color, -1);
  // cv::putText(img, std::to_string(z), cv::Point(x + 3, y - 3), cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(255, 255, 255));
}

uint16_t findZValue(int x, int y, int width, int height, uint16_t *depth_data){
  uint16_t minZ = 0;
  for (int i = -1; i < 2; i++)
  {
    int row = y + i;
    if (row < 0 || row >= height) {
      continue;
    }
    for (int j = -1; j < 2; j++)
    {
      int col = x + j;
      if (col < 0 || col >= width)
      {
        continue;
      }
      int idx = row * width + col;
      uint16_t tempz = depth_data[idx];
      if (tempz > 0)
      {
        if (minZ == 0)
        {
          minZ = tempz;
        } else {
          minZ = (tempz < minZ) ? tempz : minZ;
        }
      }
    }
  }
  return minZ;
}

void LandmarkRecord(mediapipe::NormalizedLandmark landmark, 
                    std::string name, cv::Mat img, 
                    k4a_image_t depth_img,
                    const k4a_calibration_t *calibration){
  try{
    int height = img.rows;
    int width = img.cols;
    // const float x_factor = 0.707106781f * 2.0f / width;
    // const float y_factor = 0.492423560f * 2.0f / height;
    float x_raw = landmark.x() * width;
    float y_raw = landmark.y() * height;
    int x_int = static_cast<int>(x_raw);
    int y_int = static_cast<int>(y_raw);
    uint16_t z = 0;
    if (depth_img == NULL) {
      std::cout << "No transformed depth image!" << std::endl;
    } else {
      int depth_height = k4a_image_get_height_pixels(depth_img);
      int depth_width = k4a_image_get_width_pixels(depth_img);
      if (depth_height != height) {
        std::cout << "!!!!! depth_height(" << depth_height << ") != img_height(" << height << ") !!!!!" << std::endl;
      }
      if (depth_width != width) {
        std::cout << "!!!!! depth_width(" << depth_width << ") != img_width(" << width << ") !!!!!" << std::endl;
      }

      uint16_t *depth_data = (uint16_t *)(void *)k4a_image_get_buffer(depth_img);
      if (depth_data == NULL) {
        std::cout << "depthDataPtr == NULL!!!!!" << std::endl;
      } else {
        z = findZValue(x_int, y_int, depth_width, depth_height, depth_data);
        // int idx = y_int * depth_width + x_int;
        // z = depth_data[idx];
      }
    }
    // DrawPoint(x_int, y_int, z, img);

    // Map pixel x,y into milimeter x,y
    float x_proc_my = 0.0f;
    float y_proc_my = 0.0f;
    if (z == 0) {
      // logFile << "find 0 in z\n";
      // logFile.flush();
      logFile << name << " Position:(0,0,0)\n";
      logFile.flush();

    } else {
      // x_proc_my = (x_raw - (width / 2.0f)) * z * x_factor;
      // y_proc_my = (y_raw - (height / 2.0f)) * z * y_factor;
      // std::cout << "Myself " << name << " Position:(" << x_proc_my << "," << y_proc_my  << "," << z << ")\n";

      k4a_float2_t p2d;
      p2d.xy.x = x_raw;
      p2d.xy.y = y_raw;
      k4a_float3_t p3d;
      int valid;
      k4a_calibration_2d_to_3d(calibration, &p2d, (float) z,
                                K4A_CALIBRATION_TYPE_COLOR,
                                K4A_CALIBRATION_TYPE_COLOR,
                                &p3d, &valid);
      if (valid)
      {
        logFile << name << " Position:(" << p3d.xyz.x << "," << p3d.xyz.y  << "," << p3d.xyz.z << ")\n";
        logFile.flush();
      } else {
        logFile << "Error transform in kinect 2d to 3d\n";
        logFile.flush();
      }
    }
    // k4a_float2_t p;
    // k4a_float3_t ray;
    // int valid;
    // k4a_calibration_2d_to_3d(calibration, &p, (float) z, K4A_CALIBRATION_TYPE_DEPTH, K4A_CALIBRATION_TYPE_DEPTH, &ray, &valid);

    // logFile << name << " Position:(" << y_proc_my << "," << y_proc_my  << "," << z << ")\n";
    // logFile.flush();
  } catch (std::exception e) {
    std::cout << "Error occur: " << e.what() << std::endl;
  }
}

mediapipe::Status RunMPPGraph() {
  std::string curdir = getCurrentDir();
  std::cout << "Current Directory is " << curdir << std::endl;
  std::string nowTimeStr = getNowTimeString("%Y-%m-%d");
  std::string logDir = curdir + "logs";
  if (_access(logDir.c_str(), 0) != 0)
  {
      _mkdir(logDir.c_str());
  }

  if (!logFile.is_open())
  {
      std::string logFilePath = logDir + "\\kinect-" + nowTimeStr + ".log";
      logFile.open(logFilePath, std::ios::app);
  }

  std::string calculator_graph_config_contents = R"(
    input_stream: "input_video"
    output_stream: "body_landmark_list"
    node {
      calculator: "ConstantSidePacketCalculator"
      output_side_packet: "PACKET:0:upper_body_only"
      output_side_packet: "PACKET:1:smooth_landmarks"
      node_options: {
        [type.googleapis.com/mediapipe.ConstantSidePacketCalculatorOptions]: {
          packet { bool_value: false }
          packet { bool_value: true }
        }
      }
    }
    node {
      calculator: "HolisticLandmarkCpuNoFace"
      input_stream: "IMAGE:input_video"
      input_side_packet: "UPPER_BODY_ONLY:upper_body_only"
      input_side_packet: "SMOOTH_LANDMARKS:smooth_landmarks"
      output_stream: "POSE_LANDMARKS:pose_landmarks"
      output_stream: "POSE_ROI:pose_roi"
      output_stream: "POSE_DETECTION:pose_detection"
      output_stream: "LEFT_HAND_LANDMARKS:left_hand_landmarks"
      output_stream: "RIGHT_HAND_LANDMARKS:right_hand_landmarks"
    }
    node {
      calculator: "HolisticTrackingDataPostprocess"
      input_stream: "POSE_LANDMARKS:pose_landmarks"
      input_stream: "LEFT_HAND_LANDMARKS:left_hand_landmarks"
      input_stream: "RIGHT_HAND_LANDMARKS:right_hand_landmarks"
      input_side_packet: "UPPER_BODY_ONLY:upper_body_only"
      output_stream: "body_landmark_list"
    }
  )";
  mediapipe::CalculatorGraphConfig config =
      mediapipe::ParseTextProtoOrDie<mediapipe::CalculatorGraphConfig>(
          calculator_graph_config_contents);

  std::cout << "Initialize the calculator graph." << std::endl;
  mediapipe::CalculatorGraph graph;
  // MP_RETURN_IF_ERROR(graph.Initialize(config));
  mediapipe::Status mediaRet;
  mediaRet = graph.Initialize(config);
  if (!mediaRet.ok())
  {
      std::cout << "Fail to Initialize Mediapipe Graph!" << std::endl;
      return mediapipe::OkStatus();
  }

  // Find Azure Kinect
  std::cout << "Going to Initialize Azure Kinect..." << std::endl;
  uint32_t count = k4a_device_get_installed_count();
  if (count == 0)
  {
      std::cout << "Cannot find Any Installed K4A device!" << std::endl;
      return mediapipe::OkStatus();
  }

  // Try to open the first device
  k4a_result_t ret;
  k4a_device_t kinect;
  ret = k4a_device_open(K4A_DEVICE_DEFAULT, &kinect);
  if (ret != K4A_RESULT_SUCCEEDED) {
      std::cout << "Fail to Open K4A device! Return Code: " << ret << std::endl;
      return mediapipe::OkStatus();
  }

  // Config the device
  k4a_device_configuration_t k4a_config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
  k4a_config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32; 
  k4a_config.color_resolution = K4A_COLOR_RESOLUTION_1080P;
  // k4a_config.color_resolution = K4A_COLOR_RESOLUTION_1536P;  //  larger FOV
  k4a_config.camera_fps = K4A_FRAMES_PER_SECOND_30;
  // k4a_config.depth_mode = K4A_DEPTH_MODE_OFF;
  // k4a_config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
  k4a_config.depth_mode = K4A_DEPTH_MODE_WFOV_2X2BINNED;  //  larger FOV
  k4a_config.synchronized_images_only = true;

  // Try to transform data
  k4a_calibration_t calibration;
  ret = k4a_device_get_calibration(kinect, k4a_config.depth_mode, k4a_config.color_resolution, &calibration);
  if (ret != K4A_RESULT_SUCCEEDED)
  {
      std::cout << "Failed to get calibration! Return Code: " << ret << std::endl;
      return mediapipe::OkStatus();
  }

  k4a_transformation_t transformation = k4a_transformation_create(&calibration);
  if (transformation == NULL) {
      std::cout << "Failed to create transformation! Return Code: " << ret << std::endl;
      return mediapipe::OkStatus();
  }

  // Start the camera with the given configuration
  ret = k4a_device_start_cameras(kinect, &k4a_config);
  if (ret != K4A_RESULT_SUCCEEDED)
  {
      std::cout << "Fail to start k4a camera! Return Code: " << ret << std::endl;
      k4a_device_close(kinect);
      return mediapipe::OkStatus();
  }

  std::cout << "Start running the calculator graph.";
  // ASSIGN_OR_RETURN(mediapipe::OutputStreamPoller poller,
  //                   graph.AddOutputStreamPoller(kOutputStream));
  mediapipe::StatusOrPoller status_or_poller = graph.AddOutputStreamPoller(kOutputStream);
  if (!status_or_poller.ok()) {
    std::cout << "Error assign output Stream Poller.";
    return status_or_poller.status();
  }
  mediapipe::OutputStreamPoller poller = std::move(status_or_poller.ValueOrDie());

  // MP_RETURN_IF_ERROR(graph.StartRun({}));
  mediaRet = graph.StartRun({});
  if (!mediaRet.ok())
  {
      std::cout << "Fail to Run Mediapipe Graph!" << std::endl;
      return mediapipe::OkStatus();
  }

  k4a_image_t kinectColorFrame = NULL;
  k4a_image_t kinectDepthFrame = NULL;
  k4a_image_t transformed_depth_image = NULL;
  cv::Mat imageRaw;
  cv::namedWindow(kWindowName, cv::WINDOW_NORMAL);
  cv::namedWindow("depthImage", cv::WINDOW_NORMAL);
  std::cout << "Start grabbing and processing frames..." << std::endl;
  int frameCount = 1800;
  while (frameCount > 0)
  {
      k4a_capture_t sensor_capture;
      int32_t K4A_WAIT_TIMEOUT = 5;
      switch (k4a_device_get_capture(kinect, &sensor_capture, K4A_WAIT_TIMEOUT))
      {
          case K4A_WAIT_RESULT_SUCCEEDED:
            break;
          case K4A_WAIT_RESULT_TIMEOUT:
            // std::cout << "Timed out waiting for a capture" << std::endl;
            continue;
          case K4A_WAIT_RESULT_FAILED:
            std::cout << "Failed to read a capture" << std::endl;
            goto Exit;
      }

      UINT64 start = getCurrentTimeMillis();
      // Probe for a color image
      kinectColorFrame = k4a_capture_get_color_image(sensor_capture);
      if (!kinectColorFrame){
          std::cout << "Failed to get color image from capture" << std::endl;
          continue;
      }
      int color_width = k4a_image_get_width_pixels(kinectColorFrame);
      int color_height = k4a_image_get_height_pixels(kinectColorFrame);

      // Probe for a depth image
      kinectDepthFrame = k4a_capture_get_depth_image(sensor_capture);
      if (!kinectDepthFrame)
      {
          std::cout << "Failed to get depth image from capture" << std::endl;
          continue;
      }

      // Transform Depth Data into the geometry of the Color camera
      ret = k4a_image_create(K4A_IMAGE_FORMAT_DEPTH16, 
                              color_width, color_height, 
                              (color_width * (int)sizeof(uint16_t)), 
                              &transformed_depth_image);
      if (ret != K4A_RESULT_SUCCEEDED)
      {
          std::cout << "Failed to create transformed depth image! Return code: " << ret << std::endl;
          continue;
      }
      ret = k4a_transformation_depth_image_to_color_camera(transformation, kinectDepthFrame, transformed_depth_image);
      if (ret != K4A_RESULT_SUCCEEDED)
      {
          std::cout << "Failed to compute transformed depth image! Return code: " << ret << std::endl;
          continue;
      }
      // std::cout << "Before Transform, Depth Frame Size: " << k4a_image_get_width_pixels(kinectDepthFrame) 
      // << " x " << k4a_image_get_height_pixels(kinectDepthFrame) 
      // << "; After Transform, Depth Frame Size: " << k4a_image_get_width_pixels(transformed_depth_image) 
      // << " x " << k4a_image_get_height_pixels(transformed_depth_image) << std::endl;

      // Display Transformed Depth Frame
      cv::Mat depthImg;
      if (transformed_depth_image != NULL) {
        uint8_t* depthDataPtr = k4a_image_get_buffer(transformed_depth_image);
        if (depthDataPtr == NULL) {
          std::cout << "depthDataPtr == NULL!!!!!" << std::endl;
        }
        if (depthDataPtr != NULL)
        {
          depthImg = cv::Mat(k4a_image_get_height_pixels(transformed_depth_image), 
                              k4a_image_get_width_pixels(transformed_depth_image), 
                              CV_16UC1, depthDataPtr);
          cv::imshow("depthImage", depthImg);
        }
      }

      // Pack Color Frame into OpenCV Mat
      uint8_t* imgData = k4a_image_get_buffer(kinectColorFrame);
      imageRaw = cv::Mat(color_height, color_width, CV_8UC4, imgData);
      if (imageRaw.empty()) {
          std::cout << "Transform Color Frame to empty cv::Mat" << std::endl;
          continue;
      }
      // cv::flip(imageRaw, imageRaw, /*flipcode=HORIZONTAL*/ 1);

      cv::Mat camera_frame;
      cv::cvtColor(imageRaw, camera_frame, cv::COLOR_BGRA2RGB);
      // Wrap Mat into an ImageFrame.
      auto input_frame = absl::make_unique<mediapipe::ImageFrame>(
          mediapipe::ImageFormat::SRGB, camera_frame.cols, camera_frame.rows,
          mediapipe::ImageFrame::kDefaultAlignmentBoundary);
      cv::Mat input_frame_mat = mediapipe::formats::MatView(input_frame.get());
      camera_frame.copyTo(input_frame_mat);

      // Send image packet into the graph.
      size_t frame_timestamp_us =
          (double)cv::getTickCount() / (double)cv::getTickFrequency() * 1e6;
      // MP_RETURN_IF_ERROR(graph.AddPacketToInputStream(
      //     kInputStream, mediapipe::Adopt(input_frame.release())
      //                       .At(mediapipe::Timestamp(frame_timestamp_us))));
      mediaRet = graph.AddPacketToInputStream(kInputStream, 
                      mediapipe::Adopt(input_frame.release())
                      .At(mediapipe::Timestamp(frame_timestamp_us)));
      if (!mediaRet.ok())
      {
        std::cout << "Fail to send image packet into the graph! Error Code: " << mediaRet.code()
        << "; Error Message: " << mediaRet.message() << "\n";
        continue;
      }

      // Get the graph result packet, or stop if that fails.
      mediapipe::Packet packet;
      if (!poller.Next(&packet)) break;
      auto& output_list = packet.Get<mediapipe::NormalizedLandmarkList>();
      // std::cout << "output_list size: " << output_list.landmark_size();
      UINT64 end = getCurrentTimeMillis();
      std::cout << "Calculation Time: " << (end - start) << std::endl;
      if (output_list.landmark_size() == 0) {
        logFile << "Fail to detect all landmarks\n";
        logFile.flush();
      }
      if (output_list.landmark_size() > 0) {
        // Show output list.
        auto& leftWrist = output_list.landmark(15);
        // LandmarkRecord(leftWrist, "LeftWrist", imageRaw, transformed_depth_image, &calibration);
        logFile << "LeftWrist Position:(" << leftWrist.x() << "," << leftWrist.y()  << "," << leftWrist.z() << ")\n";
        logFile.flush();

        auto& rightWrist = output_list.landmark(16);
        // LandmarkRecord(rightWrist, "RightWrist", imageRaw, transformed_depth_image, &calibration);
        logFile << "RightWrist Position:(" << rightWrist.x() << "," << rightWrist.y()  << "," << rightWrist.z() << ")\n";
        logFile.flush();

        auto& leftThumb = output_list.landmark(27);
        // LandmarkRecord(leftThumb, "LeftThumb", imageRaw, transformed_depth_image, &calibration);
        logFile << "LeftThumb Position:(" << leftThumb.x() << "," << leftThumb.y()  << "," << leftThumb.z() << ")\n";
        logFile.flush();

        auto& leftPalm = output_list.landmark(28);
        // LandmarkRecord(leftPalm, "LeftPalm", imageRaw, transformed_depth_image, &calibration);
        logFile << "LeftPalm Position:(" << leftPalm.x() << "," << leftPalm.y()  << "," << leftPalm.z() << ")\n";
        logFile.flush();

        auto& leftTip = output_list.landmark(29);
        // LandmarkRecord(leftTip, "LeftTip", imageRaw, transformed_depth_image, &calibration);
        logFile << "LeftTip Position:(" << leftTip.x() << "," << leftTip.y()  << "," << leftTip.z() << ")\n";
        logFile.flush();

        auto& rightThumb = output_list.landmark(30);
        // LandmarkRecord(rightThumb, "RightThumb", imageRaw, transformed_depth_image, &calibration);
        logFile << "RightThumb Position:(" << rightThumb.x() << "," << rightThumb.y()  << "," << rightThumb.z() << ")\n";
        logFile.flush();

        auto& rightPalm = output_list.landmark(31);
        // LandmarkRecord(rightPalm, "RightPalm", imageRaw, transformed_depth_image, &calibration);
        logFile << "RightPalm Position:(" << rightPalm.x() << "," << rightPalm.y()  << "," << rightPalm.z() << ")\n";
        logFile.flush();

        auto& rightTip = output_list.landmark(32);
        // LandmarkRecord(rightTip, "RightTip", imageRaw, transformed_depth_image, &calibration);
        logFile << "RightTip Position:(" << rightTip.x() << "," << rightTip.y()  << "," << rightTip.z() << ")\n";
        logFile.flush();
      }

      cv::imshow(kWindowName, imageRaw);
      frameCount--;

      // Press Esc or q to exit.
      const int pressed_key = cv::waitKey(5);
      if (pressed_key == 27 || pressed_key == 'q') {
          break;
      }

      // release capture
      if(kinectColorFrame != NULL){
        k4a_image_release(kinectColorFrame);
      }
      if(kinectDepthFrame != NULL){
        k4a_image_release(kinectDepthFrame);
      }
      if (transformed_depth_image != NULL) {
        k4a_image_release(transformed_depth_image);
      }
      if (sensor_capture != NULL) {
        k4a_capture_release(sensor_capture);
      }
  }

Exit:
  if(kinectColorFrame != NULL){
    k4a_image_release(kinectColorFrame);
  }
  if(kinectDepthFrame != NULL){
    k4a_image_release(kinectDepthFrame);
  }
  if (transformed_depth_image != NULL) {
    k4a_image_release(transformed_depth_image);
  }
  if (transformation != NULL) {
    k4a_transformation_destroy(transformation);
  }
  // Shut Down Device
  cv::destroyAllWindows();
  k4a_device_stop_cameras(kinect);
  k4a_device_close(kinect);

  std::cout << "Shutting down." << std::endl;
  // MP_RETURN_IF_ERROR(graph.CloseInputStream(kInputStream));
  mediaRet = graph.CloseInputStream(kInputStream);
  if (!mediaRet.ok())
  {
    std::cout << "Fail to Close Input Stream!" << std::endl;
  }
  return graph.WaitUntilDone();
}

int main(int argc, char** argv) {
  // google::InitGoogleLogging(argv[0]);
  // gflags::ParseCommandLineFlags(&argc, &argv, true);
  try{
    mediapipe::Status run_status = RunMPPGraph();
    if (!run_status.ok()) {
      std::cout << "Failed to run the graph: " << run_status.message();
      return EXIT_FAILURE;
    } else {
      std::cout << "Success!";
    }
    return EXIT_SUCCESS;
  } catch (std::exception e) {
    std::cout << "Error occur: " << e.what() << std::endl;
    return EXIT_FAILURE;
  }
}


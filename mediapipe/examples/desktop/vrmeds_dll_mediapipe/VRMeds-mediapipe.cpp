#include "com_gdcccn_vrmeds_kinect_KinectCommunicator.h"
#include "VRMeds-mediapipe.h"
#pragma comment(lib,"ws2_32.lib")

#undef StrCat

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

/** Wait Timeout in ms.
 */
#define K4A_WAIT_TIMEOUT (20)

SOCKET clientSocket = NULL;
bool keepOutput = true;
std::ofstream logFile;

k4a_device_t kinect = NULL;
k4a_device_configuration_t k4a_config;
k4a_calibration_t sensor_calibration;
k4a_transformation_t transformation = NULL;

mediapipe::CalculatorGraph graph;

constexpr char kInputStream[] = "input_video";
constexpr char kOutputStream[] = "body_landmark_list";

float findZValue(int x, int y, int width, int height, uint16_t *depth_data){
  uint16_t minZ = 0;
  for (int i = -2; i < 3; i++)
  {
    int row = y + i;
    if (row < 0 || row >= height) {
      continue;
    }
    for (int j = -2; j < 3; j++)
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
  return (float) minZ;
}


k4a_float3_t LandmarkTransform(float x_raw, float y_raw, 
                               k4a_image_t depth_img, float lastZVal){
    k4a_float3_t p3d;
    p3d.xyz.x = 0.0f;
    p3d.xyz.y = 0.0f;
    p3d.xyz.z = 0.0f;

    int height = k4a_image_get_height_pixels(depth_img);
    int width = k4a_image_get_width_pixels(depth_img);
    float x_fl = x_raw * width;
    float y_fl = y_raw * height;
    int x_int = static_cast<int>(x_fl);
    int y_int = static_cast<int>(y_fl);
    float z = 0.0f;
    uint16_t *depth_data = (uint16_t *)(void *) k4a_image_get_buffer(depth_img);
    if (depth_data == NULL) {
        logFile << "depthDataPtr == NULL!!!!!\n";
        logFile.flush();
        return p3d;
    } 

    z = findZValue(x_int, y_int, width, height, depth_data);
    if (z == 0) {
        logFile << "Cannot find valid z value\n";
        logFile.flush();
        if (lastZVal == 0) {
            return p3d;
        } else {
            z = lastZVal;
        }
    } 
    
    k4a_float2_t p2d;
    p2d.xy.x = x_fl;
    p2d.xy.y = y_fl;
    int valid;
    k4a_calibration_2d_to_3d(&sensor_calibration, &p2d, z,
                            K4A_CALIBRATION_TYPE_COLOR,
                            K4A_CALIBRATION_TYPE_COLOR,
                            &p3d, &valid);
    if (!valid)
    {
        logFile << "Error transform in kinect 2d to 3d\n";
        logFile.flush();
        p3d.xyz.x = 0.0f;
        p3d.xyz.y = 0.0f;
        p3d.xyz.z = 0.0f;
    }
    return p3d;
}


k4a_float3_t HandLandmarkTransform(mediapipe::NormalizedLandmark &handLandmark, 
                                   k4a_float3_t &wristJoint,
                                   k4a_image_t depth_img, float lastZVal){
    k4a_float3_t p3d;
    p3d.xyz.x = 0.0f;
    p3d.xyz.y = 0.0f;
    p3d.xyz.z = 0.0f;

    int height = k4a_image_get_height_pixels(depth_img);
    int width = k4a_image_get_width_pixels(depth_img);
    float x_raw = handLandmark.x();
    float y_raw = handLandmark.y();
    float z_raw = handLandmark.z();
    float x_fl = x_raw * width;
    float y_fl = y_raw * height;

    float wristZ = wristJoint.xyz.z;
    float z = 0.0f;
    if (wristZ == 0) {
        int x_int = static_cast<int>(x_fl);
        int y_int = static_cast<int>(y_fl);
        uint16_t *depth_data = (uint16_t *)(void *) k4a_image_get_buffer(depth_img);
        if (depth_data == NULL) {
            logFile << "depthDataPtr == NULL!!!!!\n";
            logFile.flush();
            return p3d;
        }

        z = findZValue(x_int, y_int, width, height, depth_data);
        if (z == 0) {
            logFile << "Cannot find valid z value\n";
            logFile.flush();
            if (lastZVal == 0) {
                return p3d;
            } else {
                z = lastZVal;
            }
        } 
    }
    else {
        z = wristZ + (z_raw * width);
    }
    
    k4a_float2_t p2d;
    p2d.xy.x = x_fl;
    p2d.xy.y = y_fl;
    int valid;
    k4a_calibration_2d_to_3d(&sensor_calibration, &p2d, z,
                            K4A_CALIBRATION_TYPE_COLOR,
                            K4A_CALIBRATION_TYPE_COLOR,
                            &p3d, &valid);
    if (!valid)
    {
        logFile << "Error transform in kinect 2d to 3d\n";
        logFile.flush();
        p3d.xyz.x = 0.0f;
        p3d.xyz.y = 0.0f;
        p3d.xyz.z = 0.0f;
    }
    return p3d;
}


// initialize log file
void initLogFile() {
    if (logFile.is_open()) {
        logFile.close();
    }

    std::string curdir = getCurrentDir();
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
}


JNIEXPORT jint JNICALL Java_com_gdcccn_vrmeds_kinect_KinectCommunicator_initializeKinect
(JNIEnv *, jobject, jint timeout)
{
    initLogFile();
    logFile << "-----------------------------------------"
            << getCurrentTimeString()
            << "-----------------------------------------\n";
    logFile << "Start initializeKinect...\n";
    logFile << "Timeout setting is: " << timeout << "\n";
    logFile.flush();

    // get installed Kinect count
    uint32_t count = k4a_device_get_installed_count();
    if (count == 0)
    {
        logFile << "Cannot find Any Installed K4A device!\n";
        logFile.flush();
        return -1;
    }

    k4a_result_t ret;
    ret = k4a_device_open(K4A_DEVICE_DEFAULT, &kinect);
    if (ret != K4A_RESULT_SUCCEEDED) {
        logFile << "Fail to Open K4A device! Return Code: " << ret << "\n";
        logFile.flush();
        return -1;
    }

    k4a_config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    k4a_config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
    // k4a_config.depth_mode = K4A_DEPTH_MODE_WFOV_2X2BINNED;  //  larger FOV
    k4a_config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32; 
    k4a_config.color_resolution = K4A_COLOR_RESOLUTION_1080P;
    k4a_config.camera_fps = K4A_FRAMES_PER_SECOND_30;
    k4a_config.synchronized_images_only = true;

    ret = k4a_device_get_calibration(kinect, k4a_config.depth_mode, k4a_config.color_resolution, &sensor_calibration);
    if (ret != K4A_RESULT_SUCCEEDED)
    {
        logFile << "Fail to Get Sensor Calibration! Return Code: " << ret << "\n";
        logFile.flush();
        k4a_device_close(kinect);
        return -1;
    }
    transformation = k4a_transformation_create(&sensor_calibration);
    if (transformation == NULL) {
        logFile << "Failed to create transformation! Return Code: " << ret << "\n";
        logFile.flush();
        k4a_device_close(kinect);
        return -1;
    }

    return 0;
}


JNIEXPORT jint JNICALL Java_com_gdcccn_vrmeds_kinect_KinectCommunicator_openKinectSocketClient
(JNIEnv *env, jobject, jstring jIP, jint jPort)
{
    initLogFile();
    logFile << "-----------------------------------------"
            << getCurrentTimeString()
            << "-----------------------------------------\n";
    logFile << "Start openKinectSocketClient...\n";
    logFile << "Port Setting is: " << jPort << "\n";
    logFile.flush();

    const char* IPStr;
    IPStr = env->GetStringUTFChars(jIP, false);
    if (IPStr == NULL)
    {
        logFile << "Invalid Host IP\n";
        logFile.flush();
        return -1;
    }
    logFile << "IP Setting is: " << IPStr << "\n";
    logFile.flush();

    WORD sockVersion = MAKEWORD(2, 2);
    WSADATA data;
    if (WSAStartup(sockVersion, &data) != 0)
    {
        logFile << "Failed calling WSAStartup \n";
        logFile.flush();
        return -2;
    }

    clientSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (clientSocket == INVALID_SOCKET || clientSocket == NULL)
    {
        logFile << "Failed creating client socket \n";
        logFile.flush();
        return -2;
    }

    sockaddr_in serAddr;
    serAddr.sin_family = AF_INET;
    serAddr.sin_port = htons(jPort);
    int r = inet_pton(AF_INET, IPStr, &serAddr.sin_addr);
    if (r != 1)
    {
        logFile << "Failed converting IP address to socket address \n";
        logFile.flush();
        return -2;
    }

    if (connect(clientSocket, (sockaddr *)&serAddr, sizeof(serAddr)) == SOCKET_ERROR)
    {
        logFile << "Failed connecting to server \n";
        logFile.flush();
        closesocket(clientSocket);
        return -3;
    }

    env->ReleaseStringUTFChars(jIP, IPStr);

    logFile << "Successfully open client socket \n";
    logFile.flush();
    return 0;
}


JNIEXPORT jint JNICALL Java_com_gdcccn_vrmeds_kinect_KinectCommunicator_detectKinectSensorFloorAngle
(JNIEnv *, jobject)
{
    initLogFile();
    logFile << "-----------------------------------------"
            << getCurrentTimeString()
            << "-----------------------------------------\n";
    logFile << "Start detectKinectSensorFloorAngle...\n";
    logFile.flush();

    logFile << "Now Detect Floor Angle cannot be fetched from Azure Kinect DK ...\n";
    logFile.flush();
    return -1;
}


JNIEXPORT jint JNICALL Java_com_gdcccn_vrmeds_kinect_KinectCommunicator_getBodyJoints
(JNIEnv *, jobject)
{
    initLogFile();
    logFile << "-----------------------------------------"
        << getCurrentTimeString()
        << "-----------------------------------------\n";
    logFile << "Start getBodyJoints...\n";
    logFile.flush();

    // Initialize Compute Graph
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
    logFile << "Initialize the calculator graph...\n" << std::endl;
    logFile.flush();
    mediapipe::Status mediaRet;
    mediaRet = graph.Initialize(config);
    if (!mediaRet.ok())
    {
        logFile << "Fail to Initialize Mediapipe Graph! Error code: "
            << mediaRet.code() << "; Error Message: " << mediaRet.message() <<"\n";
        logFile.flush();
        return -1;
    }

    logFile << "Start running the calculator graph...\n";
    logFile.flush();
    mediapipe::StatusOrPoller status_or_poller = graph.AddOutputStreamPoller(kOutputStream);
    if (!status_or_poller.ok()) {
        logFile << "Fail to assign output stream poller! Error code: "
            << mediaRet.code() << "; Error Message: " << mediaRet.message() <<"\n";
        logFile.flush();
        return -1;
    }
    mediapipe::OutputStreamPoller poller = std::move(status_or_poller.ValueOrDie());

    mediaRet = graph.StartRun({});
    if (!mediaRet.ok())
    {
        logFile << "Fail to Run Mediapipe Graph! Error code: "
            << mediaRet.code() << "; Error Message: " << mediaRet.message() <<"\n";
        logFile.flush();
        return -1;
    }

    // Start the camera with the given configuration
    logFile << "Start Kinect Cameras...\n";
    logFile.flush();
    k4a_result_t ret = k4a_device_start_cameras(kinect, &k4a_config);
    if (ret != K4A_RESULT_SUCCEEDED)
    {
        logFile << "Fail to start k4a cameras! Return Code: " << ret << "\n";
        logFile.flush();
        return -1;
    }

    keepOutput = true;
    bool socketSuccess = true;
    bool getCaptureFail = false;
    int timeoutCount = 0;
    k4a_image_t kinectColorFrame = NULL;
    k4a_image_t kinectDepthFrame = NULL;
    k4a_image_t transformed_depth_image = NULL;
    int sendRes = 0;
    skeleton_main lastSkeleton;
    lastSkeleton.valid = false;
    lastSkeleton.timeStamp = 0;
    while (keepOutput && socketSuccess)
    {
        // get Kinect capture
        k4a_capture_t sensor_capture = NULL;
        switch (k4a_device_get_capture(kinect, &sensor_capture, K4A_WAIT_TIMEOUT))
        {
            case K4A_WAIT_RESULT_SUCCEEDED:
                timeoutCount = 0;
                break;
            case K4A_WAIT_RESULT_TIMEOUT:
                // logFile << "Timed out waiting for a capture!\n";
                // logFile.flush();
                timeoutCount++;
                if (timeoutCount > 50) {
                    logFile << "Timed out too many times!\n";
                    logFile.flush();
                    getCaptureFail = true;
                    keepOutput = false;
                }
                continue;
            case K4A_WAIT_RESULT_FAILED:
                logFile << "Fail to get capture from Kinect!\n";
                logFile.flush();
                getCaptureFail = true;
                keepOutput = false;
                continue;
        }

        // Probe for a color image
        kinectColorFrame = k4a_capture_get_color_image(sensor_capture);
        if (!kinectColorFrame){
            logFile << "Failed to get color image from capture!\n";
            logFile.flush();
            continue;
        }
        int color_width = k4a_image_get_width_pixels(kinectColorFrame);
        int color_height = k4a_image_get_height_pixels(kinectColorFrame);

        // Probe for a depth image
        kinectDepthFrame = k4a_capture_get_depth_image(sensor_capture);
        if (!kinectDepthFrame)
        {
            logFile << "Failed to get depth image from capture!\n";
            logFile.flush();
            continue;
        }
        // Transform Depth Data into the geometry of the Color camera
        ret = k4a_image_create(K4A_IMAGE_FORMAT_DEPTH16, 
                                color_width, color_height, 
                                (color_width * (int)sizeof(uint16_t)), 
                                &transformed_depth_image);
        if (ret != K4A_RESULT_SUCCEEDED)
        {
            logFile << "Failed to create transformed depth image! Return code: " << ret << "\n";
            logFile.flush();
            continue;
        }
        ret = k4a_transformation_depth_image_to_color_camera(transformation, kinectDepthFrame, transformed_depth_image);
        if (ret != K4A_RESULT_SUCCEEDED)
        {
            logFile << "Failed to compute transformed depth image! Return code: " << ret << "\n";
            logFile.flush();
            continue;
        }

        // Pack Color Frame into OpenCV Mat
        uint8_t* imgData = k4a_image_get_buffer(kinectColorFrame);
        cv::Mat imageRaw;
        imageRaw = cv::Mat(color_height, color_width, CV_8UC4, imgData);
        if (imageRaw.empty()) {
            logFile << "Transform Color Frame to empty cv::Mat!\n";
            logFile.flush();
            continue;
        }
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
        mediaRet = graph.AddPacketToInputStream(kInputStream, 
                        mediapipe::Adopt(input_frame.release())
                        .At(mediapipe::Timestamp(frame_timestamp_us)));
        if (!mediaRet.ok())
        {
            logFile << "Fail to send image packet into the graph! Error Code: " << mediaRet.code()
                    << "; Error Message: " << mediaRet.message() << "\n";
            logFile.flush();
            getCaptureFail = true;
            keepOutput = false;
            continue;
        }

        // Get the graph result packet, or stop if that fails.
        mediapipe::Packet packet;
        if (!poller.Next(&packet)) {
            logFile << "Fail to poll next result from the graph! Error code: "
                << mediaRet.code() << "; Error Message: " << mediaRet.message() <<"\n";
            logFile.flush();
            getCaptureFail = true;
            keepOutput = false;
            continue;
        }
        mediapipe::NormalizedLandmarkList output_list = packet.Get<mediapipe::NormalizedLandmarkList>();
        if (output_list.landmark_size() == 0) {
            if (!lastSkeleton.valid) {
                // send 0 if no body
                char temp = { 0 };
                sendRes = send(clientSocket, &temp, sizeof(temp), 0);
                logFile << "Fail to detect all landmarks and no valid lastSkeleton\n";
                logFile.flush();
            } else {
                lastSkeleton.timeStamp = getCurrentTimeMillis();
                sendRes = SendJoints(lastSkeleton);
                lastSkeleton.reuseCount = lastSkeleton.reuseCount + 1;
                if (lastSkeleton.reuseCount >= 3) {
                    lastSkeleton.valid = false;
                }
            }
        } 
        else {
            // Transform MediaPipe data to MainService data
            skeleton_main newSkeleton;
            for (size_t i = 0; i < JOINT_COUNT_MAIN; i++)
            {
                float lastZVal = 0.0f;
                if (lastSkeleton.valid) {
                    lastZVal = lastSkeleton.jointsPos[i].xyz.z;
                }
                switch (i)
                {
                case SPINEBASE_MAIN:
                {
                    mediapipe::NormalizedLandmark hipLeft = output_list.landmark(HIPLEFT_MEDIAPIPE);
                    mediapipe::NormalizedLandmark hipRight = output_list.landmark(HIPRIGHT_MEDIAPIPE);
                    float spinebase_x = (hipRight.x() + hipLeft.x()) / 2.0f;
                    float spinebase_y = (hipRight.y() + hipLeft.y()) / 2.0f;
                    k4a_float3_t p3d =  LandmarkTransform(spinebase_x, spinebase_y,
                                                          transformed_depth_image,
                                                          lastZVal);
                    newSkeleton.jointsPos[i] = p3d;
                }
                    break;
                case SPINEMID_MAIN:
                {
                    mediapipe::NormalizedLandmark hipLeft = output_list.landmark(HIPLEFT_MEDIAPIPE);
                    mediapipe::NormalizedLandmark hipRight = output_list.landmark(HIPRIGHT_MEDIAPIPE);
                    mediapipe::NormalizedLandmark shoulderRight = output_list.landmark(SHOULDERRIGHT_MEDIAPIPE);
                    mediapipe::NormalizedLandmark shoulderLeft = output_list.landmark(SHOULDERLEFT_MEDIAPIPE);
                    float a1 = (shoulderRight.y() - hipLeft.y()) / (shoulderRight.x() - hipLeft.x());
                    float b1 = (shoulderRight.x() * hipLeft.y() - shoulderRight.y() * hipLeft.x()) 
                                / (shoulderRight.x() - hipLeft.x());
                    float a2 = (shoulderLeft.y() - hipRight.y()) / (shoulderLeft.x() - hipRight.x());
                    float b2 = (shoulderLeft.x() * hipRight.y() - shoulderLeft.y() * hipRight.x()) 
                                / (shoulderLeft.x() - hipRight.x());
                    float spinemid_x = (b2 - b1) / (a1 - a2);
                    float spinemid_y = (a1 * b2 - a2 * b1) / (a1 - a2);
                    k4a_float3_t p3d =  LandmarkTransform(spinemid_x, spinemid_y,
                                                          transformed_depth_image,
                                                          lastZVal);
                    newSkeleton.jointsPos[i] = p3d;
                }
                    break;
                case NECK_MAIN:
                {
                    mediapipe::NormalizedLandmark shoulderRight = output_list.landmark(SHOULDERRIGHT_MEDIAPIPE);
                    mediapipe::NormalizedLandmark shoulderLeft = output_list.landmark(SHOULDERLEFT_MEDIAPIPE);
                    mediapipe::NormalizedLandmark mouthRight = output_list.landmark(MOUTH_RIGHT_MEDIAPIPE);
                    mediapipe::NormalizedLandmark mouthLeft = output_list.landmark(MOUTH_LEFT_MEDIAPIPE);
                    float neck_x = (shoulderRight.x() + shoulderLeft.x() + mouthRight.x() + mouthLeft.x()) / 4.0f;
                    float neck_y = (shoulderRight.y() + shoulderLeft.y() + mouthRight.y() + mouthLeft.y()) / 4;
                    k4a_float3_t p3d =  LandmarkTransform(neck_x, neck_y,
                                                          transformed_depth_image,
                                                          lastZVal);
                    newSkeleton.jointsPos[i] = p3d;
                }
                    break;
                case HEAD_MAIN:
                {
                    mediapipe::NormalizedLandmark landmark = output_list.landmark(NOSE_MEDIAPIPE);
                    k4a_float3_t p3d =  LandmarkTransform(landmark.x(), landmark.y(),
                                                          transformed_depth_image,
                                                          lastZVal);
                    newSkeleton.jointsPos[i] = p3d;
                }
                    break;
                case SHOULDERLEFT_MAIN:
                {
                    mediapipe::NormalizedLandmark landmark = output_list.landmark(SHOULDERLEFT_MEDIAPIPE);
                    k4a_float3_t p3d =  LandmarkTransform(landmark.x(), landmark.y(),
                                                          transformed_depth_image,
                                                          lastZVal);
                    newSkeleton.jointsPos[i] = p3d;
                }
                    break;
                case ELBOWLEFT_MAIN:
                {
                    mediapipe::NormalizedLandmark landmark = output_list.landmark(ELBOWLEFT_MEDIAPIPE);
                    k4a_float3_t p3d =  LandmarkTransform(landmark.x(), landmark.y(),
                                                          transformed_depth_image,
                                                          lastZVal);
                    newSkeleton.jointsPos[i] = p3d;
                }
                    break;
                case WRISTLEFT_MAIN:
                {
                    mediapipe::NormalizedLandmark landmark = output_list.landmark(WRISTLEFT_MEDIAPIPE);
                    k4a_float3_t p3d =  LandmarkTransform(landmark.x(), landmark.y(),
                                                          transformed_depth_image,
                                                          lastZVal);
                    newSkeleton.jointsPos[i] = p3d;
                }
                    break;
                case HANDLEFT_MAIN:
                {
                    mediapipe::NormalizedLandmark handLeftLandmark = output_list.landmark(HANDLEFT_MEDIAPIPE);
                    k4a_float3_t p3d =  HandLandmarkTransform(handLeftLandmark,
                                                              newSkeleton.jointsPos[WRISTLEFT_MAIN], 
                                                              transformed_depth_image,
                                                              lastZVal);
                    newSkeleton.jointsPos[i] = p3d;
                }
                    break;
                case SHOULDERRIGHT_MAIN:
                {
                    mediapipe::NormalizedLandmark landmark = output_list.landmark(SHOULDERRIGHT_MEDIAPIPE);
                    k4a_float3_t p3d =  LandmarkTransform(landmark.x(), landmark.y(),
                                                          transformed_depth_image,
                                                          lastZVal);
                    newSkeleton.jointsPos[i] = p3d;
                }
                    break;
                case ELBOWRIGHT_MAIN:
                {
                    mediapipe::NormalizedLandmark landmark = output_list.landmark(ELBOWRIGHT_MEDIAPIPE);
                    k4a_float3_t p3d =  LandmarkTransform(landmark.x(), landmark.y(),
                                                          transformed_depth_image,
                                                          lastZVal);
                    newSkeleton.jointsPos[i] = p3d;
                }
                    break;
                case WRISTRIGHT_MAIN:
                {
                    mediapipe::NormalizedLandmark landmark = output_list.landmark(WRISTRIGHT_MEDIAPIPE);
                    k4a_float3_t p3d =  LandmarkTransform(landmark.x(), landmark.y(),
                                                          transformed_depth_image,
                                                          lastZVal);
                    newSkeleton.jointsPos[i] = p3d;
                }
                    break;
                case HANDRIGHT_MAIN:
                {
                    mediapipe::NormalizedLandmark handRightLandmark = output_list.landmark(HANDRIGHT_MEDIAPIPE);
                    k4a_float3_t p3d =  HandLandmarkTransform(handRightLandmark,
                                                              newSkeleton.jointsPos[WRISTRIGHT_MAIN], 
                                                              transformed_depth_image,
                                                              lastZVal);
                    newSkeleton.jointsPos[i] = p3d;
                }
                    break;
                case HIPLEFT_MAIN:
                {
                    mediapipe::NormalizedLandmark landmark = output_list.landmark(HIPLEFT_MEDIAPIPE);
                    k4a_float3_t p3d =  LandmarkTransform(landmark.x(), landmark.y(),
                                                          transformed_depth_image,
                                                          lastZVal);
                    newSkeleton.jointsPos[i] = p3d;
                }
                    break;
                case KNEELEFT_MAIN:
                {
                    mediapipe::NormalizedLandmark landmark = output_list.landmark(KNEELEFT_MEDIAPIPE);
                    k4a_float3_t p3d =  LandmarkTransform(landmark.x(), landmark.y(),
                                                          transformed_depth_image,
                                                          lastZVal);
                    newSkeleton.jointsPos[i] = p3d;
                }
                    break;
                case ANKLELEFT_MAIN:
                {
                    mediapipe::NormalizedLandmark landmark = output_list.landmark(ANKLELEFT_MEDIAPIPE);
                    k4a_float3_t p3d =  LandmarkTransform(landmark.x(), landmark.y(),
                                                          transformed_depth_image,
                                                          lastZVal);
                    newSkeleton.jointsPos[i] = p3d;
                }
                    break;
                case FOOTLEFT_MAIN:
                {
                    mediapipe::NormalizedLandmark landmark = output_list.landmark(FOOTLEFT_MEDIAPIPE);
                    k4a_float3_t p3d =  LandmarkTransform(landmark.x(), landmark.y(),
                                                          transformed_depth_image,
                                                          lastZVal);
                    newSkeleton.jointsPos[i] = p3d;
                }
                    break;
                case HIPRIGHT_MAIN:
                {
                    mediapipe::NormalizedLandmark landmark = output_list.landmark(HIPRIGHT_MEDIAPIPE);
                    k4a_float3_t p3d =  LandmarkTransform(landmark.x(), landmark.y(),
                                                          transformed_depth_image,
                                                          lastZVal);
                    newSkeleton.jointsPos[i] = p3d;
                }
                    break;
                case KNEERIGHT_MAIN:
                {
                    mediapipe::NormalizedLandmark landmark = output_list.landmark(KNEERIGHT_MEDIAPIPE);
                    k4a_float3_t p3d =  LandmarkTransform(landmark.x(), landmark.y(),
                                                          transformed_depth_image,
                                                          lastZVal);
                    newSkeleton.jointsPos[i] = p3d;
                }
                    break;
                case ANKLERIGHT_MAIN:
                {
                    mediapipe::NormalizedLandmark landmark = output_list.landmark(ANKLERIGHT_MEDIAPIPE);
                    k4a_float3_t p3d =  LandmarkTransform(landmark.x(), landmark.y(),
                                                          transformed_depth_image,
                                                          lastZVal);
                    newSkeleton.jointsPos[i] = p3d;
                }
                    break;
                case FOOTRIGHT_MAIN:
                {
                    mediapipe::NormalizedLandmark landmark = output_list.landmark(FOOTRIGHT_MEDIAPIPE);
                    k4a_float3_t p3d =  LandmarkTransform(landmark.x(), landmark.y(),
                                                          transformed_depth_image,
                                                          lastZVal);
                    newSkeleton.jointsPos[i] = p3d;
                }
                    break;
                case SPINESHOULDER_MAIN:
                {
                    mediapipe::NormalizedLandmark shoulderLeft = output_list.landmark(SHOULDERLEFT_MEDIAPIPE);
                    mediapipe::NormalizedLandmark shoulderRight = output_list.landmark(SHOULDERRIGHT_MEDIAPIPE);
                    float spineshoulder_x = (shoulderRight.x() + shoulderLeft.x()) / 2.0f;
                    float spineshoulder_y = (shoulderRight.y() + shoulderLeft.y()) / 2.0f;
                    k4a_float3_t p3d =  LandmarkTransform(spineshoulder_x, spineshoulder_y,
                                                          transformed_depth_image,
                                                          lastZVal);
                    newSkeleton.jointsPos[i] = p3d;
                }
                    break;
                case HANDTIPLEFT_MAIN:
                {
                    mediapipe::NormalizedLandmark handtipLeft = output_list.landmark(HANDTIPLEFT_MEDIAPIPE);
                    k4a_float3_t p3d =  HandLandmarkTransform(handtipLeft,
                                                              newSkeleton.jointsPos[WRISTLEFT_MAIN], 
                                                              transformed_depth_image,
                                                              lastZVal);
                    newSkeleton.jointsPos[i] = p3d;
                }
                    break;
                case THUMBLEFT_MAIN:
                {
                    mediapipe::NormalizedLandmark thumbLeft = output_list.landmark(THUMBLEFT_MEDIAPIPE);
                    k4a_float3_t p3d =  HandLandmarkTransform(thumbLeft,
                                                              newSkeleton.jointsPos[WRISTLEFT_MAIN], 
                                                              transformed_depth_image,
                                                              lastZVal);
                    newSkeleton.jointsPos[i] = p3d;
                }
                    break;
                case HANDTIPRIGHT_MAIN:
                {
                    mediapipe::NormalizedLandmark handtipRight = output_list.landmark(HANDTIPRIGHT_MEDIAPIPE);
                    k4a_float3_t p3d =  HandLandmarkTransform(handtipRight,
                                                              newSkeleton.jointsPos[WRISTRIGHT_MAIN], 
                                                              transformed_depth_image,
                                                              lastZVal);
                    newSkeleton.jointsPos[i] = p3d;
                }
                    break;
                case THUMBRIGHT_MAIN:
                {
                    mediapipe::NormalizedLandmark thumbRight = output_list.landmark(THUMBRIGHT_MEDIAPIPE);
                    k4a_float3_t p3d =  HandLandmarkTransform(thumbRight,
                                                              newSkeleton.jointsPos[WRISTRIGHT_MAIN], 
                                                              transformed_depth_image,
                                                              lastZVal);
                    newSkeleton.jointsPos[i] = p3d;
                }
                    break;
                }
            }

            newSkeleton.valid = true;
            newSkeleton.timeStamp = getCurrentTimeMillis();
            sendRes = SendJoints(newSkeleton);
            lastSkeleton = newSkeleton;
        }

        if (sendRes < 0)
        {
            logFile << "Error Occurred when Sending Body Joints. Error Code: "
                << sendRes << "\n";
            logFile.flush();
            if (sendRes == -2)
            {
                socketSuccess = false;
            }
        }

        // Release Resources
        if (kinectColorFrame != NULL)
        {
            k4a_image_release(kinectColorFrame);
        }
        if (kinectDepthFrame != NULL)
        {
            k4a_image_release(kinectDepthFrame);
        }
        if (transformed_depth_image != NULL)
        {
            k4a_image_release(transformed_depth_image);
        }
        if (sensor_capture != NULL)
        {
            k4a_capture_release(sensor_capture);
        }
    }

    if (transformation != NULL) {
        k4a_transformation_destroy(transformation);
    }
    k4a_device_stop_cameras(kinect);
    k4a_device_close(kinect);

    mediaRet = graph.CloseInputStream(kInputStream);
    if (!mediaRet.ok())
    {
        logFile << "Fail to Close Input Stream! Error code: "
            << mediaRet.code() << "; Error Message: " << mediaRet.message() <<"\n";
        logFile.flush();
        getCaptureFail = true;
    }
    mediaRet = graph.WaitUntilDone();
    if (!mediaRet.ok())
    {
        logFile << "Fail to Wait Graph Done! Error code: "
            << mediaRet.code() << "; Error Message: " << mediaRet.message() <<"\n";
        logFile.flush();
        getCaptureFail = true;
    }

    logFile << "Output Stop in getBodyJoints \n";
    logFile.flush();

    // Error when get data
    if (getCaptureFail) {
        logFile << "Error Occurs in getting Capture \n";
        logFile.flush();
        return -3;
    }

    // Error when send to socket
    if (!socketSuccess)
    {
        logFile << "Error getting and sending Message to Socket \n";
        logFile.flush();
        return -4;
    }

    logFile << "Return from getBodyJoints by Manually stop \n";
    logFile.flush();
    return 0;
}


JNIEXPORT jint JNICALL Java_com_gdcccn_vrmeds_kinect_KinectCommunicator_getBodyJointsOrientations
(JNIEnv *, jobject)
{
    initLogFile();
    logFile << "-----------------------------------------"
        << getCurrentTimeString()
        << "-----------------------------------------\n";
    logFile << "Start getBodyJointsOrientations...\n";
    logFile.flush();

    logFile << "Now Get Body Joints Orientations is not implemented...\n";
    logFile.flush();
    return -1;
}


JNIEXPORT jint JNICALL Java_com_gdcccn_vrmeds_kinect_KinectCommunicator_getLatestBodyJoints
(JNIEnv *, jobject)
{
    initLogFile();
    logFile << "-----------------------------------------"
        << getCurrentTimeString()
        << "-----------------------------------------\n";
    logFile << "Start getLatestBodyJoints...\n";
    logFile.flush();

    logFile << "Now cannot fetched Last Body Joints from Azure Kinect DK ...\n";
    logFile.flush();
    return -1;
}


JNIEXPORT jint JNICALL Java_com_gdcccn_vrmeds_kinect_KinectCommunicator_stopOutput
(JNIEnv *, jobject)
{
    initLogFile();
    logFile << "-----------------------------------------"
            << getCurrentTimeString()
            << "-----------------------------------------\n";
    logFile << "Enter stopOutput...\n";
    logFile.flush();

    keepOutput = false;

    logFile << "Return from stopOutput...\n";
    logFile.flush();
    return 0;
}


JNIEXPORT jint JNICALL Java_com_gdcccn_vrmeds_kinect_KinectCommunicator_closeKinectSocket
(JNIEnv *, jobject)
{
    initLogFile();
    logFile << "-----------------------------------------"
            << getCurrentTimeString()
            << "-----------------------------------------\n";
    logFile << "Enter closeKinectSocket...\n";
    logFile.flush();

    if (!clientSocket)
    {
        logFile << "No client socket Built\n";
        logFile.flush();
        return -1;
    }

    if (closesocket(clientSocket) == SOCKET_ERROR)
    {
        logFile << "Failed to close client socket \n";
        logFile.flush();
        return -2;
    }
    if (WSACleanup() == SOCKET_ERROR)
    {
        logFile << "Failed to cleanup WSA \n";
        logFile.flush();
        return -2;
    }
    clientSocket = NULL;

    logFile << "Return from closeKinectSocket...\n";
    logFile.flush();
    return 0;
}


JNIEXPORT jint JNICALL Java_com_gdcccn_vrmeds_kinect_KinectCommunicator_shutdownKinect
(JNIEnv *, jobject)
{
    initLogFile();
    logFile << "-----------------------------------------"
        << getCurrentTimeString()
        << "-----------------------------------------\n";
    logFile << "Enter shutdownKinect...\n";
    logFile.flush();

    // if (transformation != NULL) {
    //     k4a_transformation_destroy(transformation);
    // }

    // if(kinect != NULL){
    //     k4a_device_close(kinect);
    // }

    logFile << "Return from shutdownKinect...\n";
    logFile.flush();
    return 0;
}


JNIEXPORT jint JNICALL Java_com_gdcccn_vrmeds_kinect_KinectCommunicator_getMergedBodyData
(JNIEnv *, jobject)
{
    initLogFile();
    logFile << "-----------------------------------------"
        << getCurrentTimeString()
        << "-----------------------------------------\n";
    logFile << "Start getMergedBodyData...\n";
    logFile.flush();

    logFile << "Now Get Merged Body Data is not implemented...\n";
    logFile.flush();
    return -1;
}


int SendJoints(skeleton_main skeleton){
    char sendData[1024];
    int dataLen = 0;
    char *pOffset = sendData;
    pOffset += 9;
    dataLen += 9;
    int sendRes = 0;

    UINT64 trackingID = 1;
    memcpy_s(pOffset, sizeof(sendData), &trackingID, sizeof(trackingID));
    pOffset += sizeof(trackingID);
    for (size_t i = 0; i < JOINT_COUNT_MAIN; i++)
    {
        byte state = 2;
        memcpy_s(pOffset, sizeof(sendData), &state, sizeof(state));
        pOffset += sizeof(state);

        float x = -1 * skeleton.jointsPos[i].xyz.x * 0.001;
        memcpy_s(pOffset, sizeof(sendData), &x, sizeof(x));
        pOffset += sizeof(x);
        float y = -1 * skeleton.jointsPos[i].xyz.y * 0.001;
        memcpy_s(pOffset, sizeof(sendData), &y, sizeof(y));
        pOffset += sizeof(y);
        float z = skeleton.jointsPos[i].xyz.z * 0.001;
        memcpy_s(pOffset, sizeof(sendData), &z, sizeof(z));
        pOffset += sizeof(z);
    }
    dataLen += 333;

    byte count = 1;
    memcpy_s(sendData, sizeof(sendData), &count, sizeof(count));
    memcpy_s((sendData + sizeof(count)), sizeof(sendData), &skeleton.timeStamp, sizeof(skeleton.timeStamp));
    sendRes = send(clientSocket, sendData, dataLen, 0);

    int ret = 0;
    if (sendRes <= 0)
    {
        logFile << "Error Occured when sending Message, return code: " << sendRes << "\n";
        logFile.flush();
        ret = -2;
    }

    return ret;
}


std::string getCurrentTimeString()
{
    time_t now = time(NULL);
    char timeBuf[32];
    ctime_s(timeBuf, sizeof(timeBuf), &now);
    std::string timeStr(timeBuf);
    timeStr = timeStr.substr(0, timeStr.length() - 1);
    return timeStr;
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


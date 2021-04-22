// VRMeds.cpp : 定义 DLL 应用程序的导出函数。
#include "stdafx.h"
#include "com_gdcccn_vrmeds_kinect_KinectCommunicator.h"
#include "VRMeds.h"
#pragma comment(lib,"ws2_32.lib")

/** Wait Timeout in ms.
 */
#define K4A_WAIT_TIMEOUT (50)

SOCKET clientSocket = NULL;
bool keepOutput = true;
std::ofstream logFile;
k4a_device_t kinect = NULL;
k4a_calibration_t sensor_calibration;
k4abt_tracker_t tracker = NULL;
// 与Kinect V2对应的关节点顺序
k4abt_joint_id_t joints_v2[] = { K4ABT_JOINT_PELVIS, K4ABT_JOINT_SPINE_NAVEL, K4ABT_JOINT_NECK, K4ABT_JOINT_HEAD, K4ABT_JOINT_SHOULDER_LEFT, K4ABT_JOINT_ELBOW_LEFT, K4ABT_JOINT_WRIST_LEFT, K4ABT_JOINT_HAND_LEFT, K4ABT_JOINT_SHOULDER_RIGHT, K4ABT_JOINT_ELBOW_RIGHT, K4ABT_JOINT_WRIST_RIGHT, K4ABT_JOINT_HAND_RIGHT, K4ABT_JOINT_HIP_LEFT, K4ABT_JOINT_KNEE_LEFT, K4ABT_JOINT_ANKLE_LEFT, K4ABT_JOINT_FOOT_LEFT, K4ABT_JOINT_HIP_RIGHT, K4ABT_JOINT_KNEE_RIGHT, K4ABT_JOINT_ANKLE_RIGHT, K4ABT_JOINT_FOOT_RIGHT, K4ABT_JOINT_SPINE_CHEST, K4ABT_JOINT_HANDTIP_LEFT, K4ABT_JOINT_THUMB_LEFT, K4ABT_JOINT_HANDTIP_RIGHT, K4ABT_JOINT_THUMB_RIGHT};
k4abt_skeleton_t standardPose;

// 初始化日志文件
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

    // 设置K4A.dll输出日志位置和级别
    //std::string k4aLogFilePath = logDir + "\\k4a-" + nowTimeStr + ".log";
    //std::string setString = "K4A_ENABLE_LOG_TO_A_FILE=" + k4aLogFilePath;
    //_putenv(setString.c_str());
    //setString = "K4A_LOG_LEVEL=i";
    ////setString = "K4A_LOG_LEVEL=w";
    //_putenv(setString.c_str());

    ////设置K4ABT.dll日志输出和级别
    //std::string k4abtLogFilePath = logDir + "\\k4abt-" + nowTimeStr + ".log";
    //setString = "K4ABT_ENABLE_LOG_TO_A_FILE=" + k4abtLogFilePath;
    //_putenv(setString.c_str());
    //setString = "K4ABT_LOG_LEVEL=i";
    //_putenv(setString.c_str());

    // StandardPose
    float[] xs = {-0.033264226000000015f, -0.027377388000000002f, -0.014850768000000006f,
        -0.010943738000000003f, -0.1777727399999999f, -0.24145418000000013f, -0.22280504f,
        -0.22522223999999993f, 0.13287163999999993f, 0.2037601600000001f,
        0.19045648000000007f, 0.19711449999999997f, -0.12015515999999998f,
        -0.2594576799999999f, -0.20215469999999996f, -0.26111587999999997f,
        0.045089083999999995f, 0.2531599400000001f, 0.18140751999999996f,
        0.2746499400000001f, -0.02245063199999999f, -0.22458547999999984f,
        -0.18282978000000008f, 0.2047197200000001f, 0.15324646000000003f
    };
    float[] ys = {-0.29390153999999996f, -0.14015956000000002f, 0.20023106000000004f,
        0.2776688800000002f, 0.13743129999999998f, -0.10067909999999994f,
        -0.24299178000000016f, -0.27726343999999997f, 0.12136679999999998f,
        -0.11540182000000004f, -0.24606594f, -0.29527092000000027f,
        -0.29178660000000006f, -0.35412876f, -0.7142366f, -0.8015051800000004f,
        -0.2958087999999999f, -0.3722868599999999f, -0.7321603800000003f,
        -0.7979744200000003f, 0.12939891999999997f, -0.32180437999999995f,
        -0.30994446000000003f, -0.3631784400000002f, -0.32942518000000015f
    };
    float[] zs = {1.9475686000000005f, 2.020230999999999f, 2.0572362f,
        2.0496249999999994f, 2.0239324f, 1.9306322000000007f, 1.7623252000000016f,
        1.677171999999998f, 2.046793f, 1.941887f, 1.7604663999999994f,
        1.6750730000000005f, 1.9501167999999998f, 1.6001679999999998f,
        1.6129747999999997f, 1.4686811999999998f, 1.9452689999999992f,
        1.6350195999999988f, 1.5996642000000008f, 1.4814554000000002f,
        2.0353619999999992f, 1.5842685999999984f, 1.6607831999999996f, 1.6005544f,
        1.667832800000002f
    };
    for (size_t i = 0; i < K4ABT_JOINT_COUNT; i++)
    {
        standardPose.joints[i].confidence_level = K4ABT_JOINT_CONFIDENCE_MEDIUM;
        standardPose.joints[i].orientation.wxyz.w = skeleton_buffer[0].joints[i].orientation.wxyz.w;
        standardPose.joints[i].orientation.wxyz.x = skeleton_buffer[0].joints[i].orientation.wxyz.x;
        standardPose.joints[i].orientation.wxyz.y = skeleton_buffer[0].joints[i].orientation.wxyz.y;
        standardPose.joints[i].orientation.wxyz.z = skeleton_buffer[0].joints[i].orientation.wxyz.z;
        standardPose.joints[i].position.xyz.x = skeleton_buffer[0].joints[i].position.xyz.x;
        standardPose.joints[i].position.xyz.y = skeleton_buffer[0].joints[i].position.xyz.y;
        standardPose.joints[i].position.xyz.z = skeleton_buffer[0].joints[i].position.xyz.z;
    };



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

    // 查询目前连接的设备数量
    // uint32_t count = k4a_device_get_installed_count();

    // if (count == 0)
    // {
    //     logFile << "Cannot find Any Installed K4A device!\n";
    //     logFile.flush();
    //     return -1;
    // }

    // // 尝试开启第一个设备
    // k4a_result_t ret;
    // ret = k4a_device_open(K4A_DEVICE_DEFAULT, &kinect);
    // if (ret != K4A_RESULT_SUCCEEDED) {
    //     logFile << "Fail to Open K4A device! Return Code: " << ret << "\n";
    //     logFile.flush();
    //     return -1;
    // }

    // // 对设备进行设置
    // k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    // config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
    // //config.depth_mode = K4A_DEPTH_MODE_WFOV_2X2BINNED;
    // config.color_resolution = K4A_COLOR_RESOLUTION_OFF;

    // // Start the camera with the given configuration
    // ret = k4a_device_start_cameras(kinect, &config);
    // if (ret != K4A_RESULT_SUCCEEDED)
    // {
    //     logFile << "Fail to start k4a camera! Return Code: " << ret << "\n";
    //     logFile.flush();
    //     k4a_device_close(kinect);
    //     return -1;
    // }

    // // 根据配置获取传感器校准
    // ret = k4a_device_get_calibration(kinect, config.depth_mode, config.color_resolution, &sensor_calibration);
    // if (ret != K4A_RESULT_SUCCEEDED)
    // {
    //     logFile << "Fail to Get Sensor Calibration! Return Code: " << ret << "\n";
    //     logFile.flush();
    //     k4a_device_stop_cameras(kinect);
    //     k4a_device_close(kinect);
    //     return -1;
    // }

    // // 创建人体追踪器
    // k4abt_tracker_configuration_t tracker_config = K4ABT_TRACKER_CONFIG_DEFAULT;
    // // 可以仅使用CPU
    // //tracker_config.processing_mode = K4ABT_TRACKER_PROCESSING_MODE_CPU;
    // tracker_config.processing_mode = K4ABT_TRACKER_PROCESSING_MODE_GPU;
    // ret = k4abt_tracker_create(&sensor_calibration, tracker_config, &tracker);
    // if (ret != K4A_RESULT_SUCCEEDED)
    // {
    //     logFile << "Fail to Create Body Tracker! Return Code: " << ret << "\n";
    //     logFile.flush();
    //     k4a_device_stop_cameras(kinect);
    //     k4a_device_close(kinect);
    //     return -1;
    // }

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
    {  // 主机IP地址输入错误
        logFile << "Invalid Host IP\n";
        logFile.flush();
        return -1;
    }
    logFile << "IP Setting is: " << IPStr << "\n";
    logFile.flush();

    WORD sockVersion = MAKEWORD(2, 2);
    WSADATA data;
    if (WSAStartup(sockVersion, &data) != 0)
    { // WSA启动失败
        logFile << "Failed calling WSAStartup \n";
        logFile.flush();
        return -2;
    }

    clientSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (clientSocket == INVALID_SOCKET || clientSocket == NULL)
    { // 创建Socket客户端失败
        logFile << "Failed creating client socket \n";
        logFile.flush();
        return -2;
    }

    sockaddr_in serAddr;
    serAddr.sin_family = AF_INET;
    serAddr.sin_port = htons(jPort);
    int r = inet_pton(AF_INET, IPStr, &serAddr.sin_addr);
    if (r != 1)
    { // 转换IP地址失败!
        logFile << "Failed converting IP address to socket address \n";
        logFile.flush();
        return -2;
    }

    if (connect(clientSocket, (sockaddr *)&serAddr, sizeof(serAddr)) == SOCKET_ERROR)
    {  //连接失败 
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

    keepOutput = true;
    bool socketSuccess = true;
    bool getCaptureFail = false;
    const int NUM_SAMPLE = 3; // Number of Sampled Body data
    k4abt_skeleton_t skeleton_buffer[NUM_SAMPLE];
    //float beta = 0.8;  // 计算滑动平均数的参数 beta
    //int mvcount = 1;  // 滑动平均数的帧数，主要用于校准
    //// 存储滑动平均数的数组
    //k4a_float3_t lastBodyAvg[K4ABT_JOINT_COUNT];
    //for (size_t i = 0; i < K4ABT_JOINT_COUNT; i++)
    //{
    //    lastBodyAvg[i].xyz = { 0.0, 0.0, 0.0 };
    //}
    int currentIdx = 0;
    uint32_t lastBodyID = NULL;
    int sendRes = 0;
    while (keepOutput && socketSuccess)
    {
        // 获取Kinect捕获的帧
        k4a_capture_t sensor_capture;
        k4a_wait_result_t get_capture_result = k4a_device_get_capture(kinect, &sensor_capture, K4A_WAIT_TIMEOUT);
        if (get_capture_result == K4A_WAIT_RESULT_SUCCEEDED)
        {
            // 将帧数据传入追踪器队列
            k4a_wait_result_t queue_capture_result = k4abt_tracker_enqueue_capture(tracker, sensor_capture, K4A_WAIT_TIMEOUT);
            k4a_capture_release(sensor_capture);
            if (queue_capture_result == K4A_WAIT_RESULT_TIMEOUT)
            {
                logFile << "Fail to enque capture into tracker. Wait result timeout! \n";
                logFile.flush();
                continue;
            }
            else if (queue_capture_result == K4A_WAIT_RESULT_FAILED)
            {
                logFile << "Fail to enque capture into tracker. Wait result failed! Quit... \n";
                logFile.flush();
                getCaptureFail = true;
                break;
            }
            // 从队列中弹出分析结果
            k4abt_frame_t body_frame = NULL;
            k4a_wait_result_t pop_frame_result = k4abt_tracker_pop_result(tracker, &body_frame, K4A_WAIT_TIMEOUT);
            if (pop_frame_result == K4A_WAIT_RESULT_TIMEOUT)
            {
                logFile << "Fail to pop frame from tracker. Wait result timeout! \n";
                logFile.flush();
                continue;
            }
            else if (pop_frame_result == K4A_WAIT_RESULT_FAILED)
            {
                logFile << "Fail to pop frame from tracker. Wait result failed! Quit... \n";
                logFile.flush();
                getCaptureFail = true;
                break;
            }

            // 获取检测到人体数量
            size_t num_bodies = k4abt_frame_get_num_bodies(body_frame);

            // 遍历所有人体的信息
            float nearestDist = 99999.99;
            k4abt_skeleton_t selectedBodySkeleton;
            uint32_t selectedID;
            bool findBody = false;
            for (size_t i = 0; i < num_bodies; i++)
            {
                uint32_t id = k4abt_frame_get_body_id(body_frame, i);
                if (id == K4ABT_INVALID_BODY_ID) {
                    logFile << "Body Frame Return Invalid Body ID: " << id << "\n";
                    logFile.flush();
                    continue;
                }

                k4abt_skeleton_t skeleton;
                k4a_result_t ret = k4abt_frame_get_body_skeleton(body_frame, i, &skeleton);
                if (ret != K4A_RESULT_SUCCEEDED)
                {
                    logFile << "Fail to get body Skeleton. Return Code: " << ret << "\n";
                    logFile.flush();
                    continue;
                }

                // 找到与之前ID一致的身体，则优先使用该身体
                if (lastBodyID != NULL && id == lastBodyID) {
                    selectedBodySkeleton = skeleton;
                    selectedID = id;
                    findBody = true;
                    break;
                }
                else {
                    // 没有找到与之前ID一致的身体时，优先寻找距离最近的身体
                    //if (skeleton.joints[K4ABT_JOINT_SPINE_CHEST].position.xyz.z < nearestDist) {
                    //    nearestDist = skeleton.joints[K4ABT_JOINT_SPINE_CHEST].position.xyz.z;
                    //    selectedBodySkeleton = skeleton;
                    //    selectedID = id;
                    //    findBody = true;
                    //}
                    // 没有找到与之前ID一致的身体时，优先寻找最正对Kinect的身体
                    if (abs(skeleton.joints[K4ABT_JOINT_SPINE_CHEST].position.xyz.x) < nearestDist) {
                        nearestDist = abs(skeleton.joints[K4ABT_JOINT_SPINE_CHEST].position.xyz.x);
                        selectedBodySkeleton = skeleton;
                        selectedID = id;
                        findBody = true;
                        currentIdx = 0;
                        // 重置滑动平均数的参数
                        //mvcount = 1;
                        //for (size_t i = 0; i < K4ABT_JOINT_COUNT; i++)
                        //{
                        //    lastBodyAvg[i].xyz = { 0.0, 0.0, 0.0 };
                        //}
                    }
                }
            }

            if (findBody)
            {
                skeleton_buffer[currentIdx] = selectedBodySkeleton;
                lastBodyID = selectedID;
                currentIdx++;
                if (currentIdx == NUM_SAMPLE)
                {
                    sendRes = CalculateAndSendSkeletonAvg(skeleton_buffer, NUM_SAMPLE, selectedID);
                    //sendRes = CalculateAndSendSkeletonMA(skeleton_buffer, NUM_SAMPLE, beta, mvcount, lastBodyAvg, selectedID);
                    currentIdx = 0;
                    //mvcount += NUM_SAMPLE;
                }
            }
            else
            {
                char temp = { 0 };
                sendRes = send(clientSocket, &temp, sizeof(temp), 0);
                currentIdx = 0;
                lastBodyID = NULL;
            }

            if (sendRes < 0)
            {  // 处理、发送数据过程中出错
                logFile << "Error Occurred when Sending Body Joints. Error Code: "
                    << sendRes << "\n";
                logFile.flush();
                if (sendRes == -2)
                {  // 目前仅在SocketSend出错时退出循环
                    socketSuccess = false;
                }
            }

            // Remember to release the body frame once you finish using it
            k4abt_frame_release(body_frame);
        }
        else if (get_capture_result == K4A_WAIT_RESULT_FAILED)
        {
            logFile << "Fail to get capture from Kinect. Return Code: " << get_capture_result << "\n";
            logFile.flush();
            getCaptureFail = true;
            break;
        }

    }

    logFile << "Output Stop in getBodyJoints \n";
    logFile.flush();

    // 数据帧获取过程中出错
    if (getCaptureFail) {
        logFile << "Error Occurs in getting Capture \n";
        logFile.flush();
        return -3;
    }

    // Socket数据获取或发送过程中出错
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

    keepOutput = true;
    bool socketSuccess = true;
    while (keepOutput && socketSuccess)
    {
        // 获取Kinect捕获的帧
        k4a_capture_t sensor_capture;
        k4a_wait_result_t get_capture_result = k4a_device_get_capture(kinect, &sensor_capture, K4A_WAIT_TIMEOUT);
        if (get_capture_result == K4A_WAIT_RESULT_SUCCEEDED)
        {
            // 将帧数据传入追踪器队列
            k4a_wait_result_t queue_capture_result = k4abt_tracker_enqueue_capture(tracker, sensor_capture, K4A_WAIT_TIMEOUT);
            k4a_capture_release(sensor_capture);
            if (queue_capture_result != K4A_WAIT_RESULT_SUCCEEDED)
            {
                logFile << "Fail to enque capture into tracker. Return Code: " << queue_capture_result << "\n";
                logFile.flush();
                continue;
            }
            // 从队列中弹出分析结果
            k4abt_frame_t body_frame = NULL;
            k4a_wait_result_t pop_frame_result = k4abt_tracker_pop_result(tracker, &body_frame, K4A_WAIT_TIMEOUT);
            if (pop_frame_result != K4A_WAIT_RESULT_SUCCEEDED)
            {
                logFile << "Fail to pop frame from tracker. Return Code: " << pop_frame_result << "\n";
                logFile.flush();
                continue;
            }

            // 序列化并发送关节点朝向信息
            int sendRes = SendBodyJointOrientations(body_frame);
            if (sendRes < 0)
            {  // 处理、发送数据过程中出错
                logFile << "Error Occurred when Sending Body Joints Orientations. Error Code: "
                    << sendRes << "\n";
                logFile.flush();
                if (sendRes == -2)
                {  // 目前仅在SocketSend出错时退出循环
                    socketSuccess = false;
                }
            }
        }
        else
        {
            logFile << "Fail to get capture from Kinect. Return Code: " << get_capture_result << "\n";
            logFile.flush();
            continue;
        }
    }
    logFile << "Output Stop in getBodyJointsOrientations \n";
    logFile.flush();

    if (!socketSuccess)
    { // Socket数据获取或发送过程中出错
        logFile << "Error getting and sending Message to Socket \n";
        logFile.flush();
        return -4;
    }

    logFile << "Return from getBodyJointsOrientations by Manually stop \n";
    logFile.flush();
    return 0;
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
    {  // clientSocket 不存在
        logFile << "No client socket Built\n";
        logFile.flush();
        return -1;
    }

    //释放资源 closesocket函数如果执行成功就返回0，否则返回SOCKET_ERROR。 
    if (closesocket(clientSocket) == SOCKET_ERROR)
    {  // 释放Client Socket失败
        logFile << "Failed to close client socket \n";
        logFile.flush();
        return -2;
    }
    // 操作成功返回值为0；否则返回值为SOCKET_ERROR，可以通过调用WSAGetLastError获取错误代码
    if (WSACleanup() == SOCKET_ERROR)
    { // 关闭WSA错误
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

    // 释放人体追踪器占用资源
    // k4abt_tracker_shutdown(tracker);
    // k4abt_tracker_destroy(tracker);

    // // 关闭设备
    // k4a_device_stop_cameras(kinect);
    // k4a_device_close(kinect);

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

    keepOutput = true;
    bool socketSuccess = true;
    while (keepOutput && socketSuccess)
    {
        // 获取Kinect捕获的帧
        k4a_capture_t sensor_capture;
        k4a_wait_result_t get_capture_result = k4a_device_get_capture(kinect, &sensor_capture, K4A_WAIT_TIMEOUT);
        if (get_capture_result == K4A_WAIT_RESULT_SUCCEEDED)
        {
            // 将帧数据传入追踪器队列
            k4a_wait_result_t queue_capture_result = k4abt_tracker_enqueue_capture(tracker, sensor_capture, K4A_WAIT_TIMEOUT);
            k4a_capture_release(sensor_capture);
            if (queue_capture_result != K4A_WAIT_RESULT_SUCCEEDED)
            {
                logFile << "Fail to enque capture into tracker. Return Code: " << queue_capture_result << "\n";
                logFile.flush();
                continue;
            }
            // 从队列中弹出分析结果
            k4abt_frame_t body_frame = NULL;
            k4a_wait_result_t pop_frame_result = k4abt_tracker_pop_result(tracker, &body_frame, K4A_WAIT_TIMEOUT);
            if (pop_frame_result != K4A_WAIT_RESULT_SUCCEEDED)
            {
                logFile << "Fail to pop frame from tracker. Return Code: " << pop_frame_result << "\n";
                logFile.flush();
                continue;
            }

            // 序列化并发送关节点位置和朝向信息
            int sendRes = SendMergedBodyData(body_frame);
            k4abt_frame_release(body_frame);
            if (sendRes < 0)
            {  // 处理、发送数据过程中出错
                logFile << "Error Occurred when Sending Merged Body Data. Error Code: "
                    << sendRes << "\n";
                logFile.flush();
                if (sendRes == -2)
                {  // 目前仅在SocketSend出错时退出循环
                    socketSuccess = false;
                }
            }
        }
        else
        {
            logFile << "Fail to get capture from Kinect. Return Code: " << get_capture_result << "\n";
            logFile.flush();
            continue;
        }
    }

    logFile << "Output Stop in getMergedBodyData \n";
    logFile.flush();

    if (!socketSuccess)
    { // Socket数据获取或发送过程中出错
        logFile << "Error getting and sending Message to Socket \n";
        logFile.flush();
        //SendDataEnd();
        return -4;
    }

    logFile << "Return from getMergedBodyData by Manually stop \n";
    logFile.flush();
    return 0;
}


int SendBodyJoints(k4abt_frame_t body_frame)
{
    UINT64 timeStamp = getCurrentTimeMillis();
    char sendData[1024];
    int dataLen = 0;
    char *pOffset = sendData;
    pOffset += 9; // 把前9个字节提前让出来给身体数量和timestamp
    dataLen += 9;

    int sendRes = 0;
    // 获取检测到人体数量
    size_t num_bodies = k4abt_frame_get_num_bodies(body_frame);
    //if (num_bodies > 5) {
    //    logFile << num_bodies << " bodies are detected! Too Many Bodies!!!\n";
    //    logFile << "Choose first 5 bodies to send\n";
    //    logFile.flush();
    //    num_bodies = 5;
    //}

    // 遍历所有人体，找到距离最近的
    int trackedBodyCount = 0;
    float nearestDist = 9999999.99;
    k4abt_skeleton_t nearestBodySkeleton;
    uint32_t nearestID;
    for (size_t i = 0; i < num_bodies; i++)
    {
        uint32_t id = k4abt_frame_get_body_id(body_frame, i);
        if (id == K4ABT_INVALID_BODY_ID) {
            logFile << "Body Frame Return Invalid Body ID.\n";
            logFile.flush();
            continue;
        }

        k4abt_skeleton_t skeleton;
        k4a_result_t ret = k4abt_frame_get_body_skeleton(body_frame, i, &skeleton);
        if (ret != K4A_RESULT_SUCCEEDED)
        {
            logFile << "Fail to get body Skeleton. Return Code: " << ret << "\n";
            logFile.flush();
            continue;
        }

        if (skeleton.joints[K4ABT_JOINT_SPINE_CHEST].position.xyz.z < nearestDist) {
            nearestDist = skeleton.joints[K4ABT_JOINT_SPINE_CHEST].position.xyz.z;
            nearestBodySkeleton = skeleton;
            nearestID = id;
            trackedBodyCount = 1;
        }
    }

    if (trackedBodyCount > 0)
    {
        // 身体数据详情：追踪ID（trackingID） UINT64（8字节，LE） | 关节点数组 （Joints[25]，关节点下标位置与其关节点类型对应，每个关节点13字节，共325字节）
        UINT64 trackingID = nearestID;
        memcpy_s(pOffset, sizeof(sendData), &trackingID, sizeof(trackingID));
        pOffset += sizeof(trackingID);
        for (auto j_type : joints_v2)
        {// 开始按顺序序列化关节点
         // 关节点详情：关节点的追踪状态 TrackingState（1字节） | 关节点在空间中x坐标 float（4字节，LE） | 关节点在空间中y坐标 float（4字节，LE） | 关节点在空间中z坐标 float（4字节，LE）
            byte state = 2;
            memcpy_s(pOffset, sizeof(sendData), &state, sizeof(state));
            pOffset += sizeof(state);
            if (j_type == K4ABT_JOINT_SPINE_CHEST) {
                float x = -1 * (nearestBodySkeleton.joints[K4ABT_JOINT_SHOULDER_LEFT].position.xyz.x
                    + nearestBodySkeleton.joints[K4ABT_JOINT_SHOULDER_RIGHT].position.xyz.x) * 0.5 * 0.001;
                memcpy_s(pOffset, sizeof(sendData), &x, sizeof(x));
                pOffset += sizeof(x);
                float y = -1 * (nearestBodySkeleton.joints[K4ABT_JOINT_SHOULDER_LEFT].position.xyz.y
                    + nearestBodySkeleton.joints[K4ABT_JOINT_SHOULDER_RIGHT].position.xyz.y) * 0.5 * 0.001;
                memcpy_s(pOffset, sizeof(sendData), &y, sizeof(y));
                pOffset += sizeof(y);
                float z = (nearestBodySkeleton.joints[K4ABT_JOINT_SHOULDER_LEFT].position.xyz.z
                    + nearestBodySkeleton.joints[K4ABT_JOINT_SHOULDER_RIGHT].position.xyz.z) * 0.5 * 0.001;
                memcpy_s(pOffset, sizeof(sendData), &z, sizeof(z));
                pOffset += sizeof(z);
            }
            else {
                float x = -1 * nearestBodySkeleton.joints[j_type].position.xyz.x * 0.001;
                memcpy_s(pOffset, sizeof(sendData), &x, sizeof(x));
                pOffset += sizeof(x);
                float y = -1 * nearestBodySkeleton.joints[j_type].position.xyz.y * 0.001;
                memcpy_s(pOffset, sizeof(sendData), &y, sizeof(y));
                pOffset += sizeof(y);
                float z = nearestBodySkeleton.joints[j_type].position.xyz.z * 0.001;
                memcpy_s(pOffset, sizeof(sendData), &z, sizeof(z));
                pOffset += sizeof(z);
            }
        }
    }

    // 遍历所有人体的信息
    //int trackedBodyCount = 0;
    //for (size_t i = 0; i < num_bodies; i++)
    //{
    //    uint32_t id = k4abt_frame_get_body_id(body_frame, i);
    //    if (id == K4ABT_INVALID_BODY_ID) {
    //        logFile << "Body Frame Return Invalid Body ID.\n";
    //        logFile.flush();
    //        continue;
    //    }

    //    k4abt_skeleton_t skeleton;
    //    k4a_result_t ret = k4abt_frame_get_body_skeleton(body_frame, i, &skeleton);
    //    if (ret != K4A_RESULT_SUCCEEDED)
    //    {
    //        logFile << "Fail to get body Skeleton. Return Code: " << ret << "\n";
    //        logFile.flush();
    //        continue;
    //    }

    //    // 身体数据详情：追踪ID（trackingID） UINT64（8字节，LE） | 关节点数组 （Joints[25]，关节点下标位置与其关节点类型对应，每个关节点13字节，共325字节）
    //    UINT64 trackingID = id;
    //    memcpy_s(pOffset, sizeof(sendData), &trackingID, sizeof(trackingID));
    //    pOffset += sizeof(trackingID);
    //    for (auto j_type : joints_v2)
    //    {// 开始按顺序序列化关节点
    //     // 关节点详情：关节点的追踪状态 TrackingState（1字节） | 关节点在空间中x坐标 float（4字节，LE） | 关节点在空间中y坐标 float（4字节，LE） | 关节点在空间中z坐标 float（4字节，LE）
    //        byte state = 2;
    //        memcpy_s(pOffset, sizeof(sendData), &state, sizeof(state));
    //        pOffset += sizeof(state);
    //        float x = -1 * skeleton.joints[j_type].position.xyz.x * 0.001;
    //        memcpy_s(pOffset, sizeof(sendData), &x, sizeof(x));
    //        pOffset += sizeof(x);
    //        float y = -1 * skeleton.joints[j_type].position.xyz.y * 0.001;
    //        memcpy_s(pOffset, sizeof(sendData), &y, sizeof(y));
    //        pOffset += sizeof(y);
    //        float z = skeleton.joints[j_type].position.xyz.z * 0.001;
    //        memcpy_s(pOffset, sizeof(sendData), &z, sizeof(z));
    //        pOffset += sizeof(z);
    //    }

    //    trackedBodyCount++;
    //}
    dataLen += 333 * trackedBodyCount;

    // 未追踪到身体数据时仅发送0
    if (trackedBodyCount == 0)
    {
        char temp = { 0 };
        sendRes = send(clientSocket, &temp, sizeof(temp), 0);
    }
    else
    {  // 检测到身体数据时，前面拼上身体数量和timestamp
        byte count = trackedBodyCount;
        memcpy_s(sendData, sizeof(sendData), &count, sizeof(count));
        memcpy_s((sendData + sizeof(count)), sizeof(sendData), &timeStamp, sizeof(timeStamp));
        sendRes = send(clientSocket, sendData, dataLen, 0);
    }

    int ret = 0;
    if (sendRes <= 0)
    {
        logFile << "Error Occured when sending Message, return code: " << sendRes << "\n";
        logFile.flush();
        ret = -2;
    }

    k4abt_frame_release(body_frame);
    return ret;
}


int SendBodyJointOrientations(k4abt_frame_t body_frame)
{
    UINT64 timeStamp = getCurrentTimeMillis();
    char sendData[2048];
    int dataLen = 0;
    char *pOffset = sendData;
    pOffset += 9; // 把前9个字节提前让出来给身体数量和timestamp
    dataLen += 9;

    int sendRes = 0;
    // 获取检测到人体数量
    size_t num_bodies = k4abt_frame_get_num_bodies(body_frame);
    if (num_bodies > 5) {
        logFile << num_bodies << " bodies are detected! Too many Bodies\n";
        logFile << "Choose first 5 bodies to send!\n";
        logFile.flush();
        num_bodies = 5;
    }

    // 遍历所有人体的信息
    int trackedBodyCount = 0;
    for (size_t i = 0; i < num_bodies; i++)
    {
        uint32_t id = k4abt_frame_get_body_id(body_frame, i);
        if (id == K4ABT_INVALID_BODY_ID) {
            logFile << "Body Frame Return Invalid Body ID.\n";
            logFile.flush();
            continue;
        }

        k4abt_skeleton_t skeleton;
        k4a_result_t ret = k4abt_frame_get_body_skeleton(body_frame, i, &skeleton);
        if (ret != K4A_RESULT_SUCCEEDED)
        {
            logFile << "Fail to get body Skeleton. Return Code: " << ret << "\n";
            logFile.flush();
            continue;
        }

        // 身体数据详情：追踪ID（trackingID） UINT64（8字节，LE） | 关节点朝向数组 （JointOrientation[25]，关节点下标位置与其关节点类型对应，每个关节点朝向16字节，共400字节）
        UINT64 trackingID = id;
        memcpy_s(pOffset, sizeof(sendData), &trackingID, sizeof(trackingID));
        pOffset += sizeof(trackingID);
        for (auto j_type : joints_v2)
        {// 开始按顺序序列化关节点朝向
        // 关节点朝向详情：关节点朝向四元数中的x分量 float（4字节，LE） | 关节点朝向四元数中的y分量 float（4字节，LE） | 关节点朝向四元数中的z分量 float（4字节，LE） | 关节点朝向四元数中的w分量 float（4字节，LE）
            float x = skeleton.joints[j_type].orientation.wxyz.x;
            memcpy_s(pOffset, sizeof(sendData), &x, sizeof(x));
            pOffset += sizeof(x);
            float y = skeleton.joints[j_type].orientation.wxyz.y;
            memcpy_s(pOffset, sizeof(sendData), &y, sizeof(y));
            pOffset += sizeof(y);
            float z = skeleton.joints[j_type].orientation.wxyz.z;
            memcpy_s(pOffset, sizeof(sendData), &z, sizeof(z));
            pOffset += sizeof(z);
            float w = skeleton.joints[j_type].orientation.wxyz.w;
            memcpy_s(pOffset, sizeof(sendData), &w, sizeof(w));
            pOffset += sizeof(w);
        }

        trackedBodyCount++;
    }
    dataLen += 408 * trackedBodyCount;

    // 未追踪到身体数据时仅发送0
    if (trackedBodyCount == 0)
    {
        char temp = { 0 };
        sendRes = send(clientSocket, &temp, sizeof(temp), 0);
    }
    else
    {  // 检测到身体数据时，前面拼上身体数量和timestamp
        byte count = trackedBodyCount;
        memcpy_s(sendData, sizeof(sendData), &count, sizeof(count));
        memcpy_s((sendData + sizeof(count)), sizeof(sendData), &timeStamp, sizeof(timeStamp));
        sendRes = send(clientSocket, sendData, dataLen, 0);
    }

    int ret = 0;
    if (sendRes <= 0)
    {
        logFile << "Error Occured when sending Message, return code: " << sendRes << "\n";
        logFile.flush();
        ret = -2;
    }

    k4abt_frame_release(body_frame);
    return ret;
}


int SendMergedBodyData(k4abt_frame_t body_frame)
{
    UINT64 timeStamp = getCurrentTimeMillis();
    //char sendData[5120];
    char sendData[1024];
    int dataLen = 0;
    char *pOffset = sendData;
    pOffset += 9; // 把前9个字节提前让出来给身体数量和timestamp
    dataLen += 9;

    int sendRes = 0;
    // 获取检测到人体数量
    size_t num_bodies = k4abt_frame_get_num_bodies(body_frame);
    //if (num_bodies > 5) {
    //    logFile << num_bodies << " bodies are detected! Too many bodies!!!\n";
    //    logFile << "Choose first 5 bodies to send!\n";
    //    logFile.flush();
    //    num_bodies = 5;
    //}

    // 遍历所有人体，找到距离最近的
    int trackedBodyCount = 0;
    float nearestDist = 9999999.99;
    k4abt_skeleton_t nearestBodySkeleton;
    uint32_t nearestID;
    for (size_t i = 0; i < num_bodies; i++)
    {
        uint32_t id = k4abt_frame_get_body_id(body_frame, i);
        if (id == K4ABT_INVALID_BODY_ID) {
            logFile << "Body Frame Return Invalid Body ID.\n";
            logFile.flush();
            continue;
        }

        k4abt_skeleton_t skeleton;
        k4a_result_t ret = k4abt_frame_get_body_skeleton(body_frame, i, &skeleton);
        if (ret != K4A_RESULT_SUCCEEDED)
        {
            logFile << "Fail to get body Skeleton. Return Code: " << ret << "\n";
            logFile.flush();
            continue;
        }

        if (skeleton.joints[K4ABT_JOINT_SPINE_CHEST].position.xyz.z < nearestDist) {
            nearestDist = skeleton.joints[K4ABT_JOINT_SPINE_CHEST].position.xyz.z;
            nearestBodySkeleton = skeleton;
            nearestID = id;
            trackedBodyCount = 1;
        }
    }

    if (trackedBodyCount > 0)
    {
        // 身体数据详情：追踪ID（trackingID） UINT64（8字节，LE） | 关节点数组 （Joints[25]，关节点下标位置与其关节点类型对应，每个关节点13字节，共325字节）| 关节点朝向数组 （JointOrientation[25]，下标位置与其关节点类型对应，每个关节点朝向信息16字节，共400字节）
        UINT64 trackingID = nearestID;
        memcpy_s(pOffset, sizeof(sendData), &trackingID, sizeof(trackingID));
        pOffset += sizeof(trackingID);
        for (auto j_type : joints_v2)
        {// 开始按顺序序列化关节点
         // 关节点详情：关节点的追踪状态 TrackingState（1字节） | 关节点在空间中x坐标 float（4字节，LE） | 关节点在空间中y坐标 float（4字节，LE） | 关节点在空间中z坐标 float（4字节，LE）
            byte state = 2;
            memcpy_s(pOffset, sizeof(sendData), &state, sizeof(state));
            pOffset += sizeof(state);
            if (j_type == K4ABT_JOINT_SPINE_CHEST) {

                float x = -1 * (nearestBodySkeleton.joints[K4ABT_JOINT_SHOULDER_LEFT].position.xyz.x
                    + nearestBodySkeleton.joints[K4ABT_JOINT_SHOULDER_RIGHT].position.xyz.x) * 0.5 * 0.001;
                memcpy_s(pOffset, sizeof(sendData), &x, sizeof(x));
                pOffset += sizeof(x);
                float y = -1 * (nearestBodySkeleton.joints[K4ABT_JOINT_SHOULDER_LEFT].position.xyz.y
                    + nearestBodySkeleton.joints[K4ABT_JOINT_SHOULDER_RIGHT].position.xyz.y) * 0.5 * 0.001;
                memcpy_s(pOffset, sizeof(sendData), &y, sizeof(y));
                pOffset += sizeof(y);
                float z = (nearestBodySkeleton.joints[K4ABT_JOINT_SHOULDER_LEFT].position.xyz.z
                    + nearestBodySkeleton.joints[K4ABT_JOINT_SHOULDER_RIGHT].position.xyz.z) * 0.5 * 0.001;
                memcpy_s(pOffset, sizeof(sendData), &z, sizeof(z));
                pOffset += sizeof(z);
            }
            else {
                float x = -1 * nearestBodySkeleton.joints[j_type].position.xyz.x * 0.001;
                memcpy_s(pOffset, sizeof(sendData), &x, sizeof(x));
                pOffset += sizeof(x);
                float y = -1 * nearestBodySkeleton.joints[j_type].position.xyz.y * 0.001;
                memcpy_s(pOffset, sizeof(sendData), &y, sizeof(y));
                pOffset += sizeof(y);
                float z = nearestBodySkeleton.joints[j_type].position.xyz.z * 0.001;
                memcpy_s(pOffset, sizeof(sendData), &z, sizeof(z));
                pOffset += sizeof(z);
            }
        }

        for (auto j_type : joints_v2)
        {// 开始按顺序序列化关节点朝向
        // 关节点朝向详情：关节点朝向四元数中的x分量 float（4字节，LE） | 关节点朝向四元数中的y分量 float（4字节，LE） | 关节点朝向四元数中的z分量 float（4字节，LE） | 关节点朝向四元数中的w分量 float（4字节，LE）
            float x = nearestBodySkeleton.joints[j_type].orientation.wxyz.x;
            memcpy_s(pOffset, sizeof(sendData), &x, sizeof(x));
            pOffset += sizeof(x);
            float y = nearestBodySkeleton.joints[j_type].orientation.wxyz.y;
            memcpy_s(pOffset, sizeof(sendData), &y, sizeof(y));
            pOffset += sizeof(y);
            float z = nearestBodySkeleton.joints[j_type].orientation.wxyz.z;
            memcpy_s(pOffset, sizeof(sendData), &z, sizeof(z));
            pOffset += sizeof(z);
            float w = nearestBodySkeleton.joints[j_type].orientation.wxyz.w;
            memcpy_s(pOffset, sizeof(sendData), &w, sizeof(w));
            pOffset += sizeof(w);
        }
    }

    // 遍历所有人体的信息并发送
    //for (size_t i = 0; i < num_bodies; i++)
    //{
    //    uint32_t id = k4abt_frame_get_body_id(body_frame, i);
    //    if (id == K4ABT_INVALID_BODY_ID) {
    //        logFile << "Body Frame Return Invalid Body ID.\n";
    //        logFile.flush();
    //        continue;
    //    }

    //    k4abt_skeleton_t skeleton;
    //    k4a_result_t ret = k4abt_frame_get_body_skeleton(body_frame, i, &skeleton);
    //    if (ret != K4A_RESULT_SUCCEEDED)
    //    {
    //        logFile << "Fail to get body Skeleton. Return Code: " << ret << "\n";
    //        logFile.flush();
    //        continue;
    //    }

    //    // 身体数据详情：追踪ID（trackingID） UINT64（8字节，LE） | 关节点数组 （Joints[25]，关节点下标位置与其关节点类型对应，每个关节点13字节，共325字节）| 关节点朝向数组 （JointOrientation[25]，下标位置与其关节点类型对应，每个关节点朝向信息16字节，共400字节）
    //    UINT64 trackingID = id;
    //    memcpy_s(pOffset, sizeof(sendData), &trackingID, sizeof(trackingID));
    //    pOffset += sizeof(trackingID);
    //    for (auto j_type : joints_v2)
    //    {// 开始按顺序序列化关节点
    //     // 关节点详情：关节点的追踪状态 TrackingState（1字节） | 关节点在空间中x坐标 float（4字节，LE） | 关节点在空间中y坐标 float（4字节，LE） | 关节点在空间中z坐标 float（4字节，LE）
    //        byte state = 2;
    //        memcpy_s(pOffset, sizeof(sendData), &state, sizeof(state));
    //        pOffset += sizeof(state);
    //        if (j_type == K4ABT_JOINT_SPINE_CHEST) {
    //            
    //            float x = -1 * (skeleton.joints[K4ABT_JOINT_SHOULDER_LEFT].position.xyz.x
    //                + skeleton.joints[K4ABT_JOINT_SHOULDER_RIGHT].position.xyz.x) * 0.5 * 0.001;
    //            memcpy_s(pOffset, sizeof(sendData), &x, sizeof(x));
    //            pOffset += sizeof(x);
    //            float y = -1 * (skeleton.joints[K4ABT_JOINT_SHOULDER_LEFT].position.xyz.y
    //                + skeleton.joints[K4ABT_JOINT_SHOULDER_RIGHT].position.xyz.y) * 0.5 * 0.001;
    //            memcpy_s(pOffset, sizeof(sendData), &y, sizeof(y));
    //            pOffset += sizeof(y);
    //            float z = (skeleton.joints[K4ABT_JOINT_SHOULDER_LEFT].position.xyz.z
    //                + skeleton.joints[K4ABT_JOINT_SHOULDER_RIGHT].position.xyz.z) * 0.5 * 0.001;
    //            memcpy_s(pOffset, sizeof(sendData), &z, sizeof(z));
    //            pOffset += sizeof(z);
    //        }
    //        else {
    //            float x = -1 * skeleton.joints[j_type].position.xyz.x * 0.001;
    //            memcpy_s(pOffset, sizeof(sendData), &x, sizeof(x));
    //            pOffset += sizeof(x);
    //            float y = -1 * skeleton.joints[j_type].position.xyz.y * 0.001;
    //            memcpy_s(pOffset, sizeof(sendData), &y, sizeof(y));
    //            pOffset += sizeof(y);
    //            float z = skeleton.joints[j_type].position.xyz.z * 0.001;
    //            memcpy_s(pOffset, sizeof(sendData), &z, sizeof(z));
    //            pOffset += sizeof(z);
    //        }
    //    }

    //    for (auto j_type : joints_v2)
    //    {// 开始按顺序序列化关节点朝向
    //    // 关节点朝向详情：关节点朝向四元数中的x分量 float（4字节，LE） | 关节点朝向四元数中的y分量 float（4字节，LE） | 关节点朝向四元数中的z分量 float（4字节，LE） | 关节点朝向四元数中的w分量 float（4字节，LE）
    //        float x = skeleton.joints[j_type].orientation.wxyz.x;
    //        memcpy_s(pOffset, sizeof(sendData), &x, sizeof(x));
    //        pOffset += sizeof(x);
    //        float y = skeleton.joints[j_type].orientation.wxyz.y;
    //        memcpy_s(pOffset, sizeof(sendData), &y, sizeof(y));
    //        pOffset += sizeof(y);
    //        float z = skeleton.joints[j_type].orientation.wxyz.z;
    //        memcpy_s(pOffset, sizeof(sendData), &z, sizeof(z));
    //        pOffset += sizeof(z);
    //        float w = skeleton.joints[j_type].orientation.wxyz.w;
    //        memcpy_s(pOffset, sizeof(sendData), &w, sizeof(w));
    //        pOffset += sizeof(w);
    //    }

    //    trackedBodyCount++;
    //}
    dataLen += 733 * trackedBodyCount;

    // 未追踪到身体数据时仅发送0
    if (trackedBodyCount == 0)
    {
        char temp = { 0 };
        sendRes = send(clientSocket, &temp, sizeof(temp), 0);
    }
    else
    {  // 检测到身体数据时，前面拼上身体数量和timestamp
        byte count = trackedBodyCount;
        memcpy_s(sendData, sizeof(sendData), &count, sizeof(count));
        memcpy_s((sendData + sizeof(count)), sizeof(sendData), &timeStamp, sizeof(timeStamp));
        sendRes = send(clientSocket, sendData, dataLen, 0);
    }
    //logFile << "Send Socket Result: " << sendRes << "\n";

    int ret = 0;
    if (sendRes <= 0)
    {
        logFile << "Error Occured when sending Message, return code: " << sendRes << "\n";
        logFile.flush();
        ret = -2;
    }

    return ret;
}


int CalculateAndSendSkeletonAvg(k4abt_skeleton_t *skeleton_buffer, int sampleNum, uint32_t selectedID) {
    k4abt_skeleton_t skeletonAvg;
    k4abt_skeleton_t skeletonSum;
    if (sampleNum == 1) {
        for (size_t i = 0; i < K4ABT_JOINT_COUNT; i++)
        {
            skeletonAvg.joints[i].confidence_level = K4ABT_JOINT_CONFIDENCE_MEDIUM;
            skeletonAvg.joints[i].orientation.wxyz.w = skeleton_buffer[0].joints[i].orientation.wxyz.w;
            skeletonAvg.joints[i].orientation.wxyz.x = skeleton_buffer[0].joints[i].orientation.wxyz.x;
            skeletonAvg.joints[i].orientation.wxyz.y = skeleton_buffer[0].joints[i].orientation.wxyz.y;
            skeletonAvg.joints[i].orientation.wxyz.z = skeleton_buffer[0].joints[i].orientation.wxyz.z;
            skeletonAvg.joints[i].position.xyz.x = skeleton_buffer[0].joints[i].position.xyz.x;
            skeletonAvg.joints[i].position.xyz.y = skeleton_buffer[0].joints[i].position.xyz.y;
            skeletonAvg.joints[i].position.xyz.z = skeleton_buffer[0].joints[i].position.xyz.z;
        }
    }
    else
    {
        for (size_t i = 0; i < sampleNum; i++)
        {
            for (size_t j = 0; j < K4ABT_JOINT_COUNT; j++)
            {
                if (i == 0)
                {
                    skeletonSum.joints[j].orientation.wxyz.w = skeleton_buffer[i].joints[j].orientation.wxyz.w;
                    skeletonSum.joints[j].orientation.wxyz.x = skeleton_buffer[i].joints[j].orientation.wxyz.x;
                    skeletonSum.joints[j].orientation.wxyz.y = skeleton_buffer[i].joints[j].orientation.wxyz.y;
                    skeletonSum.joints[j].orientation.wxyz.z = skeleton_buffer[i].joints[j].orientation.wxyz.z;
                    skeletonSum.joints[j].position.xyz.x = skeleton_buffer[i].joints[j].position.xyz.x;
                    skeletonSum.joints[j].position.xyz.y = skeleton_buffer[i].joints[j].position.xyz.y;
                    skeletonSum.joints[j].position.xyz.z = skeleton_buffer[i].joints[j].position.xyz.z;
                }
                else
                {
                    skeletonSum.joints[j].orientation.wxyz.w += skeleton_buffer[i].joints[j].orientation.wxyz.w;
                    skeletonSum.joints[j].orientation.wxyz.x += skeleton_buffer[i].joints[j].orientation.wxyz.x;
                    skeletonSum.joints[j].orientation.wxyz.y += skeleton_buffer[i].joints[j].orientation.wxyz.y;
                    skeletonSum.joints[j].orientation.wxyz.z += skeleton_buffer[i].joints[j].orientation.wxyz.z;
                    skeletonSum.joints[j].position.xyz.x += skeleton_buffer[i].joints[j].position.xyz.x;
                    skeletonSum.joints[j].position.xyz.y += skeleton_buffer[i].joints[j].position.xyz.y;
                    skeletonSum.joints[j].position.xyz.z += skeleton_buffer[i].joints[j].position.xyz.z;
                }
            }
        }

        for (size_t i = 0; i < K4ABT_JOINT_COUNT; i++)
        {
            skeletonAvg.joints[i].confidence_level = K4ABT_JOINT_CONFIDENCE_MEDIUM;
            skeletonAvg.joints[i].orientation.wxyz.w = skeletonSum.joints[i].orientation.wxyz.w / sampleNum;
            skeletonAvg.joints[i].orientation.wxyz.x = skeletonSum.joints[i].orientation.wxyz.x / sampleNum;
            skeletonAvg.joints[i].orientation.wxyz.y = skeletonSum.joints[i].orientation.wxyz.y / sampleNum;
            skeletonAvg.joints[i].orientation.wxyz.z = skeletonSum.joints[i].orientation.wxyz.z / sampleNum;
            skeletonAvg.joints[i].position.xyz.x = skeletonSum.joints[i].position.xyz.x / sampleNum;
            skeletonAvg.joints[i].position.xyz.y = skeletonSum.joints[i].position.xyz.y / sampleNum;
            skeletonAvg.joints[i].position.xyz.z = skeletonSum.joints[i].position.xyz.z / sampleNum;
        }
    }

    UINT64 timeStamp = getCurrentTimeMillis();
    char sendData[1024];
    int dataLen = 0;
    char *pOffset = sendData;
    pOffset += 9; // 把前9个字节提前让出来给身体数量和timestamp
    dataLen += 9;
    int sendRes = 0;

    // 身体数据详情：追踪ID（trackingID） UINT64（8字节，LE） | 关节点数组 （Joints[25]，关节点下标位置与其关节点类型对应，每个关节点13字节，共325字节）
    UINT64 trackingID = selectedID;
    memcpy_s(pOffset, sizeof(sendData), &trackingID, sizeof(trackingID));
    pOffset += sizeof(trackingID);
    for (auto j_type : joints_v2)
    {// 开始按顺序序列化关节点
     // 关节点详情：关节点的追踪状态 TrackingState（1字节） | 关节点在空间中x坐标 float（4字节，LE） | 关节点在空间中y坐标 float（4字节，LE） | 关节点在空间中z坐标 float（4字节，LE）
        byte state = 2;
        memcpy_s(pOffset, sizeof(sendData), &state, sizeof(state));
        pOffset += sizeof(state);
        if (j_type == K4ABT_JOINT_SPINE_CHEST) {
            float x = -1 * (skeletonAvg.joints[K4ABT_JOINT_SHOULDER_LEFT].position.xyz.x
                + skeletonAvg.joints[K4ABT_JOINT_SHOULDER_RIGHT].position.xyz.x) * 0.5 * 0.001;
            memcpy_s(pOffset, sizeof(sendData), &x, sizeof(x));
            pOffset += sizeof(x);
            float y = -1 * (skeletonAvg.joints[K4ABT_JOINT_SHOULDER_LEFT].position.xyz.y
                + skeletonAvg.joints[K4ABT_JOINT_SHOULDER_RIGHT].position.xyz.y) * 0.5 * 0.001;
            memcpy_s(pOffset, sizeof(sendData), &y, sizeof(y));
            pOffset += sizeof(y);
            float z = (skeletonAvg.joints[K4ABT_JOINT_SHOULDER_LEFT].position.xyz.z
                + skeletonAvg.joints[K4ABT_JOINT_SHOULDER_RIGHT].position.xyz.z) * 0.5 * 0.001;
            memcpy_s(pOffset, sizeof(sendData), &z, sizeof(z));
            pOffset += sizeof(z);
        }
        else {
            float x = -1 * skeletonAvg.joints[j_type].position.xyz.x * 0.001;
            memcpy_s(pOffset, sizeof(sendData), &x, sizeof(x));
            pOffset += sizeof(x);
            float y = -1 * skeletonAvg.joints[j_type].position.xyz.y * 0.001;
            memcpy_s(pOffset, sizeof(sendData), &y, sizeof(y));
            pOffset += sizeof(y);
            float z = skeletonAvg.joints[j_type].position.xyz.z * 0.001;
            memcpy_s(pOffset, sizeof(sendData), &z, sizeof(z));
            pOffset += sizeof(z);
        }
    }
    dataLen += 333;

    // 前面拼上身体数量和timestamp
    byte count = 1;
    memcpy_s(sendData, sizeof(sendData), &count, sizeof(count));
    memcpy_s((sendData + sizeof(count)), sizeof(sendData), &timeStamp, sizeof(timeStamp));
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


int CalculateAndSendSkeletonMA(k4abt_skeleton_t *skeleton_buffer, int sampleNum, float beta, int mvcount, k4a_float3_t *lastBodyAvg, uint32_t selectedID) {
    logFile << "Caculating moving average of skeleton. mvcount: " << mvcount 
        << "; last Right Hand position:(" << lastBodyAvg[K4ABT_JOINT_HAND_RIGHT].xyz.x
        << "," << lastBodyAvg[K4ABT_JOINT_HAND_RIGHT].xyz.y
        << "," << lastBodyAvg[K4ABT_JOINT_HAND_RIGHT].xyz.z << ")"
        << "; Right Hand position:(" << skeleton_buffer[1].joints[K4ABT_JOINT_HAND_RIGHT].position.xyz.x
        << "," << skeleton_buffer[1].joints[K4ABT_JOINT_HAND_RIGHT].position.xyz.y
        << "," << skeleton_buffer[1].joints[K4ABT_JOINT_HAND_RIGHT].position.xyz.z << ")" << "\n";
    logFile.flush();

    UINT64 start = getCurrentTimeMillis();
    for (size_t i = 0; i < sampleNum; i++)
    {
        for (size_t j = 0; j < K4ABT_JOINT_COUNT; j++)
        {
            float tempx = beta * lastBodyAvg[j].xyz.x + (1.0 - beta) * skeleton_buffer[i].joints[j].position.xyz.x;
            float tempy = beta * lastBodyAvg[j].xyz.y + (1.0 - beta) * skeleton_buffer[i].joints[j].position.xyz.y;
            float tempz = beta * lastBodyAvg[j].xyz.z + (1.0 - beta) * skeleton_buffer[i].joints[j].position.xyz.z;
            // Bias Correction
            float biasFactor = 1.0;
            //if (mvcount < 30)
            //{
            //    biasFactor = 1.0 / (1.0 - pow(beta, (mvcount + i)));
            //}
            lastBodyAvg[j].xyz.x = tempx * biasFactor;
            lastBodyAvg[j].xyz.y = tempy * biasFactor;
            lastBodyAvg[j].xyz.z = tempz * biasFactor;
        }
    }
    UINT64 end = getCurrentTimeMillis();
    logFile << "After moving average, last Right Hand position:(" 
        << lastBodyAvg[K4ABT_JOINT_HAND_RIGHT].xyz.x
        << "," << lastBodyAvg[K4ABT_JOINT_HAND_RIGHT].xyz.y
        << "," << lastBodyAvg[K4ABT_JOINT_HAND_RIGHT].xyz.z 
        << "); time cost: " << (end - start) << " ms\n";
    logFile.flush();

    UINT64 timeStamp = getCurrentTimeMillis();
    char sendData[1024];
    int dataLen = 0;
    char *pOffset = sendData;
    pOffset += 9; // 把前9个字节提前让出来给身体数量和timestamp
    dataLen += 9;
    int sendRes = 0;

    // 身体数据详情：追踪ID（trackingID） UINT64（8字节，LE） | 关节点数组 （Joints[25]，关节点下标位置与其关节点类型对应，每个关节点13字节，共325字节）
    UINT64 trackingID = selectedID;
    memcpy_s(pOffset, sizeof(sendData), &trackingID, sizeof(trackingID));
    pOffset += sizeof(trackingID);
    for (auto j_type : joints_v2)
    {// 开始按顺序序列化关节点
     // 关节点详情：关节点的追踪状态 TrackingState（1字节） | 关节点在空间中x坐标 float（4字节，LE） | 关节点在空间中y坐标 float（4字节，LE） | 关节点在空间中z坐标 float（4字节，LE）
        byte state = 2;
        memcpy_s(pOffset, sizeof(sendData), &state, sizeof(state));
        pOffset += sizeof(state);
        if (j_type == K4ABT_JOINT_SPINE_CHEST) {
            float x = -1 * (lastBodyAvg[K4ABT_JOINT_SHOULDER_LEFT].xyz.x
                + lastBodyAvg[K4ABT_JOINT_SHOULDER_RIGHT].xyz.x) * 0.5 * 0.001;
            memcpy_s(pOffset, sizeof(sendData), &x, sizeof(x));
            pOffset += sizeof(x);
            float y = -1 * (lastBodyAvg[K4ABT_JOINT_SHOULDER_LEFT].xyz.y
                + lastBodyAvg[K4ABT_JOINT_SHOULDER_RIGHT].xyz.y) * 0.5 * 0.001;
            memcpy_s(pOffset, sizeof(sendData), &y, sizeof(y));
            pOffset += sizeof(y);
            float z = (lastBodyAvg[K4ABT_JOINT_SHOULDER_LEFT].xyz.z
                + lastBodyAvg[K4ABT_JOINT_SHOULDER_RIGHT].xyz.z) * 0.5 * 0.001;
            memcpy_s(pOffset, sizeof(sendData), &z, sizeof(z));
            pOffset += sizeof(z);
        }
        else {
            float x = -1 * lastBodyAvg[j_type].xyz.x * 0.001;
            memcpy_s(pOffset, sizeof(sendData), &x, sizeof(x));
            pOffset += sizeof(x);
            float y = -1 * lastBodyAvg[j_type].xyz.y * 0.001;
            memcpy_s(pOffset, sizeof(sendData), &y, sizeof(y));
            pOffset += sizeof(y);
            float z = lastBodyAvg[j_type].xyz.z * 0.001;
            memcpy_s(pOffset, sizeof(sendData), &z, sizeof(z));
            pOffset += sizeof(z);
        }
    }
    dataLen += 333;

    // 前面拼上身体数量和timestamp
    byte count = 1;
    memcpy_s(sendData, sizeof(sendData), &count, sizeof(count));
    memcpy_s((sendData + sizeof(count)), sizeof(sendData), &timeStamp, sizeof(timeStamp));
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


void SendDataEnd() {
    char exitCode = { 99 };  // 方法退出前发送退出码，以防止前置机端阻塞
    int sendRes = send(clientSocket, &exitCode, sizeof(exitCode), 0);
    logFile << "Sending return code. Send Result: " << sendRes << " \n";
    logFile.flush();
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



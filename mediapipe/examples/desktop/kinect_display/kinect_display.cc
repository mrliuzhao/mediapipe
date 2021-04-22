#include <iostream>
#include <Windows.h>
#include <sys/timeb.h>
#include <iomanip>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "k4a/k4a.h"

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

int main(int argc, char** argv) {
    std::cout << "Going to open Kinect..." << std::endl;
    // 查询目前连接的设备数量
    uint32_t count = k4a_device_get_installed_count();
    if (count == 0)
    {
        std::cout << "test... Cannot find Any Installed K4A device!" << std::endl;
        return EXIT_FAILURE;
    }
    std::cout << "Find K4A device Count: " << count << std::endl;

    k4a_device_t kinect = NULL;
    // 尝试开启第一个设备
    k4a_result_t ret;
    ret = k4a_device_open(K4A_DEVICE_DEFAULT, &kinect);
    if (ret != K4A_RESULT_SUCCEEDED) {
        std::cout << "Fail to Open K4A device! Return Code: " << ret << std::endl;
        return EXIT_FAILURE;
    }

    // 对设备进行设置
    k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32; 
    // config.color_resolution = K4A_COLOR_RESOLUTION_720P;
    config.color_resolution = K4A_COLOR_RESOLUTION_1080P;
    config.camera_fps = K4A_FRAMES_PER_SECOND_30;
    // config.depth_mode = K4A_DEPTH_MODE_OFF;
    config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
    config.synchronized_images_only = true; // ensures that depth and color images are both available in the capture

    // Try to transform data
    k4a_calibration_t calibration;
    if (K4A_RESULT_SUCCEEDED !=
        k4a_device_get_calibration(kinect, config.depth_mode, config.color_resolution, &calibration))
    {
        printf("Failed to get calibration\n");
        goto Exit;
    }
    k4a_transformation_t transformation = k4a_transformation_create(&calibration);
    if (transformation == NULL) {
        printf("Failed to create transformation\n");
        goto Exit;
    }

    // Start the camera with the given configuration
    ret = k4a_device_start_cameras(kinect, &config);
    if (ret != K4A_RESULT_SUCCEEDED)
    {
        std::cout << "Fail to start k4a camera! Return Code: " << ret << std::endl;
        k4a_device_close(kinect);
        return EXIT_FAILURE;
    }

    cv::namedWindow("ColorFrameRaw", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("DepthImage", cv::WINDOW_AUTOSIZE);
    k4a_image_t colorFrameRaw;
    k4a_image_t depth_image;
    k4a_capture_t sensor_capture;
    unsigned long long lastCaptureTime = 0;
    while (true)
    {
        int32_t K4A_WAIT_TIMEOUT = 0;
        switch (k4a_device_get_capture(kinect, &sensor_capture, K4A_WAIT_TIMEOUT))
        {
            case K4A_WAIT_RESULT_SUCCEEDED:
                break;
            case K4A_WAIT_RESULT_TIMEOUT:
                // printf("Timed out waiting for a capture\n");
                continue;
                break;
            case K4A_WAIT_RESULT_FAILED:
                printf("Failed to read a capture\n");
                goto Exit;
        }

        // Probe for a depth image
        depth_image = k4a_capture_get_depth_image(sensor_capture);
        if (!depth_image)
        {
            printf("Failed to get depth image from capture\n");
            continue;
        }

        // Probe for a color image
        colorFrameRaw = k4a_capture_get_color_image(sensor_capture);
        if (!colorFrameRaw)
        {
            printf("Failed to get color image from capture\n");
            continue;
        }

        unsigned long long now = getCurrentTimeMillis();
        if (lastCaptureTime != 0) {
            std::cout << "Time Elapse: " << (now - lastCaptureTime) << std::endl;
        }
        lastCaptureTime = now;

        int height = k4a_image_get_height_pixels(colorFrameRaw);
        int width = k4a_image_get_width_pixels(colorFrameRaw);
        int stride = k4a_image_get_stride_bytes(colorFrameRaw);
        // printf("Color res: %4dx%4d; stride: %5d \n", height, width, stride);
        
        k4a_image_t transformed_depth_image = NULL;
        if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_DEPTH16,
                                                    width,
                                                    height,
                                                    width * (int)sizeof(uint16_t),
                                                    &transformed_depth_image))
        {
            printf("Failed to create transformed depth image\n");
            continue;
        }
        if (K4A_RESULT_SUCCEEDED !=
            k4a_transformation_depth_image_to_color_camera(transformation, depth_image, transformed_depth_image))
        {
            printf("Failed to compute transformed depth image\n");
            continue;
        }
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
                cv::imshow("DepthImage", depthImg);
            }
        }

        uint8_t* imgData = k4a_image_get_buffer(colorFrameRaw);
        cv::Mat colorFrame;
        colorFrame = cv::Mat(height, width, CV_8UC4, imgData);
        if (colorFrame.empty()) {
            continue;
        }
        cv::imshow("ColorFrameRaw", colorFrame);
        k4a_image_release(colorFrameRaw);

        if (cv::waitKey(1) == 27 || cv::waitKey(1) == 'q') {
            break;
        }

        // release capture
        k4a_capture_release(sensor_capture);
        fflush(stdout);
    }

Exit:
    cv::destroyAllWindows();
    // 关闭设备
    // k4a_device_stop_cameras(kinect);
    k4a_device_close(kinect);

    std::cout << "Close Kinect" << std::endl;

    if(!kinect){
        std::cout << "Going to Close Kinect Again" << std::endl;
        k4a_device_close(kinect);
    }

    return EXIT_SUCCESS;
}

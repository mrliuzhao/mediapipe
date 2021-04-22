#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

int main(int argc, char** argv) {
    std::cout << "Going to open Camera..." << std::endl;
    cv::VideoCapture capture;
    if(capture.open(0)){
        std::cout << "Successfully open Camera" << std::endl;
    } else
    {
        std::cout << "Fail to open Camera" << std::endl;
        return EXIT_FAILURE;
    }
    
    cv::namedWindow("CameraDisplay", /*flags=WINDOW_AUTOSIZE*/ 1);
// #if (CV_MAJOR_VERSION >= 3) && (CV_MINOR_VERSION >= 2)
//     capture.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
//     capture.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
//     capture.set(cv::CAP_PROP_FPS, 30);
// #endif

    bool grab_frames = true;
    while (grab_frames) {
        // Capture opencv camera or video frame.
        cv::Mat camera_frame_raw;
        capture >> camera_frame_raw;
        if (camera_frame_raw.empty()) {
            std::cout << "Ignore empty frames from camera." << std::endl;
            continue;
        }

        cv::imshow("CameraDisplay", camera_frame_raw);
        std::cout << "cols: " << camera_frame_raw.cols << "; rows: " << camera_frame_raw.rows << std::endl;

        // Press any key to exit.
        const int pressed_key = cv::waitKey(5);
        if (pressed_key >= 0 && pressed_key != 255) grab_frames = false;
    }
    cv::destroyAllWindows();
    return EXIT_SUCCESS;
}

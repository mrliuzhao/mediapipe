# Basic Kinect Display Demo. Verify reference of Azure Kinect libraries in Windows

licenses(["notice"])

package(default_visibility = ["//mediapipe/examples:__subpackages__"])

cc_binary(
    name = "DisplayKinectColor",
    srcs = ["kinect_display.cc"],
    deps = [
        "@windows_opencv//:opencv",
        "@kinect//:kinect",
    ],
)


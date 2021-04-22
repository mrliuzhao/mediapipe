# Run holistic tracking graph without facemesh and output Holistic landmarks. Then draw on origin image by OpenCV
# build cmd: bazel build -c opt --define MEDIAPIPE_DISABLE_GPU=1 --action_env PYTHON_BIN_PATH="C://python_37//python.exe" mediapipe/examples/desktop/holistic_tracking_data:holistic_tracking_data
# Run cmd: bazel-bin\mediapipe\examples\desktop\holistic_tracking_data\holistic_tracking_data.exe --calculator_graph_config_file=mediapipe/graphs/holistic_tracking/holistic_tracking_cpu_data.pbtxt

licenses(["notice"])

package(default_visibility = ["//mediapipe/examples:__subpackages__"])

cc_binary(
    name = "holistic_tracking_data",
    srcs = ["run_holistic_tracking_data_main.cc"],
    visibility = ["//visibility:public"],
    deps = [
        "@atl_mfc//:atl_mfc",
        "@kinect//:kinect",
        "//mediapipe/framework:calculator_framework",
        "//mediapipe/framework/formats:image_frame",
        "//mediapipe/framework/formats:image_frame_opencv",
        "//mediapipe/framework/formats:landmark_cc_proto",
        "//mediapipe/framework/port:file_helpers",
        "//mediapipe/framework/port:opencv_core",
        "//mediapipe/framework/port:opencv_highgui",
        "//mediapipe/framework/port:opencv_imgproc",
        "//mediapipe/framework/port:opencv_video",
        "//mediapipe/framework/port:parse_text_proto",
        "//mediapipe/framework/port:status",
        "//mediapipe/graphs/holistic_tracking:holistic_tracking_data_graph_deps",
    ],
)



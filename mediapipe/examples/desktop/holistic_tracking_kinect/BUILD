# Substitute Kinect Color Frame to Camera input. 
# build cmd: bazel build -c opt --define MEDIAPIPE_DISABLE_GPU=1 --action_env PYTHON_BIN_PATH="C://python_37//python.exe" mediapipe/examples/desktop/holistic_tracking_kinect:holistic_tracking_kinect
# Run cmd: bazel-bin\mediapipe\examples\desktop\holistic_tracking_kinect\holistic_tracking_kinect.exe --calculator_graph_config_file=mediapipe/graphs/holistic_tracking/holistic_tracking_cpu.pbtxt

licenses(["notice"])

package(default_visibility = ["//mediapipe/examples:__subpackages__"])

cc_binary(
    name = "holistic_tracking_kinect",
    deps = [
        "//mediapipe/examples/desktop:run_graph_kinect",
        "//mediapipe/graphs/holistic_tracking:holistic_tracking_cpu_graph_deps",
    ],
)

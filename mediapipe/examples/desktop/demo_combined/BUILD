licenses(["notice"])

package(default_visibility = ["//mediapipe/examples:__subpackages__"])

cc_binary(
    name = "POCDemo",
    srcs = ["POCDemo.cc", "CycleBuffer.h"],
    deps = [
        "@rabbitmq//:rabbitmq",
        "@boost//:boost",
        "@linux_SDL2//:libSDL2",
        "@linux_ffmpeg//:libffmpeg",
        "//mediapipe/framework:calculator_framework",
        "//mediapipe/framework/formats:image_frame",
        "//mediapipe/framework/formats:image_frame_opencv",
        "//mediapipe/framework/port:file_helpers",
        "//mediapipe/framework/port:opencv_imgproc",
        "//mediapipe/framework/port:opencv_video",
        "//mediapipe/framework/port:parse_text_proto",
        "//mediapipe/framework/port:status",
        "//mediapipe/gpu:gl_calculator_helper",
        "//mediapipe/gpu:gpu_buffer",
        "//mediapipe/gpu:gpu_shared_data_internal",
        "//mediapipe/graphs/pose_tracking:pose_tracking_gpu_deps"
    ],
    linkopts = [
        "-lpthread",
        "-luuid",
    ],
)

cc_binary(
    name = "BoostFormatTest",
    srcs = ["BoostFormatTest.cc"],
    deps = [
        "@boost//:boost",
    ],
    linkopts = [
        "-lpthread",
    ],
)



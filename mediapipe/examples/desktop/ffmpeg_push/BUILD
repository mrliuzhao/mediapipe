licenses(["notice"])

package(default_visibility = ["//mediapipe/examples:__subpackages__"])

# Demo only to push Camera video stream by FFMpeg
cc_binary(
    name = "CameraPusherVideo",
    srcs = [
        "CameraPusherVideo.cc"
    ],
    visibility = ["//visibility:public"],
    deps = [
        "@linux_ffmpeg//:libffmpeg",
    ],
)

# Demo where OpenCV is used to fetch Camera and FFMpeg is used to push image stream into RTMP
cc_binary(
    name = "CameraPusherOpenCV",
    srcs = [
        "CameraPusherOpenCV.cc"
    ],
    visibility = ["//visibility:public"],
    deps = [
        "@linux_ffmpeg//:libffmpeg",
        "//mediapipe/framework/port:opencv_imgproc",
        "//mediapipe/framework/port:opencv_video",
    ],
)

# Demo combines Mediapipe to track pose and FFMpeg to push image stream with pose landmarks into RTMP
cc_binary(
    name = "CameraPusherPose",
    srcs = [
        "CameraPusherPose.cc"
    ],
    visibility = ["//visibility:public"],
    deps = [
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
)

# Demo to both push Camera video and audio stream by FFMpeg
cc_binary(
    name = "CameraPusher",
    srcs = [
        "CameraPusher.cc"
    ],
    visibility = ["//visibility:public"],
    deps = [
        "@linux_ffmpeg//:libffmpeg",
    ],
)

# FFMpeg Official Demo to show muxing
cc_binary(
    name = "Muxing",
    srcs = [
        "Muxing.cc"
    ],
    visibility = ["//visibility:public"],
    deps = [
        "@linux_ffmpeg//:libffmpeg",
    ],
)



licenses(["notice"])

package(default_visibility = ["//mediapipe/examples:__subpackages__"])

# Demo to pull RTMP live stream by FFMpeg and display by SDL2
cc_binary(
    name = "RTMPPlayer",
    srcs = [
        "RTMPPlayer.cc",
        "CycleBuffer.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "@linux_ffmpeg//:libffmpeg",
        "@linux_SDL2//:libSDL2",
    ],
)

# Demo to open webcam with mic by FFMpeg and display by SDL2
cc_binary(
    name = "CameraPlayer",
    srcs = [
        "CameraPlayer.cc",
        "CycleBuffer.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "@linux_ffmpeg//:libffmpeg",
        "@linux_SDL2//:libSDL2",
    ],
)

# Demo to transcode microphone's audio stream by FFMpeg and save to file or push to RTMP
cc_binary(
    name = "CameraAudioSaver",
    srcs = [
        "CameraAudioSaver.cc",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "@linux_ffmpeg//:libffmpeg",
    ],
)


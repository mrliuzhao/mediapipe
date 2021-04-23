# Build cmd: bazel build mediapipe/examples/desktop/ffmpeg_tryout:HelloFFMpeg --verbose_failures
# Run cmd: ./bazel-bin/mediapipe/examples/desktop/ffmpeg_tryout/HelloFFMpeg

licenses(["notice"])

package(default_visibility = ["//mediapipe/examples:__subpackages__"])

# FFMpeg Hello world demo
cc_binary(
    name = "HelloFFMpeg",
    srcs = ["HelloFFMpeg.cc"],
    visibility = ["//visibility:public"],
    deps = [
        "@linux_ffmpeg//:libffmpeg",
    ],
)

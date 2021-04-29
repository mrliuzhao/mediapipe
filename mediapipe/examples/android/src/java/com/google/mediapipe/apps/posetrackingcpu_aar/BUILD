# aar library for pose_tracking_cpu
# Build cmd: bazel build -c opt --host_crosstool_top=@bazel_tools//tools/cpp:toolchain --fat_apk_cpu=arm64-v8a,armeabi-v7a --strip=ALWAYS mediapipe/examples/android/src/java/com/google/mediapipe/apps/posetrackingcpu_aar:pose_tracking_cpu_aar

load("//mediapipe/java/com/google/mediapipe:mediapipe_aar.bzl", "mediapipe_aar")

mediapipe_aar(
    name = "pose_tracking_cpu_aar",
    calculators = ["//mediapipe/graphs/pose_tracking:pose_tracking_cpu_mobile_deps"],
)



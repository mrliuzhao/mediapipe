# Build CMD: bazel build -c opt --define MEDIAPIPE_DISABLE_GPU=1 --action_env PYTHON_BIN_PATH="C://python_37//python.exe" mediapipe/examples/desktop/vrmeds_dll:VRMeds.dll

licenses(["notice"])

package(default_visibility = ["//mediapipe/examples:__subpackages__"])

load("//third_party:windows_dll_library.bzl", "windows_dll_library")
windows_dll_library(
    name = "VRMeds",
    srcs = ["VRMeds.cpp"],
    hdrs = [
        "targetver.h",
        "stdafx.h",
        "jni_md.h",
        "jni.h",
        "com_gdcccn_vrmeds_kinect_KinectCommunicator.h",
        "VRMeds.h",
    ],
    deps = [
        "@atl_mfc//:atl_mfc",
        "@kinect//:kinect",
        "@kinectBodyTracking//:kinectBodyTracking",
    ],
)

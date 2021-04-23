# Mediapipe detect 2D landmarks, and then use kinect fine Z value in order to generate 3D landmarks.

licenses(["notice"])

package(default_visibility = ["//mediapipe/examples:__subpackages__"])

load("//third_party:windows_dll_library.bzl", "windows_dll_library")
windows_dll_library(
    name = "VRMeds",
    srcs = ["VRMeds-mediapipe.cpp"],
    hdrs = [
        "jni_md.h",
        "jni.h",
        "com_gdcccn_vrmeds_kinect_KinectCommunicator.h",
        "VRMeds-mediapipe.h",
    ],
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

# cc_binary(
#     name = "vrmeds",
#     srcs = [
#         "jni_md.h",
#         "jni.h",
#         "com_gdcccn_vrmeds_kinect_KinectCommunicator.h",
#         "VRMeds-mediapipe.h",
#         "VRMeds-mediapipe.cpp",
#     ],
#     visibility = ["//visibility:public"],
#     deps = [
#         "@atl_mfc//:atl_mfc",
#         "@kinect//:kinect",
#         "//mediapipe/framework:calculator_framework",
#         "//mediapipe/framework/formats:image_frame",
#         "//mediapipe/framework/formats:image_frame_opencv",
#         "//mediapipe/framework/formats:landmark_cc_proto",
#         "//mediapipe/framework/port:file_helpers",
#         "//mediapipe/framework/port:opencv_core",
#         "//mediapipe/framework/port:opencv_highgui",
#         "//mediapipe/framework/port:opencv_imgproc",
#         "//mediapipe/framework/port:opencv_video",
#         "//mediapipe/framework/port:parse_text_proto",
#         "//mediapipe/framework/port:status",
#         "//mediapipe/graphs/holistic_tracking:holistic_tracking_data_graph_deps",
#     ],
# )

# cc_binary(
#     name = "VRMeds.dll",
#     srcs = [
#         "jni_md.h",
#         "jni.h",
#         "com_gdcccn_vrmeds_kinect_KinectCommunicator.h",
#         "VRMeds-mediapipe.h",
#         "VRMeds-mediapipe.cpp",
#     ],
#     deps = [
#         "@atl_mfc//:atl_mfc",
#         "@kinect//:kinect",
#         "//mediapipe/framework:calculator_framework",
#         "//mediapipe/framework/formats:image_frame",
#         "//mediapipe/framework/formats:image_frame_opencv",
#         "//mediapipe/framework/formats:landmark_cc_proto",
#         "//mediapipe/framework/port:file_helpers",
#         "//mediapipe/framework/port:opencv_core",
#         "//mediapipe/framework/port:opencv_highgui",
#         "//mediapipe/framework/port:opencv_imgproc",
#         "//mediapipe/framework/port:opencv_video",
#         "//mediapipe/framework/port:parse_text_proto",
#         "//mediapipe/framework/port:status",
#         "//mediapipe/graphs/holistic_tracking:holistic_tracking_data_graph_deps",
#     ],
#     linkshared = 1,
# )




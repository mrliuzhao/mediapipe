# POC demo in android

licenses(["notice"])

# Poc library 
android_library(
    name = "poc_lib",
    srcs = glob(["*.java"]),
    manifest = "AndroidManifest.xml",
    resource_files = glob(["res/**"]),
    visibility = ["//visibility:public"],
    deps = [
        "//mediapipe/framework/formats:landmark_java_proto_lite",
        "//mediapipe/java/com/google/mediapipe/components:android_camerax_helper",
        "//mediapipe/java/com/google/mediapipe/components:android_components",
        "//mediapipe/java/com/google/mediapipe/framework:android_framework",
        "//mediapipe/java/com/google/mediapipe/glutil",
        "//third_party:androidx_appcompat",
        "//third_party:androidx_constraint_layout",
        "//third_party:opencv",
        "@maven//:androidx_concurrent_concurrent_futures",
        "@maven//:com_google_guava_guava",
        "@com_google_protobuf//:protobuf_javalite",
    ],
)

cc_binary(
    name = "libmediapipe_jni.so",
    linkshared = 1,
    linkstatic = 1,
    deps = [
        "//mediapipe/graphs/pose_tracking:pose_tracking_cpu_mobile_deps",
        "//mediapipe/java/com/google/mediapipe/framework/jni:mediapipe_framework_jni",
    ],
)

cc_library(
    name = "mediapipe_jni_lib",
    srcs = [":libmediapipe_jni.so"],
    alwayslink = 1,
)

android_binary(
    name = "pocdemo",
    srcs = glob(["*.java"]),
    assets = [
        "//mediapipe/graphs/pose_tracking:pose_tracking_cpu_mobile.binarypb",
        "//mediapipe/modules/pose_landmark:pose_landmark_full_body.tflite",
        "//mediapipe/modules/pose_detection:pose_detection.tflite",
    ],
    assets_dir = "",
    manifest = "AndroidManifest.xml",
    resource_files = glob(["res/**"]),
    manifest_values = {
        "applicationId": "com.google.mediapipe.apps.poc",
        "minSdkVersion": "26",
        "targetSdkVersion": "30",
        # "appName": "POC Demo",
        "mainActivity": ".MainActivity",
        "cameraFacingFront": "False",
        "binaryGraphName": "pose_tracking_cpu_mobile.binarypb",
        "inputVideoStreamName": "input_video",
        "outputVideoStreamName": "output_video",
        "flipFramesVertically": "True",
        "converterNumBuffers": "2",
    },
    multidex = "native",
    deps = [
        ":mediapipe_jni_lib",
        "//mediapipe/framework/formats:landmark_java_proto_lite",
        "//mediapipe/java/com/google/mediapipe/components:android_camerax_helper",
        "//mediapipe/java/com/google/mediapipe/components:android_components",
        "//mediapipe/java/com/google/mediapipe/framework:android_framework",
        "//mediapipe/java/com/google/mediapipe/glutil",
        "//third_party:androidx_appcompat",
        "//third_party:androidx_constraint_layout",
        "//third_party:opencv",
        "@maven//:androidx_concurrent_concurrent_futures",
        "@maven//:com_google_guava_guava",
        "@com_google_protobuf//:protobuf_javalite",

        "@maven//:com_tencent_liteav_LiteAVSDK_TRTC",
        # "@maven//:com_rabbitmq_amqp_client",
    ],
)


# Description:
#   Azure Kinect Bodytracking SDK libraries for Kinect Operations on Windows

licenses(["notice"])  # BSD license

exports_files(["LICENSE"])

cc_library(
    name = "kinectBodyTracking",
    srcs = [
        "windows-desktop/amd64/release/lib/k4abt.lib",
        "windows-desktop/amd64/release/bin/k4abt.dll",
        "windows-desktop/amd64/release/bin/onnxruntime.dll",
    ],
    hdrs = glob(["include/**/*.h*"]),
    includes = ["include/"],
    linkstatic = 1,
    visibility = ["//visibility:public"],
)


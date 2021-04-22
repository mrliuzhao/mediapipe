# Description:
#   Azure Kinect SDK libraries for Kinect Operations on Windows

licenses(["notice"])  # BSD license

exports_files(["LICENSE"])

cc_library(
    name = "kinect",
    srcs = [
        "windows-desktop/amd64/release/lib/k4a.lib",
        "windows-desktop/amd64/release/lib/k4arecord.lib",
        "windows-desktop/amd64/release/bin/depthengine_2_0.dll",
        "windows-desktop/amd64/release/bin/k4a.dll",
        "windows-desktop/amd64/release/bin/k4arecord.dll",
    ],
    hdrs = glob(["include/**/*.h*"]),
    includes = ["include/"],
    linkstatic = 1,
    visibility = ["//visibility:public"],
)

# Description:
#   ATL MFC libraries for Kinect Operations on Windows

licenses(["notice"])  # BSD license

exports_files(["LICENSE"])

cc_library(
    name = "atl_mfc",
    srcs = glob([
        "lib/x64/*.lib",
        "lib/x64/*.dll",
    ]),
    hdrs = glob(["include/**/*.h*"]),
    includes = ["include/"],
    linkstatic = 1,
    visibility = ["//visibility:public"],
)


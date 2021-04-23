licenses(["notice"])

exports_files(["LICENSE"])

cc_library(
    name = "libSDL2",
    srcs = glob(
        [
            "lib/aarch64-linux-gnu/libSDL*.so",
        ],
    ),
    hdrs = glob(["include/SDL2/*.h"]),
    includes = ["include"],
    linkstatic = 1,
    visibility = ["//visibility:public"],
)

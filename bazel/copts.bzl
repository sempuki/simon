COPTS = select({
        "@bazel_tools//src/conditions:windows": ["/std:c++23"],
        "//conditions:default": ["-std=c++23", "-Isrc"],
})

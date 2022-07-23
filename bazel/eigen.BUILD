load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "eigen",
    hdrs =  glob(["Eigen/**", "unsupported/**"]),
    copts = ["-w", "-O3", "-pthread"],
    visibility = ["//visibility:public"],
)

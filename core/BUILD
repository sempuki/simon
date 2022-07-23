# Copyright 2022 -- CONTRIBUTORS. See LICENSE.

load("//:bazel/copts.bzl", "COPTS")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "math",
    hdrs = ["math.hpp"],
    deps = [
      "@eigen//:eigen"
    ],
    copts = COPTS,
)


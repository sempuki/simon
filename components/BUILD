# Copyright 2022 -- CONTRIBUTORS. See LICENSE.

load("//:bazel/copts.bzl", "COPTS")

package(default_visibility = ["//visibility:public"])

cc_library(
  name = "movement",
  hdrs= ["movement.hpp"],
  deps = [
    "//core:math",
    "//framework:component",
  ],
  copts = COPTS,
)

cc_test(
  name = "movement_test",
  srcs = ["movement_test.cpp"],
  deps = [
    "//framework:testing",
    ":movement",
  ],
  copts = COPTS,
)


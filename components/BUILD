# Copyright 2022 -- CONTRIBUTORS. See LICENSE.

load("//:bazel/copts.bzl", "COPTS")

package(default_visibility = ["//visibility:public"])

cc_library(
  name = "movement",
  hdrs= ["movement.hpp"],
  deps = [
    "//base:math",
    "//framework:component",
  ],
  copts = COPTS,
)

cc_test(
  name = "movement_test",
  srcs = ["movement_test.cpp"],
  deps = [
    "//base:testing",
    ":movement",
  ],
  copts = COPTS,
)


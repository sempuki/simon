# Copyright 2022 -- CONTRIBUTORS. See LICENSE.

load("//:bazel/copts.bzl", "COPTS")

package(default_visibility = ["//visibility:public"])

cc_library(
  name = "testing",
  hdrs= ["testing.hpp"],
  srcs= ["testing.cpp"],
  defines = ["CATCH_CONFIG_MAIN"],
  deps = [
    "@catch2//:catch2",
  ],
  copts = COPTS,
)

cc_library(
  name = "type_macros",
  hdrs= ["type_macros.hpp"],
  copts = COPTS,
)

cc_test(
  name = "type_macros_test",
  srcs = ["type_macros_test.cpp"],
  deps = [
    ":testing",
    ":type_macros",
  ],
  copts = COPTS,
)

cc_library(
    name = "math",
    hdrs = ["math.hpp"],
    deps = [
      "@eigen//:eigen"
    ],
    copts = COPTS,
)

cc_test(
  name = "math_test",
  srcs = ["math_test.cpp"],
  deps = [
    ":testing",
    ":math",
  ],
  copts = COPTS,
)

cc_library(
    name = "time",
    hdrs = ["time.hpp"],
    copts = COPTS,
)

cc_test(
  name = "time_test",
  srcs = ["time_test.cpp"],
  deps = [
    ":testing",
    ":time",
  ],
  copts = COPTS,
)


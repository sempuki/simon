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
  name = "base",
  hdrs= ["base.hpp"],
  copts = COPTS,
)

cc_library(
  name = "contract",
  hdrs= ["contract.hpp"],
  copts = COPTS,
)

cc_test(
  name = "contract_test",
  srcs = ["contract_test.cpp"],
  deps = [
    ":testing",
    ":contract",
  ],
  copts = COPTS,
)

cc_library(
    name = "result_code",
    hdrs = ["result_code.hpp"],
    deps = [
      ":base",
    ],
    copts = COPTS,
)

cc_test(
  name = "error_test",
  srcs = ["error_test.cpp"],
  deps = [
    ":testing",
    ":error",
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


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
    "@catch2//:catch2_main",
  ],
  copts = COPTS,
)

cc_library(
  name = "core",
  hdrs= ["core.hpp"],
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
    name = "status",
    hdrs = ["status.hpp"],
    deps = [
      ":core",
    ],
    copts = COPTS,
)

cc_test(
  name = "status_test",
  srcs = ["status_test.cpp"],
  deps = [
    ":testing",
    ":status",
  ],
  copts = COPTS,
)

cc_library(
    name = "platform_status",
    hdrs = ["platform_status.hpp"],
    srcs = ["platform_status.cpp"],
    deps = [
      ":status",
    ],
    copts = COPTS,
)

cc_test(
  name = "platform_status_test",
  srcs = ["platform_status_test.cpp"],
  deps = [
    ":testing",
    ":platform_status",
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


# Copyright 2022 -- CONTRIBUTORS. See LICENSE.

load("//:bazel/copts.bzl", "COPTS")

package(default_visibility = ["//visibility:public"])

cc_library(
  name = "controls",
  hdrs= ["controls.hpp"],
  deps = [
    "//base:math",
    "//framework:component",
  ],
  copts = COPTS,
)

cc_test(
  name = "controls_test",
  srcs = ["controls_test.cpp"],
  deps = [
    "//base:testing",
    ":controls",
  ],
  copts = COPTS,
)

cc_library(
  name = "environment",
  hdrs= ["environment.hpp"],
  deps = [
    "//base:math",
    "//framework:component",
  ],
  copts = COPTS,
)

cc_test(
  name = "environment_test",
  srcs = ["environment_test.cpp"],
  deps = [
    "//base:testing",
    ":environment",
  ],
  copts = COPTS,
)

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

cc_library(
  name = "physical",
  hdrs= ["physical.hpp"],
  deps = [
    "//base:math",
    "//framework:component",
  ],
  copts = COPTS,
)

cc_test(
  name = "physical_test",
  srcs = ["physical_test.cpp"],
  deps = [
    "//base:testing",
    ":physical",
  ],
  copts = COPTS,
)


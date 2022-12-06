# Copyright 2022 -- CONTRIBUTORS. See LICENSE.

load("//:bazel/copts.bzl", "COPTS")

package(default_visibility = ["//visibility:public"])

cc_library(
  name = "identity",
  srcs= ["identity.cpp"],
  hdrs= ["identity.hpp"],
  deps = [
    "//base:type_macros",
  ],
  copts = COPTS,
)

cc_test(
  name = "identity_test",
  srcs = ["identity_test.cpp"],
  deps = [
    "//base:testing",
    ":identity",
  ],
  copts = COPTS,
)

cc_library(
  name = "entity",
  hdrs= ["entity.hpp"],
  deps = [
    ":identity",
    ":component",
  ],
  copts = COPTS,
)

cc_test(
  name = "entity_test",
  srcs = ["entity_test.cpp"],
  deps = [
    "//base:testing",
    ":entity",
  ],
  copts = COPTS,
)

cc_library(
  name = "event",
  hdrs= ["event.hpp"],
  deps = [
    "//base:time",
    ":identity",
  ],
  copts = COPTS,
)

cc_test(
  name = "event_test",
  srcs = ["event_test.cpp"],
  deps = [
    "//base:testing",
    ":event",
  ],
  copts = COPTS,
)

cc_library(
  name = "event_queue",
  hdrs= ["event_queue.hpp"],
  deps = [
    "//base:time",
    ":event",
  ],
  copts = COPTS,
)

cc_test(
  name = "event_queue_test",
  srcs = ["event_queue_test.cpp"],
  deps = [
    "//base:testing",
    ":event_queue",
  ],
  copts = COPTS,
)

cc_library(
  name = "component",
  hdrs= ["component.hpp"],
  deps = [
    ":identity",
  ],
  copts = COPTS,
)

cc_test(
  name = "component_test",
  srcs = ["component_test.cpp"],
  deps = [
    "//base:testing",
    ":component",
  ],
  copts = COPTS,
)

cc_library(
  name = "component_system",
  hdrs= ["component_system.hpp"],
  deps = [
    "//base:time",
    ":entity",
    ":component",
  ],
  copts = COPTS,
)

cc_test(
  name = "component_system_test",
  srcs = ["component_system_test.cpp"],
  deps = [
    "//base:testing",
    ":component_system",
  ],
  copts = COPTS,
)


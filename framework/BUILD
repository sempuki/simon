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
  name = "identity",
  srcs= ["identity.cpp"],
  hdrs= ["identity.hpp"],
  deps = [
    ":type_macros",
  ],
  copts = COPTS,
)

cc_test(
  name = "identity_test",
  srcs = ["identity_test.cpp"],
  deps = [
    ":testing",
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
    ":testing",
    ":entity",
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
    ":testing",
    ":component",
  ],
  copts = COPTS,
)

cc_library(
  name = "component_system",
  hdrs= ["component_system.hpp"],
  deps = [
    ":entity",
    ":component",
  ],
  copts = COPTS,
)

cc_test(
  name = "component_system_test",
  srcs = ["component_system_test.cpp"],
  deps = [
    ":testing",
    ":component_system",
  ],
  copts = COPTS,
)


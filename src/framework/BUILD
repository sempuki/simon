# Copyright 2022 -- CONTRIBUTORS. See LICENSE.

load("//src:copts.bzl", "COPTS")

cc_library(
    name = "contract",
    hdrs= ["contract.hpp"],
)

cc_test(
    name = "contract_test",
    srcs = ["contract_test.cpp"],
    copts = COPTS,
    defines = ["CATCH_CONFIG_MAIN"],
    deps = [
        "@catch2//:catch2",
        ":contract",
    ],
)

cc_library(
    name = "type_macros",
    hdrs= ["type_macros.hpp"],
    copts = COPTS,
)

cc_test(
    name = "type_macros_test",
    srcs = ["type_macros_test.cpp"],
    copts = COPTS,
    defines = ["CATCH_CONFIG_MAIN"],
    deps = [
        "@catch2//:catch2",
        ":type_macros",
    ],
)

cc_library(
    name = "identity",
    srcs= ["identity.cpp"],
    hdrs= ["identity.hpp"],
    copts = COPTS,
    deps = [
        ":type_macros",
    ],
)

cc_test(
    name = "identity_test",
    srcs = ["identity_test.cpp"],
    copts = COPTS,
    defines = ["CATCH_CONFIG_MAIN"],
    deps = [
        "@catch2//:catch2",
        ":identity",
    ],
)

cc_library(
    name = "entity",
    hdrs= ["entity.hpp"],
    copts = COPTS,
    deps = [
        ":identity",
        ":component",
    ],
)

cc_test(
    name = "entity_test",
    srcs = ["entity_test.cpp"],
    copts = COPTS,
    defines = ["CATCH_CONFIG_MAIN"],
    deps = [
        "@catch2//:catch2",
        ":entity",
    ],
)

cc_library(
    name = "component",
    hdrs= ["component.hpp"],
    copts = COPTS,
    deps = [
        ":identity",
    ],
)

cc_test(
    name = "component_test",
    srcs = ["component_test.cpp"],
    copts = COPTS,
    defines = ["CATCH_CONFIG_MAIN"],
    deps = [
        "@catch2//:catch2",
        ":component",
    ],
)

cc_library(
    name = "component_system",
    hdrs= ["component_system.hpp"],
    copts = COPTS,
    deps = [
        ":entity",
        ":component",
    ],
)

cc_test(
    name = "component_system_test",
    srcs = ["component_system_test.cpp"],
    copts = COPTS,
    defines = ["CATCH_CONFIG_MAIN"],
    deps = [
        "@catch2//:catch2",
        ":component_system",
    ],
)


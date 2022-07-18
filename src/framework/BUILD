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


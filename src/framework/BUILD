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

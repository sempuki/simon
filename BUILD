load("//:bazel/copts.bzl", "COPTS")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "hello",
    srcs = ["hello.cpp"],
    copts = COPTS,
)

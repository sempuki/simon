load("//:bazel/copts.bzl", "COPTS")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "hello",
    srcs = ["hello.cpp"],
    deps = [
      "//base:time",
      "//base:contract",
      "//component:controls",
      "//component:environment",
      "//component:movement",
      "//component:physical",
      "//framework:entity",
      "//framework:event_queue",
      "//framework:component_system",
      "@imgui//:imgui",
    ],
    copts = COPTS + [
      "-I/usr/local/include",
      "-D_THREAD_SAFE"
    ],
    linkopts = [
      "-L/usr/local/lib",
      "-lSDL2",
    ],
)

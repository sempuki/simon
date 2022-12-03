load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "imgui",
    hdrs = glob(["*.h"]) + [
      "backends/imgui_impl_sdl.h", 
      "backends/imgui_impl_sdlrenderer.h"
    ],
    srcs = glob(["*.cpp"]) + [
      "backends/imgui_impl_sdl.cpp", 
      "backends/imgui_impl_sdlrenderer.cpp"
    ],
    copts = [
      "-O2",
      "-I/usr/include/SDL2",
      "-I/usr/local/include/SDL2",
      "-D_THREAD_SAFE"
    ],
    visibility = ["//visibility:public"],
)

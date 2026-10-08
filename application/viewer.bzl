# Copyright 2026 -- CONTRIBUTORS. See LICENSE.

"""An application's viewer, compiled once for its binary and its tests."""

load("@lib//bazel:copts.bzl", "COPTS")
load("@rules_cc//cc:cc_binary.bzl", "cc_binary")
load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")

def application_viewer(deps, data = [], tests = {}):
    """Builds viewer.cpp once, as viewer_library, and links it into the viewer
    binary and into one test per entry of `tests`, its name to its arguments.
    Each test runs the viewer without a display, on SDL's dummy video driver,
    for the frames its arguments ask.

    Args:
      deps: what viewer.cpp depends on.
      data: what the viewer reads at run time.
      tests: each test's name to the viewer's arguments.
    """
    cc_library(
        name = "viewer_library",
        srcs = ["viewer.cpp"],
        copts = COPTS,
        deps = deps,
        # It holds main, which nothing in the binary refers to.
        alwayslink = True,
    )
    cc_binary(
        name = "viewer",
        data = data,
        deps = [":viewer_library"],
    )
    for name, args in tests.items():
        cc_test(
            name = name,
            size = "small",
            args = args,
            data = data,
            env = {"SDL_VIDEODRIVER": "dummy"},
            deps = [":viewer_library"],
        )

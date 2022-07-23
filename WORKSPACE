workspace(name = "Simon")

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

http_archive(
    name = "catch2",
    urls = ["https://github.com/catchorg/Catch2/archive/v2.13.8.tar.gz"],
    strip_prefix = "Catch2-2.13.8",
    sha256 = "b9b592bd743c09f13ee4bf35fc30eeee2748963184f6bea836b146e6cc2a585a"
)

http_archive(
    name = "eigen",
    urls = ["https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz"],
    strip_prefix = "eigen-3.4.0",
    sha256 = "8586084f71f9bde545ee7fa6d00288b264a2b7ac3607b974e54d13e7162c1c72",
    build_file = "//:bazel/eigen.BUILD",
)


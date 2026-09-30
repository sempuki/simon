# simon

Out of the Box: a small entity-component simulation rendered with Dear ImGui on
SDL2.

## Build

Requires [Bazelisk](https://github.com/bazelbuild/bazelisk) (installed as
`bazel`); the Bazel release is pinned in `.bazelversion`. All dependencies,
SDL2 included, come from the Bazel Central Registry, except the shared core
libraries in the [lib](https://github.com/sempuki/lib) submodule:

```sh
git clone --recurse-submodules git@github.com:sempuki/simon.git
# or, in an existing clone:
git submodule update --init

bazel test //...
bazel run //application/hello
bazel run //application/missile:viewer   # watch a missile scenario
bazel run //application/missile -- 7    # run seed 7 headless
```

Code targets C++26; flags come from `@lib//bazel:copts.bzl`.

The architecture, its decisions and the roadmap are in
[documents/design.md](documents/design.md).

## Editor setup

clangd needs a `compile_commands.json`. Generate it from the workspace root,
and again after adding files, targets or dependencies:

```sh
python3 2nd_party/lib/bazel/compile_commands.py
```

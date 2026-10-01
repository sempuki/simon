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
bazel run //application/flight -- 1000 100   # 1,000 aircraft, 100 on RK4
bazel run -c opt //application/flight:flight_benchmark
```

Code targets C++26; flags come from `@lib//bazel:copts.bzl`.

The architecture, its decisions and the roadmap are in
[documents/design.md](documents/design.md).

## Editor setup

clangd needs a `compile_commands.json`, and the headers it names must stay put.
Bazel's execution root does not: every build relinks it to only the external
repositories that build needed. `tools/lsp/mirror.py` builds in an output base
of its own, copies the headers clangd reads into `.lsp/mirror/` (ignored by git
and Bazel), and writes `compile_commands.json` against that mirror, so builds
and compiler switches never disturb your editor:

```sh
python3 tools/lsp/mirror.py                  # build the mirror now
python3 tools/lsp/mirror.py --if-stale       # only if files or targets changed
python3 tools/lsp/mirror.py --watch 60       # check every minute, e.g. in a tmux pane
python3 tools/lsp/mirror.py --install-hooks  # refresh after checkout, merge, rebase
```

`--if-stale` takes a fraction of a second when nothing changed, so it is cheap
to run often. Restart clangd (`:LspRestart` in Neovim) after the first build.

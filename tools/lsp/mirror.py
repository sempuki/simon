#!/usr/bin/env python3
# Copyright 2022 -- CONTRIBUTORS. See LICENSE.

"""Mirrors what clangd needs from Bazel into .lsp/, outside Bazel's reach.

    python3 tools/lsp/mirror.py                  # refresh now
    python3 tools/lsp/mirror.py --if-stale       # refresh only if out of date
    python3 tools/lsp/mirror.py --watch 60       # check every 60 s, forever
    python3 tools/lsp/mirror.py --install-hooks  # refresh after checkout, merge

clangd reads headers through the paths in compile_commands.json. Pointing
those into Bazel's execution root breaks it: every build relinks that tree to
just the external repositories the build needed, and switching compilers
reconfigures it. So this tool builds in an output base of its own, copies the
headers under every include directory the compile commands name into
.lsp/mirror/, and writes compile_commands.json against the mirror. Builds,
tests and compiler switches elsewhere never touch it.

- Symlinks are dereferenced, so the mirror stands alone. Repositories that are
  local directories, such as lib (2nd_party/lib), are not copied: the compile
  commands point at them, so go-to-definition opens the real file, and their
  own sources get compile commands too.
- A refresh builds the new mirror beside the old one and swaps it in, so
  clangd never sees half a mirror. A lock keeps refreshes from overlapping.
- --if-stale refreshes only when the build graph or the set of C++ files
  changed since the last refresh, so it is cheap to run often.
"""

import argparse
import fcntl
import hashlib
import json
import os
import pathlib
import shutil
import subprocess
import sys
import time

WORKSPACE = pathlib.Path(__file__).resolve().parents[2]
LSP = WORKSPACE / ".lsp"
MIRROR = LSP / "mirror"
STAMP = LSP / "stamp"
LOCK = LSP / "lock"
LOG = LSP / "refresh.log"
COMPILE_COMMANDS = WORKSPACE / "compile_commands.json"

# An output base of its own, outside the workspace, so the mirror's builds
# never share an execution root with anyone else's.
OUTPUT_BASE = pathlib.Path(
    os.environ.get(
        "LSP_OUTPUT_BASE",
        pathlib.Path.home()
        / ".cache"
        / "bazel-lsp"
        / (WORKSPACE.name + "-" + hashlib.sha1(str(WORKSPACE).encode()).hexdigest()[:8]),
    )
)

# Clang's flags, which clangd understands; GCC's can include options it does not.
BAZEL_FLAGS = ["--repo_env=CC=clang"]

# What clangd may open through an include directory.
HEADER_SUFFIXES = {".h", ".hh", ".hpp", ".hxx", ".h++", ".inl", ".ipp", ".inc", ".def", ".tcc"}
# Extensionless files are headers too, such as Eigen/Dense, but only in
# external repositories (bazel-out holds extensionless executables), and only
# small ones (a repository's root also holds scripts and licenses).
EXTENSIONLESS_LIMIT = 256 * 1024
CXX_SUFFIXES = {".c", ".cc", ".cpp", ".cxx", ".h", ".hh", ".hpp", ".hxx"}

# Flags whose value is an include directory, joined or as the next argument.
INCLUDE_FLAGS = ("-isystem", "-iquote", "-idirafter", "-I")

# Directories under bazel-out that hold no headers worth reading.
SKIPPED_DIRECTORIES = {"_objs", "_solib_k8", "testlogs"}


def bazel(*arguments: str, capture: bool = True) -> str:
    command = ["bazel", f"--output_base={OUTPUT_BASE}", *arguments]
    if capture:
        return subprocess.check_output(command, text=True, cwd=WORKSPACE)
    subprocess.check_call(command, cwd=WORKSPACE)
    return ""


def fingerprint() -> str:
    """What the compile commands depend on: the build graph and the set of C++
    files (not their contents, which clangd reads itself)."""
    digest = hashlib.sha256()
    roots = [WORKSPACE, WORKSPACE / "2nd_party" / "lib"]
    for name in ("MODULE.bazel", "MODULE.bazel.lock", ".bazelrc", ".bazelversion"):
        for root in roots:
            path = root / name
            if path.is_file():
                digest.update(str(path).encode() + path.read_bytes())
    for root in roots:
        for path in sorted(root.rglob("*")):
            if any(part.startswith((".", "bazel-")) for part in path.relative_to(root).parts):
                continue
            if path.name in ("BUILD", "BUILD.bazel") or path.suffix == ".bzl":
                digest.update(str(path).encode() + path.read_bytes())
            elif path.suffix in CXX_SUFFIXES:
                digest.update(str(path).encode())
    digest.update(pathlib.Path(__file__).read_bytes())
    return digest.hexdigest()


def include_directories(arguments: list[str]) -> set[str]:
    directories = set()
    for i, argument in enumerate(arguments):
        for flag in INCLUDE_FLAGS:
            if argument == flag and i + 1 < len(arguments):
                directories.add(arguments[i + 1])
            elif argument.startswith(flag) and argument != flag:
                directories.add(argument[len(flag):])
    return {d for d in directories if d.startswith(("external/", "bazel-out/"))}


def local_repositories(execution_root: pathlib.Path) -> dict[str, pathlib.Path]:
    """External repositories that are local directories, such as lib, by name:
    the compile commands point at them instead of at a copy."""
    local = {}
    external = execution_root / "external"
    for entry in external.iterdir() if external.is_dir() else []:
        target = entry.resolve()
        if "/external/" in str(target) or "/cache/repos/" in str(target) or "/install/" in str(target):
            continue  # Fetched or built in, so copied.
        local[entry.name] = target
    return local


def is_header(path: pathlib.Path, external: bool) -> bool:
    if not path.is_file():
        return False
    if path.suffix in HEADER_SUFFIXES:
        return True
    return (external and not path.suffix and not path.name.startswith(".")
            and path.stat().st_size <= EXTENSIONLESS_LIMIT)


class Mirror:
    """Places headers in a staging mirror so that each header exists once.

    A header reached through a symlink, as in Bazel's _virtual_includes,
    becomes a symlink to the one copy of what it names: the mirrored copy of a
    fetched repository's file, or the real file in the workspace or a local
    repository. Two copies of one header would defeat #pragma once."""

    def __init__(self, staging: pathlib.Path, execution_root: pathlib.Path,
                 local: dict[str, pathlib.Path]) -> None:
        self.staging = staging
        self.local_roots = [WORKSPACE, *local.values()]
        # Where each fetched repository really lives, to map a resolved file
        # back to its place in the mirror.
        self.execution_root = execution_root
        self.fetched_roots = {}
        external = execution_root / "external"
        for entry in external.iterdir() if external.is_dir() else []:
            if entry.name not in local:
                self.fetched_roots[entry.resolve()] = pathlib.Path("external") / entry.name
        self.copied = 0

    def place(self, path: pathlib.Path, at: pathlib.Path) -> None:
        """Puts header `path` at `at`, relative to the mirror's root."""
        target = self.staging / at
        if target.exists() or target.is_symlink():
            return
        real = path.resolve()
        canonical = self.canonical(real)
        target.parent.mkdir(parents=True, exist_ok=True)
        if canonical is None:
            # Generated: it exists nowhere else, so this is its one copy.
            shutil.copyfile(real, target)
            self.copied += 1
        elif isinstance(canonical, pathlib.Path) and canonical.is_absolute():
            target.symlink_to(canonical)  # The workspace's, or a local repository's.
        elif canonical == at:
            shutil.copyfile(real, target)
            self.copied += 1
        else:
            self.place(real, canonical)
            target.symlink_to(os.path.relpath(self.staging / canonical, target.parent))

    def canonical(self, real: pathlib.Path) -> pathlib.Path | None:
        """The one place `real` belongs: its absolute path if it is in the
        workspace or a local repository, its path in the mirror if it is in a
        fetched repository, or nothing if it is generated."""
        for root in self.local_roots:
            if real.is_relative_to(root):
                return real
        for root, mirrored in self.fetched_roots.items():
            if real.is_relative_to(root):
                return mirrored / real.relative_to(root)
        return None

    def copy_headers(self, directory: str, external: bool) -> None:
        """Places the headers under include directory `directory`, relative to
        the execution root `source`."""
        source = self.execution_root / directory
        for walked, subdirectories, files in os.walk(source, followlinks=True):
            subdirectories[:] = [
                d for d in subdirectories
                if d not in SKIPPED_DIRECTORIES and not d.endswith(".runfiles") and not d.startswith(".")
            ]
            for name in files:
                path = pathlib.Path(walked) / name
                if is_header(path, external):
                    self.place(path, pathlib.Path(directory) / path.relative_to(source))


def rewrite(argument: str, local: dict[str, pathlib.Path]) -> str:
    """A path relative to Bazel's execution root, made relative to the
    workspace through the mirror, or a local repository's own directory."""
    for flag in ("", *INCLUDE_FLAGS):
        if not argument.startswith(flag):
            continue
        path = argument[len(flag):]
        if path.startswith("external/"):
            repository, _, rest = path[len("external/"):].partition("/")
            if repository in local:
                return flag + str(local[repository] / rest).rstrip("/")
            return flag + str(MIRROR.relative_to(WORKSPACE) / path)
        if path.startswith("bazel-out/"):
            return flag + str(MIRROR.relative_to(WORKSPACE) / path)
    return argument


def refresh(targets: list[str]) -> None:
    LSP.mkdir(exist_ok=True)
    bazel("build", *BAZEL_FLAGS, *targets, capture=False)  # Generated headers.
    execution_root = pathlib.Path(bazel("info", *BAZEL_FLAGS, "execution_root").strip())
    # Dependencies too, for the sources of local repositories such as lib;
    # fetched repositories' sources are skipped below.
    query = 'mnemonic("CppCompile", deps({}))'.format(" + ".join(targets))
    graph = json.loads(
        bazel("aquery", *BAZEL_FLAGS, query, "--output=jsonproto", "--include_artifacts=false")
    )
    local = local_repositories(execution_root)

    commands = {}
    directories = set()
    for action in graph.get("actions", []):
        arguments = action["arguments"]
        if "-c" not in arguments:
            continue  # Not a compile of one source, such as a header check.
        source = arguments[arguments.index("-c") + 1]
        file = None
        if source.startswith("external/"):
            repository, _, rest = source[len("external/"):].partition("/")
            if repository in local:
                file = local[repository] / rest  # A local repository's own source.
        elif not source.startswith("bazel-out/"):
            file = WORKSPACE / source
        if file is None or str(file) in commands:
            continue
        directories |= include_directories(arguments)
        commands[str(file)] = {
            "directory": str(WORKSPACE),
            "file": str(file),
            "arguments": [rewrite(a, local) if a != source else str(file) for a in arguments],
        }

    # Build the new mirror beside the old one. Leftovers of a refresh that
    # failed partway go first; a failure here leaves the old mirror in use.
    for leftover in LSP.glob("mirror.*"):
        shutil.rmtree(leftover, ignore_errors=True)
    staging = LSP / f"mirror.{os.getpid()}"
    staging.mkdir()
    mirror = Mirror(staging, execution_root, local)
    try:
        for directory in sorted(directories):
            repository = directory.split("/")[1] if directory.startswith("external/") else None
            if repository in local or not (execution_root / directory).is_dir():
                continue
            mirror.copy_headers(directory, external=directory.startswith("external/"))
        copied = mirror.copied
    except BaseException:
        shutil.rmtree(staging, ignore_errors=True)
        raise

    # Swap the new mirror in, then the compile commands that name it.
    previous = LSP / f"mirror.old.{os.getpid()}"
    if MIRROR.exists():
        MIRROR.rename(previous)
    staging.rename(MIRROR)
    shutil.rmtree(previous, ignore_errors=True)
    temporary = COMPILE_COMMANDS.with_suffix(".json.tmp")
    temporary.write_text(json.dumps(list(commands.values()), indent=2) + "\n")
    os.replace(temporary, COMPILE_COMMANDS)
    print(f"Mirrored {copied} headers from {len(directories)} include directories; "
          f"wrote {len(commands)} compile commands to {COMPILE_COMMANDS.name}")


def refresh_if_stale(targets: list[str], force: bool) -> None:
    LSP.mkdir(exist_ok=True)
    with open(LOCK, "w") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            print("A refresh is already running.")
            return
        current = fingerprint()
        if not force and STAMP.is_file() and STAMP.read_text() == current and MIRROR.is_dir():
            return
        refresh(targets)
        STAMP.write_text(current)


HOOK = """#!/bin/sh
# Installed by tools/lsp/mirror.py --install-hooks: refreshes the clangd mirror
# in the background when this checkout changes.
cd "$(git rev-parse --show-toplevel)" || exit 0
mkdir -p .lsp
nohup python3 tools/lsp/mirror.py --if-stale >> .lsp/refresh.log 2>&1 &
"""


def install_hooks() -> None:
    hooks = pathlib.Path(
        subprocess.check_output(["git", "rev-parse", "--git-path", "hooks"], text=True, cwd=WORKSPACE).strip()
    )
    hooks = hooks if hooks.is_absolute() else WORKSPACE / hooks
    hooks.mkdir(parents=True, exist_ok=True)
    LSP.mkdir(exist_ok=True)
    for name in ("post-checkout", "post-merge", "post-rewrite"):
        hook = hooks / name
        if hook.exists() and "tools/lsp/mirror.py" not in hook.read_text():
            print(f"Left {hook} alone: it is not ours.")
            continue
        hook.write_text(HOOK)
        hook.chmod(0o755)
        print(f"Installed {hook}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("targets", nargs="*", default=["//..."], help="Bazel target patterns")
    parser.add_argument("--if-stale", action="store_true", help="refresh only if out of date")
    parser.add_argument("--watch", type=int, metavar="SECONDS", help="check every SECONDS, forever")
    parser.add_argument("--install-hooks", action="store_true", help="refresh after checkout and merge")
    options = parser.parse_args()

    if options.install_hooks:
        install_hooks()
        return
    if options.watch:
        while True:
            try:
                refresh_if_stale(options.targets, force=False)
            except subprocess.CalledProcessError as error:
                print(f"Refresh failed ({error}); trying again later.", file=sys.stderr)
            time.sleep(options.watch)
    refresh_if_stale(options.targets, force=not options.if_stale)


if __name__ == "__main__":
    main()

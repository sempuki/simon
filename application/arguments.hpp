// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

// The arguments a program's main reads: positional ones by their index,
// and options, `--name=value` or a bare `--name`, by name. Each read gives a
// fallback for an argument that is not there, and remembers the first one
// that is there but malformed, so a main reads them all and then checks
// error() once, refusing an option or a positional argument it never read.
namespace simon::application {

class Arguments final {
 public:
  Arguments(int argc, char** argv);

  // The positional argument at `index`, or `fallback`.
  auto text(std::size_t index, std::string_view fallback) -> std::string_view;

  // The positional argument at `index` as a whole number, or `fallback`.
  auto integer(std::size_t index, std::int64_t fallback) -> std::int64_t;

  // The positional argument at `index` as a number, or `fallback`.
  auto number(std::size_t index, double fallback) -> double;

  // The option `--name=value`'s value, or `fallback`.
  auto option_text(std::string_view name, std::string_view fallback)
      -> std::string_view;

  auto option_integer(std::string_view name, std::int64_t fallback)
      -> std::int64_t;

  auto option_number(std::string_view name, double fallback) -> double;

  // Whether the bare option `--name` is given.
  auto flag(std::string_view name) -> bool;

  // What is wrong, if anything: the first argument read that is not the
  // number it should be, an option never read, or a positional argument past
  // the last one read.
  auto error() const -> std::optional<std::string>;

  // Prints error() and `usage` to standard error if there is an error, and
  // returns whether there is.
  auto report_error(std::string_view usage) const -> bool;

 private:
  struct Option final {
    std::string_view name;
    std::optional<std::string_view> value;  // Nothing for a bare flag.
    bool read = false;
  };

  auto find(std::string_view name) -> Option*;
  auto note(std::string message) -> void;

  std::vector<std::string_view> positional_;
  std::vector<Option> options_;
  std::size_t positional_read_ = 0;  // One past the last index read.
  std::optional<std::string> error_;
};

}  // namespace simon::application

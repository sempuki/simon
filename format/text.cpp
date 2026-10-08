// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include "format/text.hpp"

#include <algorithm>
#include <fstream>
#include <sstream>

namespace simon::format {

auto read_text_file(const std::string& path)
    -> std::expected<std::string, lib::Status> {
  std::ifstream file{path};
  if (!file) {
    return Failure{lib::raise(FormatError::UNREADABLE, "cannot open " + path)};
  }
  std::stringstream text;
  text << file.rdbuf();
  return text.str();
}

auto find_line(std::string_view text, std::ptrdiff_t offset) -> std::size_t {
  auto end = static_cast<std::size_t>(std::clamp<std::ptrdiff_t>(
      offset, 0, static_cast<std::ptrdiff_t>(text.size())));
  return 1 + static_cast<std::size_t>(
                 std::count(text.begin(), text.begin() + end, '\n'));
}

auto fail_at(std::size_t line, std::string_view why) -> Failure {
  return Failure{
      lib::raise(FormatError::MALFORMED,
                 "line " + std::to_string(line) + ": " + std::string{why})};
}

auto refuse(std::string_view what) -> Failure {
  return Failure{lib::raise(FormatError::UNSUPPORTED, std::string{what})};
}

}  // namespace simon::format

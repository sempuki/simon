// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include "format/text.hpp"

#include <algorithm>
#include <cctype>
#include <charconv>
#include <cmath>
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

auto trim(std::string_view text) -> std::string_view {
  auto blank = [](char c) {
    return std::isspace(static_cast<unsigned char>(c)) != 0;
  };
  while (!text.empty() && blank(text.front())) {
    text.remove_prefix(1);
  }
  while (!text.empty() && blank(text.back())) {
    text.remove_suffix(1);
  }
  return text;
}

auto parse_number(std::string_view text) -> std::optional<double> {
  text = trim(text);
  if (!text.empty() && text.front() == '+') {
    text.remove_prefix(1);
  }
  double value = 0.0;
  auto [end, error] =
      std::from_chars(text.data(), text.data() + text.size(), value);
  if (error != std::errc{} || end != text.data() + text.size() ||
      !std::isfinite(value)) {
    return std::nullopt;
  }
  return value;
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

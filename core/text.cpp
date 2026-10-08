// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include "core/text.hpp"

#include <cctype>
#include <charconv>
#include <cmath>
#include <system_error>

namespace simon {

namespace {

// `text` trimmed, without a leading `+`, which from_chars refuses.
auto unsign(std::string_view text) -> std::string_view {
  text = trim(text);
  if (!text.empty() && text.front() == '+') {
    text.remove_prefix(1);
  }
  return text;
}

}  // namespace

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
  text = unsign(text);
  double value = 0.0;
  auto [end, error] =
      std::from_chars(text.data(), text.data() + text.size(), value);
  if (error != std::errc{} || end != text.data() + text.size() ||
      !std::isfinite(value)) {
    return std::nullopt;
  }
  return value;
}

auto parse_integer(std::string_view text) -> std::optional<std::int64_t> {
  text = unsign(text);
  std::int64_t value = 0;
  auto [end, error] =
      std::from_chars(text.data(), text.data() + text.size(), value);
  if (error != std::errc{} || end != text.data() + text.size()) {
    return std::nullopt;
  }
  return value;
}

}  // namespace simon

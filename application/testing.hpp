// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <algorithm>
#include <charconv>
#include <cstddef>
#include <fstream>
#include <string>
#include <string_view>
#include <system_error>
#include <vector>

#include "base/testing.hpp"

// Shared by every application's tests: the reference tables they check
// against, comma-separated with a header line, and the numbers in them.
namespace simon::testing {

// The cells of one line of a table.
inline auto split_cells(std::string_view line) -> std::vector<std::string> {
  std::vector<std::string> cells;
  for (std::size_t at = 0; at <= line.size();) {
    std::size_t comma = std::min(line.find(',', at), line.size());
    cells.emplace_back(line.substr(at, comma - at));
    at = comma + 1;
  }
  return cells;
}

// A table's column names, and every line after them as cells.
struct Table final {
  std::vector<std::string> header;
  std::vector<std::vector<std::string>> lines;
};

// The table at `path`.
inline auto load_table(std::string_view path) -> Table {
  std::ifstream file{std::string{path}};
  REQUIRE(file);
  Table table;
  std::string line;
  std::getline(file, line);
  table.header = split_cells(line);
  while (std::getline(file, line)) {
    table.lines.push_back(split_cells(line));
  }
  return table;
}

// The number `text` holds.
inline auto parse_number(std::string_view text) -> double {
  double value = 0.0;
  auto [end, error] =
      std::from_chars(text.data(), text.data() + text.size(), value);
  REQUIRE(error == std::errc{});
  return value;
}

// The numbers `text` holds, separated by spaces.
inline auto parse_numbers(std::string_view text) -> std::vector<double> {
  std::vector<double> values;
  for (std::size_t at = text.find_first_not_of(' ');
       at != std::string_view::npos; at = text.find_first_not_of(' ', at)) {
    std::size_t end = std::min(text.find(' ', at), text.size());
    values.push_back(parse_number(text.substr(at, end - at)));
    at = end;
  }
  return values;
}

}  // namespace simon::testing

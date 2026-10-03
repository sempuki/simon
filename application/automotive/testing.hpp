// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <algorithm>
#include <charconv>
#include <fstream>
#include <functional>
#include <map>
#include <string>
#include <string_view>
#include <system_error>
#include <vector>

#include "base/testing.hpp"

// Shared by the automotive tests: the reference tables in
// application/automotive/reference, read as rows of text by column name.
namespace simon::automotive::testing {

inline constexpr std::string_view ROADS = "application/automotive/roads/";
inline constexpr std::string_view REFERENCE =
    "application/automotive/reference/";

// One row of a table, as text by column name.
using Row = std::map<std::string, std::string, std::less<>>;

// Every row of the reference table `name`, named by its header.
inline auto load_rows(std::string_view name) -> std::vector<Row> {
  std::ifstream file{std::string{REFERENCE} + std::string{name}};
  REQUIRE(file);
  auto split = [](const std::string& line) {
    std::vector<std::string> cells;
    for (std::size_t at = 0; at <= line.size();) {
      std::size_t comma = std::min(line.find(',', at), line.size());
      cells.emplace_back(line.substr(at, comma - at));
      at = comma + 1;
    }
    return cells;
  };
  std::string line;
  std::getline(file, line);
  std::vector<std::string> names = split(line);
  std::vector<Row> rows;
  while (std::getline(file, line)) {
    std::vector<std::string> cells = split(line);
    REQUIRE(cells.size() == names.size());
    Row row;
    for (std::size_t i = 0; i < names.size(); ++i) {
      row[names[i]] = cells[i];
    }
    rows.push_back(std::move(row));
  }
  return rows;
}

// The number in `row`'s `column`.
inline auto number(const Row& row, std::string_view column) -> double {
  const std::string& text = row.find(column)->second;
  double value = 0.0;
  auto [end, error] =
      std::from_chars(text.data(), text.data() + text.size(), value);
  REQUIRE(error == std::errc{});
  return value;
}

}  // namespace simon::automotive::testing

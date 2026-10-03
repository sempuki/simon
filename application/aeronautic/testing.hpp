// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <algorithm>
#include <charconv>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <functional>
#include <map>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <vector>

#include "base/testing.hpp"
#include "framework/vocabulary.hpp"
#include "model/rigid_body.hpp"
#include "model/units.hpp"

// Shared by the flight tests.
namespace simon::aeronautic::testing {

inline constexpr std::string_view BOEING_737 =
    "application/aeronautic/aircraft/737.aircraft";
inline constexpr std::string_view F16 =
    "application/aeronautic/aircraft/f16.aircraft";

// One row of a recording, as columns by name.
using Row = std::map<std::string, double, std::less<>>;

// Every row of the CSV at `path`, named by its header.
inline auto load_rows(std::string_view path) -> std::vector<Row> {
  std::ifstream file{std::string{path}};
  REQUIRE(file);
  std::string line;
  std::getline(file, line);
  std::vector<std::string> names;
  for (std::size_t at = 0; at <= line.size();) {
    std::size_t comma = std::min(line.find(',', at), line.size());
    names.emplace_back(line.substr(at, comma - at));
    at = comma + 1;
  }

  std::vector<Row> rows;
  while (std::getline(file, line)) {
    Row row;
    const char* next = line.data();
    const char* end = line.data() + line.size();
    for (const std::string& name : names) {
      double value = 0.0;
      auto [stop, error] = std::from_chars(next, end, value);
      REQUIRE(error == std::errc{});
      row[name] = value;
      next = stop + 1;
    }
    rows.push_back(std::move(row));
  }
  return rows;
}

// The columns `x`, `y` and `z` of `row`.
inline auto read_vector(const Row& row, std::string_view x, std::string_view y,
                        std::string_view z) -> model::QuantityVector {
  return model::QuantityVector{row.at(x), row.at(y), row.at(z)};
}

// The body `row` records, in SI units.
inline auto read_body(const Row& row) -> model::RigidBody {
  return model::RigidBody{
      .position = read_vector(row, "x", "y", "z") * model::meter,
      .velocity = read_vector(row, "vx", "vy", "vz") * model::meter_per_second,
      .attitude =
          Quaternion{row.at("qw"), row.at("qx"), row.at("qy"), row.at("qz")},
      .rate = read_vector(row, "p", "q", "r") * model::radian_per_second,
  };
}

// The largest of `worst` and how far `actual` is from `expected`, relative
// to `scale`.
inline auto worse(double worst, double actual, double expected, double scale)
    -> double {
  return std::max(worst, std::abs(actual - expected) / scale);
}

}  // namespace simon::aeronautic::testing

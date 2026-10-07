// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <functional>
#include <map>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "application/testing.hpp"
#include "base/testing.hpp"
#include "core/units.hpp"
#include "core/vocabulary.hpp"
#include "model/rigid_body.hpp"

// Shared by the aeronautic tests.
namespace simon::aeronautic::testing {

inline constexpr std::string_view BOEING_737 = "3rd_party/jsbsim/737.aircraft";
inline constexpr std::string_view F16 = "3rd_party/jsbsim/f16.aircraft";

// One row of a recording, as columns by name.
using Row = std::map<std::string, double, std::less<>>;

// Every row of the CSV at `path`, named by its header.
inline auto load_rows(std::string_view path) -> std::vector<Row> {
  simon::testing::Table table = simon::testing::load_table(path);
  std::vector<Row> rows;
  for (const std::vector<std::string>& cells : table.lines) {
    REQUIRE(cells.size() == table.header.size());
    Row row;
    for (std::size_t i = 0; i < cells.size(); ++i) {
      row[table.header[i]] = simon::testing::parse_number(cells[i]);
    }
    rows.push_back(std::move(row));
  }
  return rows;
}

// The columns `x`, `y` and `z` of `row`.
inline auto read_vector(const Row& row, std::string_view x, std::string_view y,
                        std::string_view z) -> QuantityVector {
  return QuantityVector{row.at(x), row.at(y), row.at(z)};
}

// The body `row` records, in SI units.
inline auto read_body(const Row& row) -> model::RigidBody {
  return model::RigidBody{
      .position = read_vector(row, "x", "y", "z") * meter,
      .velocity = read_vector(row, "vx", "vy", "vz") * meter_per_second,
      .attitude =
          Quaternion{row.at("qw"), row.at("qx"), row.at("qy"), row.at("qz")},
      .rate = read_vector(row, "p", "q", "r") * radian_per_second,
  };
}

// The largest of `worst` and how far `actual` is from `expected`, relative
// to `scale`.
inline auto worse(double worst, double actual, double expected, double scale)
    -> double {
  return std::max(worst, std::abs(actual - expected) / scale);
}

}  // namespace simon::aeronautic::testing

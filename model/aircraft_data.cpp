// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

#include "model/aircraft_data.hpp"

#include <algorithm>
#include <charconv>
#include <fstream>
#include <functional>
#include <sstream>
#include <utility>

template <>
const std::array<lib::StatusConditionEntry,
                 simon::model::AIRCRAFT_DATA_ERROR_COUNT>
    lib::EnumStatusKindConditionMixin<
        simon::model::AircraftDataError,
        simon::model::AIRCRAFT_DATA_ERROR_COUNT>::conditions_ = {
        lib::StatusConditionEntry{"aircraft unreadable"},
        lib::StatusConditionEntry{"aircraft malformed"},
};

namespace simon::model {

namespace {

// The file as lines of words, without comments or blank lines.
struct Line final {
  std::size_t number = 0;
  std::vector<std::string_view> words;
};

auto split(std::string_view text) -> std::vector<Line> {
  std::vector<Line> lines;
  std::size_t number = 0;
  while (!text.empty()) {
    std::size_t end = text.find('\n');
    std::string_view line = text.substr(0, end);
    text = end == std::string_view::npos ? std::string_view{}
                                         : text.substr(end + 1);
    ++number;
    line = line.substr(0, line.find('#'));

    Line words{.number = number};
    std::size_t at = 0;
    while (at < line.size()) {
      at = line.find_first_not_of(" \t\r", at);
      if (at == std::string_view::npos) {
        break;
      }
      std::size_t stop = std::min(line.find_first_of(" \t\r", at), line.size());
      words.words.push_back(line.substr(at, stop - at));
      at = stop;
    }
    if (!words.words.empty()) {
      lines.push_back(std::move(words));
    }
  }
  return lines;
}

using Failure = std::unexpected<lib::Status>;

class Parser final {
 public:
  explicit Parser(std::vector<Line> lines) : lines_{std::move(lines)} {}

  auto parse() -> std::expected<AircraftData, lib::Status> {
    if (done() || line().words.size() != 2 ||
        line().words[0] != "simon-aircraft" || line().words[1] != "1") {
      return fail("expected the header `simon-aircraft 1`");
    }
    ++at_;

    AircraftData data;
    while (!done()) {
      std::string_view key = line().words[0];
      std::expected<void, lib::Status> read;
      if (key == "name") {
        read = text(data.name);
      } else if (key == "metrics") {
        read = metrics(data);
      } else if (key == "aero_reference") {
        read = location(data.aero_reference);
      } else if (key == "empty_mass") {
        read = mass(data.empty_mass);
      } else if (key == "empty_inertia") {
        read = numbers(data.empty_inertia);
      } else if (key == "empty_center_of_mass") {
        read = location(data.empty_center_of_mass);
      } else if (key == "tank") {
        read = tank(data);
      } else if (key == "engine") {
        read = engine(data);
      } else if (key == "term") {
        read = term(data);
      } else {
        return fail("unknown entry `" + std::string{key} + "`");
      }
      if (!read) {
        return Failure{read.error()};
      }
    }
    return data;
  }

 private:
  auto done() const -> bool { return at_ >= lines_.size(); }
  auto line() const -> const Line& { return lines_[at_]; }

  auto fail(const std::string& why) const -> Failure {
    std::size_t number =
        done() ? (lines_.empty() ? 0 : lines_.back().number) : line().number;
    return Failure{lib::raise(AircraftDataError::MALFORMED,
                              "line " + std::to_string(number) + ": " + why)};
  }

  // The words after the key on the current line, if there are `count`.
  auto values(std::size_t count)
      -> std::expected<std::vector<double>, lib::Status> {
    const Line& current = line();
    if (current.words.size() != count + 1) {
      return fail("`" + std::string{current.words[0]} + "` takes " +
                  std::to_string(count) + " values");
    }
    std::vector<double> result;
    for (std::size_t i = 1; i < current.words.size(); ++i) {
      std::expected<double, lib::Status> value = number(current.words[i]);
      if (!value) {
        return Failure{value.error()};
      }
      result.push_back(*value);
    }
    ++at_;
    return result;
  }

  auto number(std::string_view word) const
      -> std::expected<double, lib::Status> {
    double value = 0.0;
    auto [end, error] =
        std::from_chars(word.data(), word.data() + word.size(), value);
    if (error != std::errc{} || end != word.data() + word.size()) {
      return fail("`" + std::string{word} + "` is not a number");
    }
    return value;
  }

  auto text(std::string& out) -> std::expected<void, lib::Status> {
    if (line().words.size() != 2) {
      return fail("`name` takes one word");
    }
    out = std::string{line().words[1]};
    ++at_;
    return {};
  }

  template <std::size_t Count>
  auto numbers(std::array<double, Count>& out)
      -> std::expected<void, lib::Status> {
    auto read = values(Count);
    if (!read) {
      return Failure{read.error()};
    }
    std::ranges::copy(*read, out.begin());
    return {};
  }

  auto location(Displacement& out) -> std::expected<void, lib::Status> {
    std::array<double, 3> xyz{};
    auto read = numbers(xyz);
    if (read) {
      out = meters(xyz[0], xyz[1], xyz[2]);
    }
    return read;
  }

  auto mass(Mass& out) -> std::expected<void, lib::Status> {
    std::array<double, 1> value{};
    auto read = numbers(value);
    if (read) {
      out = value[0] * kilogram;
    }
    return read;
  }

  auto metrics(AircraftData& data) -> std::expected<void, lib::Status> {
    std::array<double, 3> value{};
    auto read = numbers(value);
    if (read) {
      data.wing_area = value[0] * square_meter;
      data.wing_span = value[1] * meter;
      data.chord = value[2] * meter;
    }
    return read;
  }

  auto tank(AircraftData& data) -> std::expected<void, lib::Status> {
    std::array<double, 5> value{};
    auto read = numbers(value);
    if (read) {
      data.tanks.push_back(FuelTank{
          .location = meters(value[0], value[1], value[2]),
          .capacity = value[3] * kilogram,
          .contents = value[4] * kilogram,
      });
    }
    return read;
  }

  auto variable(std::string_view name) const
      -> std::expected<AeroVariable, lib::Status> {
    std::optional<AeroVariable> found = aero_variable_named(name);
    if (!found) {
      return fail("unknown variable `" + std::string{name} + "`");
    }
    return *found;
  }

  static auto increasing(const std::vector<double>& breakpoints) -> bool {
    return !breakpoints.empty() &&
           std::ranges::adjacent_find(breakpoints, std::greater_equal{}) ==
               breakpoints.end();
  }

  // A table block: `table <row> [<column>]`, then for one variable a
  // breakpoint and value per line, or for two a `columns` line and then a
  // row breakpoint and its values per line, then `end`.
  auto table() -> std::expected<AeroTable, lib::Status> {
    const Line& header = line();
    if (header.words.size() < 2 || header.words.size() > 3) {
      return fail("`table` takes one or two variables");
    }
    auto row = variable(header.words[1]);
    if (!row) {
      return Failure{row.error()};
    }
    std::optional<AeroVariable> column;
    if (header.words.size() == 3) {
      auto found = variable(header.words[2]);
      if (!found) {
        return Failure{found.error()};
      }
      column = *found;
    }
    ++at_;

    std::vector<double> rows;
    std::vector<double> columns;
    std::vector<double> values;
    if (column) {
      if (done() || line().words[0] != "columns") {
        return fail("a table of two variables starts with `columns`");
      }
      for (std::size_t i = 1; i < line().words.size(); ++i) {
        auto value = number(line().words[i]);
        if (!value) {
          return Failure{value.error()};
        }
        columns.push_back(*value);
      }
      ++at_;
    }
    while (!done() && line().words[0] != "end") {
      std::size_t width = column ? columns.size() : 1;
      if (line().words.size() != width + 1) {
        return fail("a table row needs a breakpoint and " +
                    std::to_string(width) + " values");
      }
      for (std::size_t i = 0; i < line().words.size(); ++i) {
        auto value = number(line().words[i]);
        if (!value) {
          return Failure{value.error()};
        }
        (i == 0 ? rows : values).push_back(*value);
      }
      ++at_;
    }
    if (done()) {
      return fail("a table needs an `end`");
    }
    if (!increasing(rows) || (column && !increasing(columns))) {
      return fail("table breakpoints must be strictly increasing");
    }
    ++at_;

    if (column) {
      return AeroTable{.row = *row,
                       .column = column,
                       .table = Table2<>{std::move(rows), std::move(columns),
                                         std::move(values)}};
    }
    return AeroTable{.row = *row,
                     .table = Table1<>{std::move(rows), std::move(values)}};
  }

  auto term(AircraftData& data) -> std::expected<void, lib::Status> {
    const Line& header = line();
    if (header.words.size() != 3) {
      return fail("`term` takes an axis and a name");
    }
    std::optional<AeroAxis> axis = aero_axis_named(header.words[1]);
    if (!axis) {
      return fail("unknown axis `" + std::string{header.words[1]} + "`");
    }
    AeroTerm term{.name = std::string{header.words[2]}};
    ++at_;

    while (!done() && line().words[0] != "end") {
      std::string_view key = line().words[0];
      if (key == "constant") {
        auto value = values(1);
        if (!value) {
          return Failure{value.error()};
        }
        term.constant = (*value)[0];
      } else if (key == "factor") {
        if (line().words.size() != 2) {
          return fail("`factor` takes one variable");
        }
        auto factor = variable(line().words[1]);
        if (!factor) {
          return Failure{factor.error()};
        }
        term.factors.push_back(*factor);
        ++at_;
      } else if (key == "table") {
        auto read = table();
        if (!read) {
          return Failure{read.error()};
        }
        term.tables.push_back(std::move(*read));
      } else {
        return fail("unknown term entry `" + std::string{key} + "`");
      }
    }
    if (done()) {
      return fail("a term needs an `end`");
    }
    ++at_;
    data.aero.axes[static_cast<std::size_t>(*axis)].push_back(std::move(term));
    return {};
  }

  auto engine(AircraftData& data) -> std::expected<void, lib::Status> {
    const Line& header = line();
    if (header.words.size() != 3 || header.words[1] != "turbine") {
      return fail("`engine` takes `turbine` and a name");
    }
    TurbineData turbine{.name = std::string{header.words[2]}};
    ++at_;

    while (!done() && line().words[0] != "end") {
      std::string_view key = line().words[0];
      std::expected<void, lib::Status> read;
      auto scalar = [&](double& out) -> std::expected<void, lib::Status> {
        auto value = values(1);
        if (!value) {
          return Failure{value.error()};
        }
        out = (*value)[0];
        return {};
      };
      double thrust = 0.0;
      if (key == "location") {
        read = location(turbine.location);
      } else if (key == "feeds") {
        for (std::size_t i = 1; i < line().words.size(); ++i) {
          auto value = number(line().words[i]);
          if (!value || *value < 0.0) {
            return fail("a feed is a tank's index");
          }
          turbine.feeds.push_back(static_cast<std::size_t>(*value));
        }
        ++at_;
      } else if (key == "military_thrust") {
        read = scalar(thrust);
        turbine.military_thrust = thrust * newton;
      } else if (key == "bypass_ratio") {
        read = scalar(turbine.bypass_ratio);
      } else if (key == "thrust_specific_fuel_consumption") {
        read = scalar(turbine.thrust_specific_fuel_consumption);
      } else if (key == "bleed") {
        read = scalar(turbine.bleed);
      } else if (key == "idle_n1") {
        read = scalar(turbine.idle_n1);
      } else if (key == "idle_n2") {
        read = scalar(turbine.idle_n2);
      } else if (key == "max_n1") {
        read = scalar(turbine.max_n1);
      } else if (key == "max_n2") {
        read = scalar(turbine.max_n2);
      } else if (key == "idle_fuel_flow") {
        read = scalar(turbine.idle_fuel_flow);
      } else if (key == "n1_spool_up") {
        read = scalar(turbine.n1_spool_up);
      } else if (key == "n1_spool_down") {
        read = scalar(turbine.n1_spool_down);
      } else if (key == "n2_spool_up") {
        read = scalar(turbine.n2_spool_up);
      } else if (key == "n2_spool_down") {
        read = scalar(turbine.n2_spool_down);
      } else if (key == "idle_thrust" || key == "military_thrust_factor") {
        ++at_;
        if (done() || line().words[0] != "table") {
          return fail("`" + std::string{key} + "` is followed by a table");
        }
        auto table_read = table();
        if (!table_read) {
          return Failure{table_read.error()};
        }
        (key == "idle_thrust" ? turbine.idle_thrust
                              : turbine.military_thrust_factor) =
            std::move(*table_read);
      } else {
        return fail("unknown engine entry `" + std::string{key} + "`");
      }
      if (!read) {
        return Failure{read.error()};
      }
    }
    if (done()) {
      return fail("an engine needs an `end`");
    }
    ++at_;
    data.engines.push_back(std::move(turbine));
    return {};
  }

  std::vector<Line> lines_;
  std::size_t at_ = 0;
};

}  // namespace

auto parse_aircraft(std::string_view text)
    -> std::expected<AircraftData, lib::Status> {
  return Parser{split(text)}.parse();
}

auto load_aircraft(const std::string& path)
    -> std::expected<AircraftData, lib::Status> {
  std::ifstream file{path};
  if (!file) {
    return std::unexpected(
        lib::raise(AircraftDataError::UNREADABLE, "cannot open " + path));
  }
  std::stringstream text;
  text << file.rdbuf();
  return parse_aircraft(text.str());
}

}  // namespace simon::model

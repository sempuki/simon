// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include "model/aircraft_data.hpp"

#include <algorithm>
#include <charconv>
#include <cmath>
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
        read = read_text(Out(data.name));
      } else if (key == "metrics") {
        read = read_metrics(InOut(data));
      } else if (key == "aero_reference") {
        read = read_location(Out(data.aero_reference));
      } else if (key == "eye_point") {
        read = read_location(Out(data.eye_point));
      } else if (key == "point_mass") {
        read = append_point_mass(InOut(data));
      } else if (key == "empty_mass") {
        read = read_mass(Out(data.empty_mass));
      } else if (key == "empty_inertia") {
        read = read_numbers(Out(data.empty_inertia));
      } else if (key == "empty_center_of_mass") {
        read = read_location(Out(data.empty_center_of_mass));
      } else if (key == "tank") {
        read = append_tank(InOut(data));
      } else if (key == "engine") {
        read = append_engine(InOut(data));
      } else if (key == "term") {
        read = append_term(InOut(data));
      } else if (key == "flight_controls") {
        read = read_flight_controls(InOut(data));
      } else {
        return fail("unknown entry `" + std::string{key} + "`");
      }
      if (!read) {
        return Failure{read.error()};
      }
    }
    if (data.engines.size() > MAX_ENGINES || data.tanks.size() > MAX_TANKS) {
      return fail("an aircraft has at most " + std::to_string(MAX_ENGINES) +
                  " engines and " + std::to_string(MAX_TANKS) + " tanks");
    }
    append_throttles(InOut(data.flight_controls), data.engines.size());
    data.aero.forces_read_alpha_rate =
        data.aero.reads(AeroAxis::LIFT, AeroVariable::ALPHA_RATE) ||
        data.aero.reads(AeroAxis::DRAG, AeroVariable::ALPHA_RATE) ||
        data.aero.reads(AeroAxis::SIDE, AeroVariable::ALPHA_RATE);
    if (data.flight_controls.signals.size() > MAX_FLIGHT_SIGNALS) {
      return fail("the flight controls have more than " +
                  std::to_string(MAX_FLIGHT_SIGNALS) + " signals");
    }
    for (const TurbineData& turbine : data.engines) {
      if (!turbine.idle_thrust || !turbine.military_thrust_factor) {
        return fail("engine `" + turbine.name + "` needs both thrust tables");
      }
      if (turbine.has_reheat() && turbine.max_thrust <= 0.0 * newton) {
        return fail("engine `" + turbine.name +
                    "` has reheat and needs its maximum thrust");
      }
      for (std::size_t feed : turbine.feeds) {
        if (feed >= data.tanks.size()) {
          return fail("engine `" + turbine.name + "` feeds from tank " +
                      std::to_string(feed) + ", which is not there");
        }
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
    if (!std::isfinite(value)) {
      return fail("`" + std::string{word} + "` is not a finite number");
    }
    return value;
  }

  // The words after the key, joined by single spaces.
  auto read_text(Out<std::string> out) -> std::expected<void, lib::Status> {
    const std::vector<std::string_view>& words = line().words;
    if (words.size() < 2) {
      return fail("`" + std::string{words[0]} + "` takes some words");
    }
    out->clear();
    for (std::size_t i = 1; i < words.size(); ++i) {
      if (i > 1) {
        *out += ' ';
      }
      *out += words[i];
    }
    ++at_;
    return {};
  }

  template <std::size_t Count>
  auto read_numbers(Out<std::array<double, Count>> out)
      -> std::expected<void, lib::Status> {
    auto read = values(Count);
    if (!read) {
      return Failure{read.error()};
    }
    std::ranges::copy(*read, out->begin());
    return {};
  }

  auto read_location(Out<Displacement> out)
      -> std::expected<void, lib::Status> {
    std::array<double, 3> xyz{};
    auto read = read_numbers(Out(xyz));
    if (read) {
      *out = meters(xyz[0], xyz[1], xyz[2]);
    }
    return read;
  }

  auto read_mass(Out<Mass> out) -> std::expected<void, lib::Status> {
    std::array<double, 1> value{};
    auto read = read_numbers(Out(value));
    if (read) {
      *out = value[0] * kilogram;
    }
    return read;
  }

  auto read_metrics(InOut<AircraftData> data)
      -> std::expected<void, lib::Status> {
    std::array<double, 3> value{};
    auto read = read_numbers(Out(value));
    if (read) {
      data->wing_area = value[0] * square_meter;
      data->wing_span = value[1] * meter;
      data->chord = value[2] * meter;
    }
    return read;
  }

  auto append_point_mass(InOut<AircraftData> data)
      -> std::expected<void, lib::Status> {
    std::array<double, 4> value{};
    auto read = read_numbers(Out(value));
    if (read) {
      data->point_masses.push_back(PointMass{
          .mass = value[0] * kilogram,
          .location = meters(value[1], value[2], value[3]),
      });
    }
    return read;
  }

  auto append_tank(InOut<AircraftData> data)
      -> std::expected<void, lib::Status> {
    std::array<double, 5> value{};
    auto read = read_numbers(Out(value));
    if (read) {
      data->tanks.push_back(FuelTank{
          .location = meters(value[0], value[1], value[2]),
          .capacity = value[3] * kilogram,
          .contents = value[4] * kilogram,
      });
    }
    return read;
  }

  // An input by name: a variable, else a flight control signal, which the
  // flight controls must already have named, or `|signal|` for its magnitude.
  // The aerodynamics read each signal from their own slot.
  auto input(std::string_view name, InOut<AircraftData> data) const
      -> std::expected<AeroInput, lib::Status> {
    if (std::optional<AeroVariable> found = find_aero_variable(name)) {
      return aero_input(*found);
    }
    bool magnitude =
        name.size() > 2 && name.starts_with('|') && name.ends_with('|');
    if (magnitude) {
      name = name.substr(1, name.size() - 2);
    }
    std::optional<std::size_t> signal =
        find_signal(data->flight_controls, name);
    if (!signal) {
      return fail("unknown input `" + std::string{name} + "`");
    }
    std::vector<AeroSignal>& read = data->aero.signals;
    AeroSignal wanted{.signal = *signal, .magnitude = magnitude};
    auto slot = std::ranges::find(read, wanted);
    if (slot == read.end()) {
      if (read.size() == MAX_AERO_SIGNALS) {
        return fail("the aerodynamics read more than " +
                    std::to_string(MAX_AERO_SIGNALS) + " signals");
      }
      read.push_back(wanted);
      slot = read.end() - 1;
    }
    return AeroInput{.index = static_cast<std::uint8_t>(AERO_VARIABLE_COUNT +
                                                        (slot - read.begin()))};
  }

  static auto increasing(const std::vector<double>& breakpoints) -> bool {
    return !breakpoints.empty() &&
           std::ranges::adjacent_find(breakpoints, std::greater_equal{}) ==
               breakpoints.end();
  }

  // A table block: `table <row> [<column>]`, then for one variable a
  // breakpoint and value per line, or for two a `columns` line and then a
  // row breakpoint and its values per line, then `end`.
  auto table(InOut<AircraftData> data)
      -> std::expected<AeroTable, lib::Status> {
    const Line& header = line();
    if (header.words.size() < 2 || header.words.size() > 3) {
      return fail("`table` takes one or two inputs");
    }
    auto row = input(header.words[1], data);
    if (!row) {
      return Failure{row.error()};
    }
    std::optional<AeroInput> column;
    if (header.words.size() == 3) {
      auto found = input(header.words[2], data);
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

  auto append_term(InOut<AircraftData> data)
      -> std::expected<void, lib::Status> {
    const Line& header = line();
    if (header.words.size() != 3) {
      return fail("`term` takes an axis and a name");
    }
    std::optional<AeroAxis> axis = find_aero_axis(header.words[1]);
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
          return fail("`factor` takes one input");
        }
        auto factor = input(line().words[1], data);
        if (!factor) {
          return Failure{factor.error()};
        }
        term.factors.push_back(*factor);
        ++at_;
      } else if (key == "table") {
        auto read = table(data);
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
    data->aero.axes[static_cast<std::size_t>(*axis)].push_back(std::move(term));
    return {};
  }

  auto append_engine(InOut<AircraftData> data)
      -> std::expected<void, lib::Status> {
    const Line& header = line();
    if (header.words.size() != 3 || header.words[1] != "turbine") {
      return fail("`engine` takes `turbine` and a name");
    }
    TurbineData turbine{.name = std::string{header.words[2]}};
    ++at_;

    while (!done() && line().words[0] != "end") {
      std::string_view key = line().words[0];
      std::expected<void, lib::Status> read;
      auto scalar = [&](Out<double> out) -> std::expected<void, lib::Status> {
        auto value = values(1);
        if (!value) {
          return Failure{value.error()};
        }
        *out = (*value)[0];
        return {};
      };
      double thrust = 0.0;
      if (key == "location") {
        read = read_location(Out(turbine.location));
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
        read = scalar(Out(thrust));
        turbine.military_thrust = thrust * newton;
      } else if (key == "bypass_ratio") {
        read = scalar(Out(turbine.bypass_ratio));
      } else if (key == "thrust_specific_fuel_consumption") {
        read = scalar(Out(turbine.thrust_specific_fuel_consumption));
      } else if (key == "bleed") {
        read = scalar(Out(turbine.bleed));
      } else if (key == "max_thrust") {
        read = scalar(Out(thrust));
        turbine.max_thrust = thrust * newton;
      } else if (key == "reheat_thrust_specific_fuel_consumption") {
        read = scalar(Out(turbine.reheat_thrust_specific_fuel_consumption));
      } else if (key == "idle_n1") {
        read = scalar(Out(turbine.idle_n1));
      } else if (key == "idle_n2") {
        read = scalar(Out(turbine.idle_n2));
      } else if (key == "max_n1") {
        read = scalar(Out(turbine.max_n1));
      } else if (key == "max_n2") {
        read = scalar(Out(turbine.max_n2));
      } else if (key == "idle_fuel_flow") {
        read = scalar(Out(turbine.idle_fuel_flow));
      } else if (key == "n1_spool_up") {
        read = scalar(Out(turbine.n1_spool_up));
      } else if (key == "n1_spool_down") {
        read = scalar(Out(turbine.n1_spool_down));
      } else if (key == "n2_spool_up") {
        read = scalar(Out(turbine.n2_spool_up));
      } else if (key == "n2_spool_down") {
        read = scalar(Out(turbine.n2_spool_down));
      } else if (key == "idle_thrust" || key == "military_thrust_factor" ||
                 key == "max_thrust_factor") {
        ++at_;
        if (done() || line().words[0] != "table") {
          return fail("`" + std::string{key} + "` is followed by a table");
        }
        auto table_read = table(data);
        if (!table_read) {
          return Failure{table_read.error()};
        }
        (key == "idle_thrust"              ? turbine.idle_thrust
         : key == "military_thrust_factor" ? turbine.military_thrust_factor
                                           : turbine.max_thrust_factor) =
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
    data->engines.push_back(std::move(turbine));
    return {};
  }

  // An input as written: a signal's name, whether it is negated, and the
  // scale that converts it.
  struct NamedInput final {
    std::string name;
    bool negated = false;
    double scale = 1.0;
  };

  struct NamedOperand final {
    double value = 0.0;
    std::optional<NamedInput> input;
  };

  struct NamedCondition final {
    std::string signal;
    FlightBlock::Condition::Comparison comparison =
        FlightBlock::Condition::Comparison::EQ;
    NamedOperand right;
  };

  struct NamedTest final {
    bool any = false;
    NamedOperand value;
    std::vector<NamedCondition> conditions;
  };

  // A block as written, its signals still by name.
  struct NamedBlock final {
    FlightBlock block;
    std::vector<NamedInput> inputs;
    std::optional<NamedInput> output;
    std::string schedule_signal;
    NamedOperand fallback;
    std::vector<NamedTest> tests;
    std::optional<NamedInput> trigger;
    std::vector<std::optional<NamedInput>> pushes;  // By operation.
    std::size_t line = 0;
  };

  // `flight_controls`, then signals and blocks, then `end`. A signal is
  // `signal <name>`: one nothing writes, which holds zero. A block is
  // `block <kind> <name>`, its entries, then `end`.
  auto read_flight_controls(InOut<AircraftData> data)
      -> std::expected<void, lib::Status> {
    ++at_;
    std::vector<std::string> declared;
    std::vector<NamedBlock> named;
    while (!done() && line().words[0] != "end") {
      if (line().words[0] == "signal") {
        if (line().words.size() != 2) {
          return fail("`signal` takes a name");
        }
        declared.emplace_back(line().words[1]);
        ++at_;
        continue;
      }
      auto read = block();
      if (!read) {
        return Failure{read.error()};
      }
      named.push_back(std::move(*read));
    }
    if (done()) {
      return fail("`flight_controls` needs an `end`");
    }
    ++at_;

    // Every signal: the fixed ones, the declared ones, each block's own and
    // outputs, then each PID's state.
    FlightControlData& controls = data->flight_controls;
    controls.signals.clear();
    for (std::size_t i = 0; i < FLIGHT_SIGNAL_COUNT; ++i) {
      controls.signals.emplace_back(
          flight_signal_name(static_cast<FlightSignal>(i)));
    }
    auto find_or_append = [&](const std::string& name) -> std::size_t {
      if (std::optional<std::size_t> found = find_signal(controls, name)) {
        return *found;
      }
      controls.signals.push_back(name);
      return controls.signals.size() - 1;
    };
    for (const std::string& name : declared) {
      find_or_append(name);
    }
    for (NamedBlock& each : named) {
      each.block.signal = find_or_append(each.block.name);
      if (each.output) {
        each.block.output = find_or_append(each.output->name);
        each.block.output_scale = each.output->scale;
      }
    }
    for (NamedBlock& each : named) {
      if (each.block.kind == FlightBlock::Kind::PID) {
        each.block.state = controls.signals.size();
        for (std::string_view part : {"/integral", "/previous", "/before"}) {
          controls.signals.push_back(each.block.name + std::string{part});
        }
      }
    }
    if (controls.signals.size() > MAX_FLIGHT_SIGNALS) {
      return fail("the flight controls have more than " +
                  std::to_string(MAX_FLIGHT_SIGNALS) + " signals");
    }

    // Then what each block reads.
    for (NamedBlock& each : named) {
      auto resolved = resolve(InOut(controls), InOut(each));
      if (!resolved) {
        return Failure{resolved.error()};
      }
      controls.blocks.push_back(std::move(each.block));
    }
    return {};
  }

  // Appends each of `engines` engines' throttle to `controls`, and the fixed
  // signals first if there are no flight controls.
  static auto append_throttles(InOut<FlightControlData> controls,
                               std::size_t engines) -> void {
    if (controls->signals.empty()) {
      for (std::size_t i = 0; i < FLIGHT_SIGNAL_COUNT; ++i) {
        controls->signals.emplace_back(
            flight_signal_name(static_cast<FlightSignal>(i)));
      }
    }
    for (std::size_t i = 0; i < engines; ++i) {
      std::string name = "throttle_" + std::to_string(i);
      std::optional<std::size_t> found = find_signal(*controls, name);
      if (!found) {
        controls->signals.push_back(name);
        found = controls->signals.size() - 1;
      }
      controls->throttles.push_back(*found);
    }
  }

  // Finds each signal `each` reads by name, and marks the fixed ones read.
  static auto resolve(InOut<FlightControlData> controls, InOut<NamedBlock> each)
      -> std::expected<void, lib::Status> {
    std::string missing;
    auto signal = [&](const std::string& name) -> std::size_t {
      std::optional<std::size_t> found = find_signal(*controls, name);
      if (!found) {
        missing = name;
        return 0;
      }
      if (*found < FLIGHT_SIGNAL_COUNT) {
        controls->read.set(*found);
      }
      return *found;
    };
    auto input = [&](const NamedInput& named) -> FlightBlock::Input {
      return FlightBlock::Input{.signal = signal(named.name),
                                .negated = named.negated,
                                .scale = named.scale};
    };
    auto operand = [&](const NamedOperand& named) -> FlightBlock::Operand {
      FlightBlock::Operand result{.value = named.value};
      if (named.input) {
        result.input = input(*named.input);
      }
      return result;
    };

    FlightBlock& block = each->block;
    for (const NamedInput& named : each->inputs) {
      block.inputs.push_back(input(named));
    }
    if (!each->schedule_signal.empty()) {
      block.schedule_signal = signal(each->schedule_signal);
    }
    block.fallback = operand(each->fallback);
    for (const NamedTest& named : each->tests) {
      FlightBlock::Test test{.any = named.any, .value = operand(named.value)};
      for (const NamedCondition& condition : named.conditions) {
        test.conditions.push_back(
            FlightBlock::Condition{.signal = signal(condition.signal),
                                   .comparison = condition.comparison,
                                   .right = operand(condition.right)});
      }
      block.tests.push_back(std::move(test));
    }
    if (each->trigger) {
      block.trigger = input(*each->trigger);
    }
    for (std::size_t i = 0; i < block.operations.size(); ++i) {
      if (each->pushes[i]) {
        block.operations[i].input = input(*each->pushes[i]);
      }
    }
    if (!missing.empty()) {
      return Failure{lib::raise(AircraftDataError::MALFORMED,
                                "line " + std::to_string(each->line) +
                                    ": no signal `" + missing + "`")};
    }
    return {};
  }

  // `<name> [<scale>]` from word `at` of the current line, the name maybe
  // negated.
  auto named_input(std::size_t at) const
      -> std::expected<NamedInput, lib::Status> {
    const std::vector<std::string_view>& words = line().words;
    if (words.size() != at + 1 && words.size() != at + 2) {
      return fail("`" + std::string{words[0]} +
                  "` takes a signal and maybe a scale");
    }
    std::string_view name = words[at];
    bool negated = name.starts_with('-');
    NamedInput named{.name = std::string{negated ? name.substr(1) : name},
                     .negated = negated};
    if (words.size() == at + 2) {
      auto scale = number(words[at + 1]);
      if (!scale) {
        return Failure{scale.error()};
      }
      named.scale = *scale;
    }
    return named;
  }

  // A number, or a signal's name, maybe negated.
  auto operand(std::string_view word) const -> NamedOperand {
    double value = 0.0;
    auto [end, error] =
        std::from_chars(word.data(), word.data() + word.size(), value);
    if (error == std::errc{} && end == word.data() + word.size()) {
      return NamedOperand{.value = value};
    }
    bool negated = word.starts_with('-');
    return NamedOperand{
        .input =
            NamedInput{.name = std::string{negated ? word.substr(1) : word},
                       .negated = negated}};
  }

  // `test <and|or> <value>`, then `condition <signal> <comparison> <operand>`
  // lines, then `end`.
  auto test() -> std::expected<NamedTest, lib::Status> {
    const std::vector<std::string_view>& words = line().words;
    if (words.size() != 3 || (words[1] != "and" && words[1] != "or")) {
      return fail("expected `test <and|or> <value>`");
    }
    NamedTest named{.any = words[1] == "or", .value = operand(words[2])};
    ++at_;
    while (!done() && line().words[0] != "end") {
      const std::vector<std::string_view>& condition = line().words;
      if (condition.size() != 4 || condition[0] != "condition") {
        return fail("expected `condition <signal> <comparison> <operand>`");
      }
      using enum FlightBlock::Condition::Comparison;
      constexpr std::array<
          std::pair<std::string_view, FlightBlock::Condition::Comparison>, 6>
          COMPARISONS{{{"lt", LT},
                       {"le", LE},
                       {"gt", GT},
                       {"ge", GE},
                       {"eq", EQ},
                       {"ne", NE}}};
      auto found = std::ranges::find(
          COMPARISONS, condition[2],
          &std::pair<std::string_view,
                     FlightBlock::Condition::Comparison>::first);
      if (found == COMPARISONS.end()) {
        return fail("unknown comparison `" + std::string{condition[2]} + "`");
      }
      named.conditions.push_back(NamedCondition{
          .signal = std::string{condition[1]},
          .comparison = found->second,
          .right = operand(condition[3]),
      });
      ++at_;
    }
    if (done()) {
      return fail("a test needs an `end`");
    }
    ++at_;
    return named;
  }

  auto block() -> std::expected<NamedBlock, lib::Status> {
    const Line& header = line();
    if (header.words.size() != 3 || header.words[0] != "block") {
      return fail("expected `block <kind> <name>`");
    }
    NamedBlock named{.line = header.number};
    FlightBlock& block = named.block;
    using enum FlightBlock::Kind;
    constexpr std::array<std::pair<std::string_view, FlightBlock::Kind>, 8>
        KINDS{{{"summer", SUMMER},
               {"pure_gain", GAIN},
               {"scheduled_gain", SCHEDULED_GAIN},
               {"surface_scale", SURFACE_SCALE},
               {"kinematic", KINEMATIC},
               {"switch", SWITCH},
               {"pid", PID},
               {"function", FUNCTION}}};
    auto kind = std::ranges::find(
        KINDS, header.words[1],
        &std::pair<std::string_view, FlightBlock::Kind>::first);
    if (kind == KINDS.end()) {
      return fail("unknown block kind `" + std::string{header.words[1]} + "`");
    }
    block.kind = kind->second;
    block.name = std::string{header.words[2]};
    ++at_;

    using Operation = FlightBlock::Operation;
    while (!done() && line().words[0] != "end") {
      std::string_view key = line().words[0];
      const std::vector<std::string_view>& words = line().words;
      if (key == "input" || key == "output" || key == "trigger" ||
          key == "push") {
        auto read = named_input(1);
        if (!read) {
          return Failure{read.error()};
        }
        if (key == "input") {
          named.inputs.push_back(std::move(*read));
        } else if (key == "output") {
          if (read->negated) {
            return fail("an output cannot be negated");
          }
          named.output = std::move(*read);
        } else if (key == "trigger") {
          named.trigger = std::move(*read);
        } else {
          block.operations.push_back(Operation{.kind = Operation::Kind::PUSH});
          named.pushes.push_back(std::move(*read));
        }
        ++at_;
        continue;
      }
      if (key == "table") {
        if (words.size() != 2) {
          return fail("a schedule's `table` takes one signal");
        }
        named.schedule_signal = std::string{words[1]};
        ++at_;
        std::vector<double> xs;
        std::vector<double> ys;
        while (!done() && line().words[0] != "end") {
          auto row = values(1);
          if (!row) {
            return Failure{row.error()};
          }
          xs.push_back(*number(lines_[at_ - 1].words[0]));
          ys.push_back((*row)[0]);
        }
        if (done()) {
          return fail("a table needs an `end`");
        }
        if (!increasing(xs)) {
          return fail("table breakpoints must be strictly increasing");
        }
        ++at_;
        block.schedule = Table1<>{std::move(xs), std::move(ys)};
        continue;
      }
      if (key == "default") {
        if (words.size() != 2) {
          return fail("`default` takes a value");
        }
        named.fallback = operand(words[1]);
        ++at_;
        continue;
      }
      if (key == "test") {
        auto read = test();
        if (!read) {
          return Failure{read.error()};
        }
        named.tests.push_back(std::move(*read));
        continue;
      }
      if (key == "integrator") {
        using enum FlightBlock::Integrator;
        constexpr std::array<
            std::pair<std::string_view, FlightBlock::Integrator>, 5>
            INTEGRATORS{{{"none", NONE},
                         {"rect", RECTANGULAR},
                         {"trap", TRAPEZOIDAL},
                         {"ab2", ADAMS_BASHFORTH_2},
                         {"ab3", ADAMS_BASHFORTH_3}}};
        auto found =
            words.size() == 2
                ? std::ranges::find(INTEGRATORS, words[1],
                                    &std::pair<std::string_view,
                                               FlightBlock::Integrator>::first)
                : INTEGRATORS.end();
        if (found == INTEGRATORS.end()) {
          return fail("unknown integrator");
        }
        block.integrator = found->second;
        ++at_;
        continue;
      }
      // A function's operations that take nothing more.
      constexpr std::array<std::pair<std::string_view, Operation::Kind>, 6>
          OPERATIONS{{{"difference", Operation::Kind::DIFFERENCE},
                      {"quotient", Operation::Kind::QUOTIENT},
                      {"sin", Operation::Kind::SIN},
                      {"cos", Operation::Kind::COS},
                      {"tan", Operation::Kind::TAN},
                      {"abs", Operation::Kind::ABS}}};
      if (auto operation = std::ranges::find(
              OPERATIONS, key,
              &std::pair<std::string_view, Operation::Kind>::first);
          operation != OPERATIONS.end()) {
        if (words.size() != 1) {
          return fail("`" + std::string{key} + "` takes nothing");
        }
        block.operations.push_back(Operation{.kind = operation->second});
        named.pushes.emplace_back();
        ++at_;
        continue;
      }

      std::size_t count =
          key == "clip" || key == "domain" || key == "range" || key == "setting"
              ? 2
              : 1;
      auto read = values(count);
      if (!read) {
        return Failure{read.error()};
      }
      const std::vector<double>& v = *read;
      if (key == "clip") {
        block.clip = std::pair{v[0], v[1]};
      } else if (key == "domain") {
        block.domain = {v[0], v[1]};
      } else if (key == "range") {
        block.range = {v[0], v[1]};
      } else if (key == "setting") {
        block.detents.push_back(v[0]);
        block.times.push_back(v[1]);
      } else if (key == "bias") {
        block.bias = v[0];
      } else if (key == "gain") {
        block.gain = v[0];
      } else if (key == "zero_centered") {
        block.zero_centered = v[0] != 0.0;
      } else if (key == "scale") {
        block.scale = v[0] != 0.0;
      } else if (key == "kp") {
        block.kp = v[0];
      } else if (key == "ki") {
        block.ki = v[0];
      } else if (key == "kd") {
        block.kd = v[0];
      } else if (key == "constant") {
        block.operations.push_back(
            Operation{.kind = Operation::Kind::CONSTANT, .value = v[0]});
        named.pushes.emplace_back();
      } else if (key == "sum" || key == "product") {
        if (v[0] < 1.0) {
          --at_;
          return fail("`" + std::string{key} + "` takes a count");
        }
        block.operations.push_back(
            Operation{.kind = key == "sum" ? Operation::Kind::SUM
                                           : Operation::Kind::PRODUCT,
                      .count = static_cast<std::size_t>(v[0])});
        named.pushes.emplace_back();
      } else {
        --at_;
        return fail("unknown block entry `" + std::string{key} + "`");
      }
    }
    if (done()) {
      return fail("a block needs an `end`");
    }
    ++at_;
    if (block.kind == SCHEDULED_GAIN && !block.schedule) {
      return fail("a scheduled gain needs a table");
    }
    if (block.kind == KINEMATIC &&
        (block.detents.size() < 2 || !increasing(block.detents))) {
      return fail("a kinematic block needs two or more increasing settings");
    }
    if (block.kind == PID && named.inputs.size() != 1) {
      return fail("a PID needs one input");
    }
    if (block.kind == FUNCTION && !balanced(block.operations)) {
      return fail("a function's operations must leave one value");
    }
    return named;
  }

  // Whether `operations` always have what they take, never hold more than a
  // function's stack does, and leave one value.
  static auto balanced(const std::vector<FlightBlock::Operation>& operations)
      -> bool {
    std::size_t size = 0;
    for (const FlightBlock::Operation& operation : operations) {
      using enum FlightBlock::Operation::Kind;
      std::size_t takes = 0;
      switch (operation.kind) {
        case PUSH:
        case CONSTANT:
          takes = 0;
          break;
        case SUM:
        case PRODUCT:
          takes = operation.count;
          break;
        case DIFFERENCE:
        case QUOTIENT:
          takes = 2;
          break;
        case SIN:
        case COS:
        case TAN:
        case ABS:
          takes = 1;
          break;
      }
      if (size < takes) {
        return false;
      }
      size = size - takes + 1;
      if (size > FUNCTION_STACK_SIZE) {
        return false;
      }
    }
    return size == 1;
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

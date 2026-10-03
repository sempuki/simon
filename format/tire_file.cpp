// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include "format/tire_file.hpp"

#include <algorithm>
#include <cctype>
#include <charconv>
#include <fstream>
#include <sstream>
#include <utility>

template <>
const std::array<lib::StatusConditionEntry,
                 simon::format::TIRE_FILE_ERROR_COUNT>
    lib::EnumStatusKindConditionMixin<
        simon::format::TireFileError,
        simon::format::TIRE_FILE_ERROR_COUNT>::conditions_ = {
        lib::StatusConditionEntry{"tire file unreadable"},
        lib::StatusConditionEntry{"tire file malformed"},
};

namespace simon::format {

using namespace model;

namespace {

using Failure = std::unexpected<lib::Status>;

// Each coefficient by the name a property file gives it.
struct Field final {
  std::string_view name;
  double MagicFormulaTire::* member = nullptr;
};

constexpr std::array FIELDS{
    Field{"UNLOADED_RADIUS", &MagicFormulaTire::unloaded_radius},
    Field{"FNOMIN", &MagicFormulaTire::fnomin},
    Field{"LFZO", &MagicFormulaTire::lfzo},
    Field{"LCX", &MagicFormulaTire::lcx},
    Field{"LMUX", &MagicFormulaTire::lmux},
    Field{"LEX", &MagicFormulaTire::lex},
    Field{"LKX", &MagicFormulaTire::lkx},
    Field{"LHX", &MagicFormulaTire::lhx},
    Field{"LVX", &MagicFormulaTire::lvx},
    Field{"LGAX", &MagicFormulaTire::lgax},
    Field{"LCY", &MagicFormulaTire::lcy},
    Field{"LMUY", &MagicFormulaTire::lmuy},
    Field{"LEY", &MagicFormulaTire::ley},
    Field{"LKY", &MagicFormulaTire::lky},
    Field{"LHY", &MagicFormulaTire::lhy},
    Field{"LVY", &MagicFormulaTire::lvy},
    Field{"LGAY", &MagicFormulaTire::lgay},
    Field{"LTR", &MagicFormulaTire::ltr},
    Field{"LRES", &MagicFormulaTire::lres},
    Field{"LGAZ", &MagicFormulaTire::lgaz},
    Field{"LXAL", &MagicFormulaTire::lxal},
    Field{"LYKA", &MagicFormulaTire::lyka},
    Field{"LVYKA", &MagicFormulaTire::lvyka},
    Field{"LS", &MagicFormulaTire::ls},
    Field{"PCX1", &MagicFormulaTire::pcx1},
    Field{"PDX1", &MagicFormulaTire::pdx1},
    Field{"PDX2", &MagicFormulaTire::pdx2},
    Field{"PDX3", &MagicFormulaTire::pdx3},
    Field{"PEX1", &MagicFormulaTire::pex1},
    Field{"PEX2", &MagicFormulaTire::pex2},
    Field{"PEX3", &MagicFormulaTire::pex3},
    Field{"PEX4", &MagicFormulaTire::pex4},
    Field{"PKX1", &MagicFormulaTire::pkx1},
    Field{"PKX2", &MagicFormulaTire::pkx2},
    Field{"PKX3", &MagicFormulaTire::pkx3},
    Field{"PHX1", &MagicFormulaTire::phx1},
    Field{"PHX2", &MagicFormulaTire::phx2},
    Field{"PVX1", &MagicFormulaTire::pvx1},
    Field{"PVX2", &MagicFormulaTire::pvx2},
    Field{"RBX1", &MagicFormulaTire::rbx1},
    Field{"RBX2", &MagicFormulaTire::rbx2},
    Field{"RCX1", &MagicFormulaTire::rcx1},
    Field{"REX1", &MagicFormulaTire::rex1},
    Field{"REX2", &MagicFormulaTire::rex2},
    Field{"RHX1", &MagicFormulaTire::rhx1},
    Field{"PCY1", &MagicFormulaTire::pcy1},
    Field{"PDY1", &MagicFormulaTire::pdy1},
    Field{"PDY2", &MagicFormulaTire::pdy2},
    Field{"PDY3", &MagicFormulaTire::pdy3},
    Field{"PEY1", &MagicFormulaTire::pey1},
    Field{"PEY2", &MagicFormulaTire::pey2},
    Field{"PEY3", &MagicFormulaTire::pey3},
    Field{"PEY4", &MagicFormulaTire::pey4},
    Field{"PKY1", &MagicFormulaTire::pky1},
    Field{"PKY2", &MagicFormulaTire::pky2},
    Field{"PKY3", &MagicFormulaTire::pky3},
    Field{"PHY1", &MagicFormulaTire::phy1},
    Field{"PHY2", &MagicFormulaTire::phy2},
    Field{"PHY3", &MagicFormulaTire::phy3},
    Field{"PVY1", &MagicFormulaTire::pvy1},
    Field{"PVY2", &MagicFormulaTire::pvy2},
    Field{"PVY3", &MagicFormulaTire::pvy3},
    Field{"PVY4", &MagicFormulaTire::pvy4},
    Field{"RBY1", &MagicFormulaTire::rby1},
    Field{"RBY2", &MagicFormulaTire::rby2},
    Field{"RBY3", &MagicFormulaTire::rby3},
    Field{"RCY1", &MagicFormulaTire::rcy1},
    Field{"REY1", &MagicFormulaTire::rey1},
    Field{"REY2", &MagicFormulaTire::rey2},
    Field{"RHY1", &MagicFormulaTire::rhy1},
    Field{"RHY2", &MagicFormulaTire::rhy2},
    Field{"RVY1", &MagicFormulaTire::rvy1},
    Field{"RVY2", &MagicFormulaTire::rvy2},
    Field{"RVY3", &MagicFormulaTire::rvy3},
    Field{"RVY4", &MagicFormulaTire::rvy4},
    Field{"RVY5", &MagicFormulaTire::rvy5},
    Field{"RVY6", &MagicFormulaTire::rvy6},
    Field{"QBZ1", &MagicFormulaTire::qbz1},
    Field{"QBZ2", &MagicFormulaTire::qbz2},
    Field{"QBZ3", &MagicFormulaTire::qbz3},
    Field{"QBZ4", &MagicFormulaTire::qbz4},
    Field{"QBZ5", &MagicFormulaTire::qbz5},
    Field{"QBZ9", &MagicFormulaTire::qbz9},
    Field{"QBZ10", &MagicFormulaTire::qbz10},
    Field{"QCZ1", &MagicFormulaTire::qcz1},
    Field{"QDZ1", &MagicFormulaTire::qdz1},
    Field{"QDZ2", &MagicFormulaTire::qdz2},
    Field{"QDZ3", &MagicFormulaTire::qdz3},
    Field{"QDZ4", &MagicFormulaTire::qdz4},
    Field{"QDZ6", &MagicFormulaTire::qdz6},
    Field{"QDZ7", &MagicFormulaTire::qdz7},
    Field{"QDZ8", &MagicFormulaTire::qdz8},
    Field{"QDZ9", &MagicFormulaTire::qdz9},
    Field{"QEZ1", &MagicFormulaTire::qez1},
    Field{"QEZ2", &MagicFormulaTire::qez2},
    Field{"QEZ3", &MagicFormulaTire::qez3},
    Field{"QEZ4", &MagicFormulaTire::qez4},
    Field{"QEZ5", &MagicFormulaTire::qez5},
    Field{"QHZ1", &MagicFormulaTire::qhz1},
    Field{"QHZ2", &MagicFormulaTire::qhz2},
    Field{"QHZ3", &MagicFormulaTire::qhz3},
    Field{"QHZ4", &MagicFormulaTire::qhz4},
    Field{"SSZ1", &MagicFormulaTire::ssz1},
    Field{"SSZ2", &MagicFormulaTire::ssz2},
    Field{"SSZ3", &MagicFormulaTire::ssz3},
    Field{"SSZ4", &MagicFormulaTire::ssz4},
};

// The formats whose steady state is the Magic Formula 5.2.
constexpr std::array FORMATS{std::string_view{"PAC2002"},
                             std::string_view{"MF_05"}};

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

auto upper(std::string_view text) -> std::string {
  std::string result{text};
  std::ranges::transform(result, result.begin(), [](char c) {
    return static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
  });
  return result;
}

auto fail(std::size_t line, std::string_view why) -> Failure {
  return Failure{
      lib::raise(TireFileError::MALFORMED,
                 "line " + std::to_string(line) + ": " + std::string{why})};
}

}  // namespace

auto parse_tire_file(std::string_view text)
    -> std::expected<MagicFormulaTire, lib::Status> {
  MagicFormulaTire tire;
  bool has_format = false;
  bool has_radius = false;
  bool has_load = false;
  std::size_t number = 0;
  for (std::size_t at = 0; at < text.size();) {
    std::size_t end = std::min(text.find('\n', at), text.size());
    std::string_view line = text.substr(at, end - at);
    at = end + 1;
    ++number;

    // $ starts a comment anywhere; ! starts one at the line's start.
    line = trim(line.substr(0, std::min(line.find('$'), line.size())));
    if (line.empty() || line.front() == '!' || line.front() == '[' ||
        line.front() == '{') {
      continue;
    }
    std::size_t equals = line.find('=');
    if (equals == std::string_view::npos) {
      continue;  // A row of a table.
    }
    std::string key = upper(trim(line.substr(0, equals)));
    std::string_view value = trim(line.substr(equals + 1));

    if (key == "PROPERTY_FILE_FORMAT") {
      std::string format = upper(value);
      std::erase(format, '\'');
      format = std::string{trim(format)};
      if (std::ranges::find(FORMATS, format) == FORMATS.end()) {
        return fail(number,
                    "format " + format + " is not the Magic Formula 5.2");
      }
      has_format = true;
      continue;
    }
    auto field = std::ranges::find(FIELDS, key, &Field::name);
    if (field == FIELDS.end()) {
      continue;  // Not a steady-state coefficient.
    }
    double parsed = 0.0;
    auto [rest, error] =
        std::from_chars(value.data(), value.data() + value.size(), parsed);
    if (error != std::errc{} ||
        !trim(std::string_view{rest, value.data() + value.size()}).empty()) {
      return fail(number, key + " is not a number");
    }
    tire.*(field->member) = parsed;
    has_radius = has_radius || key == "UNLOADED_RADIUS";
    has_load = has_load || key == "FNOMIN";
  }
  if (!has_format) {
    return fail(number, "no PROPERTY_FILE_FORMAT");
  }
  if (!has_radius || !has_load || !(tire.fnomin > 0.0)) {
    return fail(number, "no UNLOADED_RADIUS and positive FNOMIN");
  }
  return tire;
}

auto load_tire_file(const std::string& path)
    -> std::expected<MagicFormulaTire, lib::Status> {
  std::ifstream file{path};
  if (!file) {
    return std::unexpected(
        lib::raise(TireFileError::UNREADABLE, "cannot open " + path));
  }
  std::stringstream text;
  text << file.rdbuf();
  return parse_tire_file(text.str());
}

}  // namespace simon::format

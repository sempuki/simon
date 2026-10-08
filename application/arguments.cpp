// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include "application/arguments.hpp"

#include <algorithm>
#include <cstdio>
#include <format>
#include <print>
#include <utility>

#include "core/text.hpp"

namespace simon::application {

Arguments::Arguments(int argc, char** argv) {
  for (int i = 1; i < argc; ++i) {
    std::string_view argument = argv[i];
    if (!argument.starts_with("--")) {
      positional_.push_back(argument);
      continue;
    }
    argument.remove_prefix(2);
    std::size_t equals = argument.find('=');
    if (equals == std::string_view::npos) {
      options_.push_back(Option{.name = argument});
    } else {
      options_.push_back(Option{.name = argument.substr(0, equals),
                                .value = argument.substr(equals + 1)});
    }
  }
}

auto Arguments::text(std::size_t index, std::string_view fallback)
    -> std::string_view {
  positional_read_ = std::max(positional_read_, index + 1);
  return index < positional_.size() ? positional_[index] : fallback;
}

auto Arguments::integer(std::size_t index, std::int64_t fallback)
    -> std::int64_t {
  positional_read_ = std::max(positional_read_, index + 1);
  if (index >= positional_.size()) {
    return fallback;
  }
  std::optional<std::int64_t> value = parse_integer(positional_[index]);
  if (!value) {
    note(std::format("argument {}, \"{}\", is not a whole number", index + 1,
                     positional_[index]));
    return fallback;
  }
  return *value;
}

auto Arguments::number(std::size_t index, double fallback) -> double {
  positional_read_ = std::max(positional_read_, index + 1);
  if (index >= positional_.size()) {
    return fallback;
  }
  std::optional<double> value = parse_number(positional_[index]);
  if (!value) {
    note(std::format("argument {}, \"{}\", is not a number", index + 1,
                     positional_[index]));
    return fallback;
  }
  return *value;
}

auto Arguments::option_text(std::string_view name, std::string_view fallback)
    -> std::string_view {
  Option* option = find(name);
  if (!option) {
    return fallback;
  }
  if (!option->value) {
    note(std::format("--{} needs a value, --{}=...", name, name));
    return fallback;
  }
  return *option->value;
}

auto Arguments::option_integer(std::string_view name, std::int64_t fallback)
    -> std::int64_t {
  Option* option = find(name);
  if (!option) {
    return fallback;
  }
  std::optional<std::int64_t> value =
      option->value ? parse_integer(*option->value) : std::nullopt;
  if (!value) {
    note(std::format("--{} needs a whole number", name));
    return fallback;
  }
  return *value;
}

auto Arguments::option_number(std::string_view name, double fallback)
    -> double {
  Option* option = find(name);
  if (!option) {
    return fallback;
  }
  std::optional<double> value =
      option->value ? parse_number(*option->value) : std::nullopt;
  if (!value) {
    note(std::format("--{} needs a number", name));
    return fallback;
  }
  return *value;
}

auto Arguments::flag(std::string_view name) -> bool {
  Option* option = find(name);
  if (option && option->value) {
    note(std::format("--{} takes no value", name));
  }
  return option != nullptr;
}

auto Arguments::error() const -> std::optional<std::string> {
  if (error_) {
    return error_;
  }
  for (const Option& option : options_) {
    if (!option.read) {
      return std::format("unknown option --{}", option.name);
    }
  }
  if (positional_.size() > positional_read_) {
    return std::format("unexpected argument \"{}\"",
                       positional_[positional_read_]);
  }
  return std::nullopt;
}

auto Arguments::report_error(std::string_view usage) const -> bool {
  std::optional<std::string> message = error();
  if (message) {
    std::println(stderr, "{}\nUsage: {}", *message, usage);
  }
  return message.has_value();
}

auto Arguments::find(std::string_view name) -> Option* {
  auto option = std::ranges::find(options_, name, &Option::name);
  if (option == options_.end()) {
    return nullptr;
  }
  option->read = true;
  return &*option;
}

auto Arguments::note(std::string message) -> void {
  if (!error_) {
    error_ = std::move(message);
  }
}

}  // namespace simon::application

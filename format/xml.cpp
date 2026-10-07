// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include "format/xml.hpp"

#include <optional>
#include <utility>

#include "format/text.hpp"

namespace simon::format {

auto XmlDocument::load(std::string text) -> std::expected<void, lib::Status> {
  text_ = std::move(text);
  pugi::xml_parse_result result =
      document_.load_buffer(text_.data(), text_.size());
  if (!result) {
    return fail_at(format::find_line(text_, result.offset),
                   result.description());
  }
  return {};
}

auto XmlDocument::find_root(std::string_view name) const
    -> std::expected<pugi::xml_node, lib::Status> {
  pugi::xml_node root = document_.child(name);
  if (!root) {
    return Failure{lib::raise(FormatError::MALFORMED,
                              "no " + std::string{name} + " element")};
  }
  return root;
}

auto XmlDocument::holds(pugi::xml_node node) const -> bool {
  return node && node.root() == this->node();
}

auto XmlDocument::find_line(pugi::xml_node node) const -> std::size_t {
  return format::find_line(text_, node.offset_debug());
}

auto XmlDocument::read_number(pugi::xml_node node, std::string_view name) const
    -> std::expected<double, lib::Status> {
  pugi::xml_attribute attribute = node.attribute(name);
  if (!attribute) {
    return fail(node, "needs " + std::string{name});
  }
  std::string_view word = attribute.value();
  std::optional<double> value = parse_number(word);
  if (!value) {
    return fail(node, std::string{name} + " `" + std::string{word} +
                          "` is not a finite number");
  }
  return *value;
}

auto XmlDocument::fail(pugi::xml_node node, std::string_view why) const
    -> Failure {
  return Failure{lib::raise(FormatError::MALFORMED, describe(node, why))};
}

auto XmlDocument::refuse(pugi::xml_node node, std::string_view what) const
    -> Failure {
  return Failure{lib::raise(FormatError::UNSUPPORTED, describe(node, what))};
}

auto XmlDocument::describe(pugi::xml_node node, std::string_view rest) const
    -> std::string {
  if (!node) {
    return std::string{rest};
  }
  std::string text =
      "line " + std::to_string(find_line(node)) + ": <" + node.name() + ">";
  if (!rest.empty()) {
    text += ' ';
    text += rest;
  }
  return text;
}

}  // namespace simon::format

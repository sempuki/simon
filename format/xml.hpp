// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <cstddef>
#include <expected>
#include <string>
#include <string_view>

#include "base/core.hpp"
#include "base/status.hpp"
#include "format/format_error.hpp"
#include "pugixml.hpp"

// What every XML reader does with pugixml: reads a document whole and says
// on which line anything in it is wrong.
namespace simon::format {

// An XML document that keeps its text, so that it can say on which line any
// of its nodes stands. Nodes refer into it, so it stays put once loaded.
class XmlDocument final {
 public:
  DECLARE_COPY_DELETE(XmlDocument);
  DECLARE_MOVE_DELETE(XmlDocument);

  XmlDocument() = default;
  ~XmlDocument() = default;

  // Reads `text`; MALFORMED, with the line, if it is not XML.
  auto load(std::string text) -> std::expected<void, lib::Status>;

  // The document's element `name`; MALFORMED if it has none.
  auto find_root(std::string_view name) const
      -> std::expected<pugi::xml_node, lib::Status>;

  // The document node, which holds the root element.
  auto node() const -> pugi::xml_node { return document_; }

  // Whether `node` stands in this document.
  auto holds(pugi::xml_node node) const -> bool;

  // The line `node` starts on, counted from 1.
  auto find_line(pugi::xml_node node) const -> std::size_t;

  // The number in `node`'s attribute `name`; MALFORMED if it has none, or
  // it is not a finite number.
  auto read_number(pugi::xml_node node, std::string_view name) const
      -> std::expected<double, lib::Status>;

  // A MALFORMED failure: "line N: <node> why".
  auto fail(pugi::xml_node node, std::string_view why) const -> Failure;

  // An UNSUPPORTED failure: "line N: <node>", or "line N: <node> what".
  auto refuse(pugi::xml_node node, std::string_view what = {}) const -> Failure;

 private:
  // "line N: <node> rest", or only `rest` for no node.
  auto describe(pugi::xml_node node, std::string_view rest) const
      -> std::string;

  std::string text_;
  pugi::xml_document document_;
};

}  // namespace simon::format

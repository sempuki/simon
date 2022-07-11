// Copyright 2022 -- CONTRIBUTORS.

#pragma once

#include <cstddef>
#include <memory>

#include "framework/type_macros.hpp"

namespace simon::framework {

class Identity;

class Name {
  MARK_NON_DEFAULTABLE(Name);

 public:
  bool operator==(const Name& that) const { return name_ == that.name_; }
  bool operator!=(const Name& that) const { return name_ != that.name_; }
  bool operator<(const Name& that) const { return name_ < that.name_; }

 private:
  friend class Identity;
  Name(std::size_t name) : name_{name} {}
  std::size_t name_ = 0;
};

class Identity {
  MARK_NON_COPYABLE(Identity);

 public:
  Identity();
  Name name() const { return Name{id_}; }
  bool operator==(const Identity& that) const { return name() == that.name(); }
  bool operator!=(const Identity& that) const { return name() != that.name(); }
  bool operator<(const Identity& that) const { return name() < that.name(); }

 private:
  std::size_t id_ = 0;
};

}  // namespace simon::framework

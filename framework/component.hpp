// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include "framework/identity.hpp"

namespace simon::framework {

struct ComponentName final : public Name {
  ComponentName(Name name) : Name{name} {}
};

class ComponentBase {
 public:
  virtual ComponentName component_name() const = 0;
};

template <typename ComponentType>
class Component : public ComponentBase, public PerTypeIdentity<ComponentType> {
 public:
  ComponentName component_name() const override { return Component::name(); }
  static ComponentName name() { return PerTypeIdentity<ComponentType>::id_.name(); }
};

}  // namespace simon::framework


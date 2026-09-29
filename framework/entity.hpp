// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <map>

#include "framework/component.hpp"
#include "framework/identity.hpp"

namespace simon::framework {

struct EntityName final : public Name {
  EntityName(Name name) : Name{name} {}
};

class Entity : public PerObjectIdentity {
 public:
  EntityName entity_name() const { return id_.name(); }
  // An entity holds at most one component of each type.
  void attach(ComponentBase* component) {
    CHECK_PRECONDITION(component);
    auto [iter, inserted] = components_.try_emplace(component->component_name(), component);
    CHECK_PRECONDITION(inserted);
  }

  template <typename ComponentType>
  ComponentType* component() {
    auto iter = components_.find(ComponentType::name());
    return iter != components_.end() ? dynamic_cast<ComponentType*>(iter->second) : nullptr;
  }

 private:
  std::map<ComponentName, ComponentBase*> components_;
};

}  // namespace simon::framework


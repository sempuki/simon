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
  void attach(ComponentBase* component) { components_[component->component_name()] = component; }

  template <typename ComponentType>
  ComponentType* component() {
    auto iter = components_.find(ComponentType::name());
    return iter != components_.end() ? dynamic_cast<ComponentType*>(iter->second) : nullptr;
  }

 private:
  std::map<ComponentName, ComponentBase*> components_;
};

}  // namespace simon::framework


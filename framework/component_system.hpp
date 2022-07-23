// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <iostream>
#include <memory>
#include <vector>

#include "framework/component.hpp"
#include "framework/entity.hpp"

namespace simon::framework {

class ComponentSystemBase {
 public:
  virtual ComponentName component_name() const = 0;
};

template <typename ComponentType>
class TypedComponentSystemBase : public ComponentSystemBase {
 public:
  ComponentName component_name() const override { return ComponentType::name(); }

  ComponentType* attach(Entity* entity) {
    components_.emplace_back(std::make_unique<ComponentType>());
    auto* component = components_.back().get();
    entity->attach(component);
    return component;
  }

 protected:
  std::vector<std::unique_ptr<ComponentType>> components_;
};

template <typename ComponentType, typename ComputationType>
struct ComponentSystem final : public TypedComponentSystemBase<ComponentType> {
  ComponentSystem() = default;

  template <typename StatefulComputation>
  ComponentSystem(StatefulComputation&& compute)
    : compute_{std::forward<StatefulComputation>(compute)} {}

  template <typename EventSink>
  void operator()(double time, double step, EventSink events) {
    for (auto&& component : this->components_) {
      compute_.prepare();
    }

    for (auto&& component : this->components_) {
      compute_(component.get(), time, step, events);
    }

    for (auto&& component : this->components_) {
      compute_.resolve();
    }
  }

  ComputationType compute_;
};

}  // namespace simon::framework


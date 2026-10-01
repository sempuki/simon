// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <cstddef>
#include <variant>

#include "framework/entity.hpp"
#include "framework/type_list.hpp"

namespace simon::framework {

// Commands are the low-level vocabulary of structural change. Builders emit
// them; the world applies them at sync points, in the order they were recorded.

template <typename ComponentType>
struct AttachCommand final {
  Entity entity;
  ComponentType component;
  std::size_t segment = 0;  // In the component's store; see ComponentStore.
};

template <typename ComponentType>
struct DetachCommand final {
  Entity entity;
};

struct DestroyCommand final {
  Entity entity;
};

template <typename ComponentListType>
struct CommandFor;

template <typename... ComponentTypes>
struct CommandFor<TypeList<ComponentTypes...>> final {
  using type = std::variant<DestroyCommand, AttachCommand<ComponentTypes>...,
                            DetachCommand<ComponentTypes>...>;
};

template <typename ComponentListType>
using command_for_t = typename CommandFor<ComponentListType>::type;

}  // namespace simon::framework

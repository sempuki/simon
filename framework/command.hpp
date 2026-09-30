// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <variant>

#include "framework/entity.hpp"
#include "framework/type_list.hpp"

namespace simon::framework {

// Commands are the low-level vocabulary of structural change. Builders emit
// them; the world applies them at sync points, in the order they were recorded.

template <typename Component>
struct AttachCommand {
  Entity entity;
  Component component;
};

template <typename Component>
struct DetachCommand {
  Entity entity;
};

struct DestroyCommand {
  Entity entity;
};

template <typename ComponentList>
struct CommandFor;

template <typename... Components>
struct CommandFor<TypeList<Components...>> {
  using type = std::variant<DestroyCommand, AttachCommand<Components>...,
                            DetachCommand<Components>...>;
};

template <typename ComponentList>
using command_for_t = typename CommandFor<ComponentList>::type;

}  // namespace simon::framework

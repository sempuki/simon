// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <array>
#include <cstddef>
#include <expected>
#include <optional>
#include <string>
#include <string_view>
#include <tuple>
#include <type_traits>
#include <utility>
#include <vector>

#include "base/core.hpp"
#include "base/status.hpp"
#include "core/argument.hpp"
#include "framework/archetype.hpp"
#include "framework/build_error.hpp"
#include "framework/entity.hpp"
#include "framework/name.hpp"
#include "framework/type_list.hpp"

namespace simon::framework {

// Builders are the only way to write to a world, and they form a fluent
// grammar. Each utterance starts with a verb on the world and ends with
// `build()`:
//
//   world.create<Ball>("Luke Skywalker").under(squadron)
//        .with(Kinematics{}).build();
//   world.change(ball).attach(Health{}).detach<Drag>().alias("ego").build();
//   world.destroy(ball).build();
//
// A builder accumulates the utterance, validates it when it is complete, and
// only then emits commands. The grammar is checked at compile time: word order,
// and which components the entity's archetype requires and permits. Anything
// that depends on the world's data is checked by `build()` against the state
// the world will be in once pending commands apply. A refused utterance returns
// a Status (see BuildError) and emits nothing, so applying commands at a
// sync point never fails.

template <typename ComponentType>
inline constexpr bool is_built_in_v =
    std::is_same_v<ComponentType, EntityArchetype> ||
    std::is_same_v<ComponentType, Parent>;

template <typename WorldType, Archetypal ArchetypeType, bool CanParent,
          typename... InitialTypes>
class [[nodiscard]] CreateBuilder final {
 public:
  // Keeps a reference to `world` until the utterance is built.
  CreateBuilder(Alias alias, std::optional<Entity> parent,
                std::tuple<InitialTypes...> components, Depend<WorldType> world)
      : world_{world.get()},
        alias_{std::move(alias)},
        parent_{parent},
        components_{std::move(components)} {}

  // Records `parent` as the entity this one was created under. At most once,
  // before any `with`.
  auto under(Entity parent) &&
    requires CanParent
  {
    return CreateBuilder<WorldType, ArchetypeType, false>{
        std::move(alias_), parent, {}, Depend(*world_)};
  }

  // Declares a component the entity starts with.
  template <typename ArgumentType>
  auto with(ArgumentType&& component) && {
    using ComponentType = std::remove_cvref_t<ArgumentType>;
    static_assert(contains_v<typename WorldType::ComponentList, ComponentType>,
                  "This component is not in the world's component list.");
    static_assert(
        !is_built_in_v<ComponentType>,
        "Built-in components come from create<ArchetypeType> and under().");
    static_assert(
        contains_v<typename ArchetypeType::PermittedComponentList,
                   ComponentType>,
        "The entity's archetype neither requires nor allows this component.");
    static_assert(!contains_v<TypeList<InitialTypes...>, ComponentType>,
                  "The entity already starts with this component.");
    return CreateBuilder<WorldType, ArchetypeType, false, InitialTypes...,
                         ComponentType>{
        std::move(alias_), parent_,
        std::tuple_cat(
            std::move(components_),
            std::tuple<ComponentType>{std::forward<ArgumentType>(component)}),
        Depend(*world_)};
  }

  auto build() && -> std::expected<Entity, Status> {
    static_assert(is_subset_v<typename ArchetypeType::RequiredComponentList,
                              TypeList<InitialTypes...>>,
                  "The entity lacks a component its archetype requires.");
    return world_->template build_create<ArchetypeType>(alias_, parent_,
                                                        std::move(components_));
  }

 private:
  // Never null once constructed; Depend checks it.
  WorldType* world_ = nullptr;
  Alias alias_;
  std::optional<Entity> parent_;
  std::tuple<InitialTypes...> components_;
};

struct AliasChanges final {
  std::vector<Alias> given;
  std::vector<Alias> taken;
};

template <typename WorldType, typename AttachedListType = TypeList<>,
          typename DetachedListType = TypeList<>, bool Aliasing = false>
class ChangeBuilder;

template <typename WorldType, typename... AttachedTypes,
          typename... DetachedTypes, bool Aliasing>
class [[nodiscard]]
ChangeBuilder<WorldType, TypeList<AttachedTypes...>, TypeList<DetachedTypes...>,
              Aliasing>
    final {
 public:
  // Keeps a reference to `world` until the utterance is built.
  ChangeBuilder(Entity entity, std::tuple<AttachedTypes...> components,
                AliasChanges aliases, Depend<WorldType> world)
      : world_{world.get()},
        entity_{entity},
        components_{std::move(components)},
        aliases_{std::move(aliases)} {}

  // Attaches a component to the live entity.
  template <typename ArgumentType>
  auto attach(ArgumentType&& component) && {
    using ComponentType = std::remove_cvref_t<ArgumentType>;
    check_component<ComponentType>();
    return ChangeBuilder<WorldType, TypeList<AttachedTypes..., ComponentType>,
                         TypeList<DetachedTypes...>, Aliasing>{
        entity_,
        std::tuple_cat(
            std::move(components_),
            std::tuple<ComponentType>{std::forward<ArgumentType>(component)}),
        std::move(aliases_), Depend(*world_)};
  }

  // Detaches a component from the live entity.
  template <typename ComponentType>
  auto detach() && {
    check_component<ComponentType>();
    return ChangeBuilder<WorldType, TypeList<AttachedTypes...>,
                         TypeList<DetachedTypes..., ComponentType>, Aliasing>{
        entity_, std::move(components_), std::move(aliases_), Depend(*world_)};
  }

  // Gives the entity an alias, such as "ego".
  auto alias(Alias alias) && {
    aliases_.given.push_back(std::move(alias));
    return ChangeBuilder<WorldType, TypeList<AttachedTypes...>,
                         TypeList<DetachedTypes...>, true>{
        entity_, std::move(components_), std::move(aliases_), Depend(*world_)};
  }

  // Takes an alias away from the entity.
  auto unalias(Alias alias) && {
    aliases_.taken.push_back(std::move(alias));
    return ChangeBuilder<WorldType, TypeList<AttachedTypes...>,
                         TypeList<DetachedTypes...>, true>{
        entity_, std::move(components_), std::move(aliases_), Depend(*world_)};
  }

  auto build() && -> std::expected<void, Status> {
    static_assert(
        sizeof...(AttachedTypes) + sizeof...(DetachedTypes) > 0 || Aliasing,
        "A change must attach, detach, alias or unalias something.");
    return world_->build_change(entity_, std::move(components_),
                                TypeList<DetachedTypes...>{}, aliases_);
  }

 private:
  template <typename ComponentType>
  static constexpr auto check_component() -> void {
    static_assert(contains_v<typename WorldType::ComponentList, ComponentType>,
                  "This component is not in the world's component list.");
    static_assert(!is_built_in_v<ComponentType>,
                  "An entity's archetype and parent cannot change.");
    static_assert(
        !contains_v<TypeList<AttachedTypes..., DetachedTypes...>,
                    ComponentType>,
        "Each component may be attached or detached once per change.");
  }

  // Never null once constructed; Depend checks it.
  WorldType* world_ = nullptr;
  Entity entity_;
  std::tuple<AttachedTypes...> components_;
  AliasChanges aliases_;
};

template <typename WorldType>
class [[nodiscard]] DestroyBuilder final {
 public:
  // Keeps a reference to `world` until the utterance is built.
  DestroyBuilder(Entity entity, Depend<WorldType> world)
      : world_{world.get()}, entity_{entity} {}

  auto build() && -> std::expected<void, Status> {
    return world_->build_destroy(entity_);
  }

 private:
  WorldType* world_ = nullptr;
  Entity entity_;
};

}  // namespace simon::framework

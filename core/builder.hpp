// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <expected>
#include <optional>
#include <string>
#include <string_view>
#include <tuple>
#include <type_traits>
#include <utility>
#include <vector>

#include "base/core.hpp"
#include "core/archetype.hpp"
#include "core/entity.hpp"
#include "core/status.hpp"
#include "core/type_list.hpp"

namespace simon::core {

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
// a Status (see BuildCondition) and emits nothing, so applying commands at a
// sync point never fails.

template <typename Component>
inline constexpr bool is_built_in_v =
    std::is_same_v<Component, EntityArchetype> ||
    std::is_same_v<Component, Parent>;

template <typename World, ArchetypeType Archetype, bool CanParent,
          typename... Initial>
class [[nodiscard]] CreateBuilder final {
 public:
  // Keeps a reference to `world` until the utterance is built.
  CreateBuilder(lib::Depend<World> world, std::string alias,
                std::optional<Entity> parent, std::tuple<Initial...> components)
      : world_{world.get()},
        alias_{std::move(alias)},
        parent_{parent},
        components_{std::move(components)} {}

  // Records `parent` as the entity this one was created under. At most once,
  // before any `with`.
  auto under(Entity parent) &&
    requires CanParent
  {
    return CreateBuilder<World, Archetype, false>{
        lib::Depend<World>{*world_}, std::move(alias_), parent, {}};
  }

  // Declares a component the entity starts with.
  template <typename Argument>
  auto with(Argument&& component) && {
    using Component = std::remove_cvref_t<Argument>;
    static_assert(contains_v<typename World::ComponentList, Component>,
                  "This component is not in the world's component list.");
    static_assert(
        !is_built_in_v<Component>,
        "Built-in components come from create<Archetype> and under().");
    static_assert(
        contains_v<typename Archetype::PermittedComponents, Component>,
        "The entity's archetype neither requires nor allows this component.");
    static_assert(!contains_v<TypeList<Initial...>, Component>,
                  "The entity already starts with this component.");
    return CreateBuilder<World, Archetype, false, Initial..., Component>{
        lib::Depend<World>{*world_}, std::move(alias_), parent_,
        std::tuple_cat(
            std::move(components_),
            std::tuple<Component>{std::forward<Argument>(component)})};
  }

  std::expected<Entity, Status> build() && {
    static_assert(is_subset_v<typename Archetype::RequiredComponents,
                              TypeList<Initial...>>,
                  "The entity lacks a component its archetype requires.");
    return world_->template build_create<Archetype>(alias_, parent_,
                                                    std::move(components_));
  }

 private:
  World* world_;  // Never null; checked once by Depend at construction.
  std::string alias_;
  std::optional<Entity> parent_;
  std::tuple<Initial...> components_;
};

struct AliasChanges {
  std::vector<std::string> given;
  std::vector<std::string> taken;
};

template <typename World, typename AttachedList = TypeList<>,
          typename DetachedList = TypeList<>, bool Aliasing = false>
class ChangeBuilder;

template <typename World, typename... Attached, typename... Detached,
          bool Aliasing>
class [[nodiscard]]
ChangeBuilder<World, TypeList<Attached...>, TypeList<Detached...>, Aliasing>
    final {
 public:
  // Keeps a reference to `world` until the utterance is built.
  ChangeBuilder(lib::Depend<World> world, Entity entity,
                std::tuple<Attached...> components, AliasChanges aliases)
      : world_{world.get()},
        entity_{entity},
        components_{std::move(components)},
        aliases_{std::move(aliases)} {}

  // Attaches a component to the live entity.
  template <typename Argument>
  auto attach(Argument&& component) && {
    using Component = std::remove_cvref_t<Argument>;
    check_component<Component>();
    return ChangeBuilder<World, TypeList<Attached..., Component>,
                         TypeList<Detached...>, Aliasing>{
        lib::Depend<World>{*world_}, entity_,
        std::tuple_cat(
            std::move(components_),
            std::tuple<Component>{std::forward<Argument>(component)}),
        std::move(aliases_)};
  }

  // Detaches a component from the live entity.
  template <typename Component>
  auto detach() && {
    check_component<Component>();
    return ChangeBuilder<World, TypeList<Attached...>,
                         TypeList<Detached..., Component>, Aliasing>{
        lib::Depend<World>{*world_}, entity_, std::move(components_),
        std::move(aliases_)};
  }

  // Gives the entity an alias, such as "ego".
  auto alias(std::string_view alias) && {
    aliases_.given.emplace_back(alias);
    return ChangeBuilder<World, TypeList<Attached...>, TypeList<Detached...>,
                         true>{lib::Depend<World>{*world_}, entity_,
                               std::move(components_), std::move(aliases_)};
  }

  // Takes an alias away from the entity.
  auto unalias(std::string_view alias) && {
    aliases_.taken.emplace_back(alias);
    return ChangeBuilder<World, TypeList<Attached...>, TypeList<Detached...>,
                         true>{lib::Depend<World>{*world_}, entity_,
                               std::move(components_), std::move(aliases_)};
  }

  std::expected<void, Status> build() && {
    static_assert(sizeof...(Attached) + sizeof...(Detached) > 0 || Aliasing,
                  "A change must attach, detach, alias or unalias something.");
    return world_->build_change(entity_, std::move(components_),
                                TypeList<Detached...>{}, aliases_);
  }

 private:
  template <typename Component>
  static constexpr void check_component() {
    static_assert(contains_v<typename World::ComponentList, Component>,
                  "This component is not in the world's component list.");
    static_assert(!is_built_in_v<Component>,
                  "An entity's archetype and parent cannot change.");
    static_assert(
        !contains_v<TypeList<Attached..., Detached...>, Component>,
        "Each component may be attached or detached once per change.");
  }

  World* world_;  // Never null; checked once by Depend at construction.
  Entity entity_;
  std::tuple<Attached...> components_;
  AliasChanges aliases_;
};

template <typename World>
class [[nodiscard]] DestroyBuilder final {
 public:
  // Keeps a reference to `world` until the utterance is built.
  DestroyBuilder(lib::Depend<World> world, Entity entity)
      : world_{world.get()}, entity_{entity} {}

  std::expected<void, Status> build() && {
    return world_->build_destroy(entity_);
  }

 private:
  World* world_;  // Never null; checked once by Depend at construction.
  Entity entity_;
};

}  // namespace simon::core

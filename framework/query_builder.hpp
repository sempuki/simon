// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <algorithm>
#include <cstddef>
#include <expected>
#include <functional>
#include <optional>
#include <tuple>
#include <type_traits>
#include <utility>
#include <vector>

#include "base/core.hpp"
#include "base/status.hpp"
#include "core/argument.hpp"
#include "framework/archetype.hpp"
#include "framework/entity.hpp"
#include "framework/entity_builder.hpp"
#include "framework/name.hpp"
#include "framework/spatial.hpp"
#include "framework/type_list.hpp"

namespace simon::framework {

// A query form's read policy: which components its selection may read. Choosing
// entities by a component reads that component's store, and `within` reads the
// spatial component through the spatial index. A query form started from the
// world may read anything; one started from a system's ProjectedWorld may read
// only what the system's AllowComponentList declares.
struct ReadAnything final {
  template <typename ComponentType>
  static constexpr bool can_read = true;
};

// Selects the entities for a query form. `each<ChosenType>()` chose them by
// archetype, if ChosenType is one, or else by component; `near`, `keep`,
// `having` and `lacking` narrow them. Entities already planned for destruction
// are never selected.
template <typename WorldType, typename ChosenType>
class Query final {
 public:
  using SpatialType = typename WorldType::SpatialComponent;
  using DistanceType = distance_of_t<SpatialType>;

  static_assert(
      Archetypal<ChosenType>
          ? contains_v<typename WorldType::ArchetypeList, ChosenType>
          : contains_v<typename WorldType::ComponentList, ChosenType>,
      "Choose an archetype in the world's archetype list or a component in "
      "its component list.");

  // Keeps only entities whose spatial component is within `radius` of
  // `center`.
  auto near(const SpatialType& center, DistanceType radius) -> void {
    near_ = Near{.center = center, .radius = radius};
  }

  // Keeps only entities for which `predicate(Entity)` is true.
  template <typename PredicateType>
  auto keep(PredicateType&& predicate) -> void {
    predicates_.emplace_back(std::forward<PredicateType>(predicate));
  }

  // Keeps only entities that will have `ComponentType` once pending commands
  // apply.
  template <typename ComponentType>
  auto having() -> void {
    conditions_.push_back([](const WorldType& world, Entity entity) {
      return world.template will_have<ComponentType>(entity);
    });
  }

  // Keeps only entities that will lack `ComponentType` once pending commands
  // apply.
  template <typename ComponentType>
  auto lacking() -> void {
    conditions_.push_back([](const WorldType& world, Entity entity) {
      return !world.template will_have<ComponentType>(entity);
    });
  }

  // The selected entities, in a deterministic order: the archetype's segment
  // or the component's store, or the spatial index's order when narrowed by
  // `near`.
  auto select(InOut<WorldType> world) const -> std::vector<Entity> {
    std::vector<Entity> selected;
    auto consider = [&](Entity entity) {
      if (world->will_be_alive(entity) &&
          std::ranges::all_of(conditions_,
                              [&](const auto& condition) {
                                return condition(*world, entity);
                              }) &&
          std::ranges::all_of(predicates_, [&](const auto& predicate) {
            return predicate(entity);
          })) {
        selected.push_back(entity);
      }
    };
    if (near_) {
      world->within(near_->center, near_->radius,
                    [&](Entity entity, const SpatialType&) {
                      if (is_chosen(*world, entity)) {
                        consider(entity);
                      }
                    });
    } else {
      for_each_chosen(*world, consider);
    }
    return selected;
  }

 private:
  struct Near final {
    SpatialType center;
    DistanceType radius;
  };

  static auto is_chosen(const WorldType& world, Entity entity) -> bool {
    if constexpr (Archetypal<ChosenType>) {
      return world.archetype_of_index_[entity.index] ==
             index_of_v<typename WorldType::ArchetypeList, ChosenType>;
    } else {
      return world.template store_of<ChosenType>().contains(entity);
    }
  }

  // Visits every chosen entity: every entity of the archetype, or every owner
  // in the component's store.
  template <typename VisitorType>
  static auto for_each_chosen(const WorldType& world, VisitorType&& visit)
      -> void {
    if constexpr (Archetypal<ChosenType>) {
      world.template for_each_entity_of<ChosenType>(visit);
    } else {
      world.template store_of<ChosenType>().for_each(
          [&](Entity owner, const ChosenType&) { visit(owner); });
    }
  }

  std::optional<Near> near_;
  std::vector<std::function<bool(Entity)>> predicates_;
  // The conditions `having` and `lacking` ask of the world's planned state.
  std::vector<bool (*)(const WorldType&, Entity)> conditions_;
};

// Destroys every entity a query selects, atomically, and returns how many:
//
//   world.destroy()
//       .each<archetype::RedDrone>()  // Or a component: each<Health>().
//       .within(asset_kinematics, 500.0 * meter)
//       .where([&](Entity drone) { return drone != spared; })
//       .build();
//
// `each` comes first. `build()` plans every destruction in one transaction,
// so it destroys all of the selected entities or none.
template <typename WorldType, typename ReadPolicyType, typename ChosenType>
class [[nodiscard]] DestroyQueryBuilder final {
 public:
  using SpatialType = typename WorldType::SpatialComponent;
  using DistanceType = distance_of_t<SpatialType>;

  // Keeps a reference to `world` until the utterance is built.
  explicit DestroyQueryBuilder(Depend<WorldType> world) : world_{world.get()} {}

  // Selects every entity of archetype `NextType`, or every entity with
  // component `NextType`.
  template <typename NextType>
    requires std::is_void_v<ChosenType>
  auto each() && {
    static_assert(
        Archetypal<NextType> || ReadPolicyType::template can_read<NextType>,
        "Declare this component in the system's AllowComponentList "
        "to select by it.");
    return DestroyQueryBuilder<WorldType, ReadPolicyType, NextType>{
        Depend(*world_)};
  }

  // Keeps only entities whose spatial component is within `radius` of
  // `center`.
  auto within(const SpatialType& center, DistanceType radius) &&
    requires(!std::is_void_v<ChosenType>)
  {
    static_assert(ReadPolicyType::template can_read<SpatialType>,
                  "Declare the spatial component in the system's "
                  "AllowComponentList to query space.");
    query_.near(center, radius);
    return std::move(*this);
  }

  // Keeps only entities for which `predicate(Entity)` is true. Predicates add
  // up.
  template <typename PredicateType>
  auto where(PredicateType&& predicate) &&
    requires(!std::is_void_v<ChosenType>)
  {
    query_.keep(std::forward<PredicateType>(predicate));
    return std::move(*this);
  }

  // Keeps only entities that will have `ComponentType` once pending commands
  // apply.
  template <typename ComponentType>
  auto having() &&
    requires(!std::is_void_v<ChosenType>)
  {
    check_readable<ComponentType>();
    query_.template having<ComponentType>();
    return std::move(*this);
  }

  // Keeps only entities that will lack `ComponentType` once pending commands
  // apply.
  template <typename ComponentType>
  auto lacking() &&
    requires(!std::is_void_v<ChosenType>)
  {
    check_readable<ComponentType>();
    query_.template lacking<ComponentType>();
    return std::move(*this);
  }

  auto build() && -> std::expected<std::size_t, Status>
    requires(!std::is_void_v<ChosenType>)
  {
    std::vector<Entity> selected = query_.select(InOut(*world_));
    auto transaction = world_->transaction();
    for (Entity entity : selected) {
      RETURN_IF_UNEXPECTED(world_->destroy(entity).build());
    }
    transaction.commit();
    return selected.size();
  }

 private:
  template <typename, typename, typename>
  friend class DestroyQueryBuilder;

  template <typename ComponentType>
  static constexpr auto check_readable() -> void {
    static_assert(contains_v<typename WorldType::ComponentList, ComponentType>,
                  "This component is not in the world's component list.");
    static_assert(ReadPolicyType::template can_read<ComponentType>,
                  "Declare this component in the system's AllowComponentList "
                  "to select by it.");
  }

  struct Unchosen final {};
  using QueryType = std::conditional_t<std::is_void_v<ChosenType>, Unchosen,
                                       Query<WorldType, ChosenType>>;

  WorldType* world_ = nullptr;
  QueryType query_;
};

// Makes the same change to every entity a query selects, atomically, and
// returns how many it changed:
//
//   world.change()
//       .each<archetype::RedDrone>()
//       .within(asset_kinematics, 4000.0 * meter)
//       .attach(Tracked{})
//       .alias("hostile")
//       .build();
//
// `each` comes first, then any of `within` and `where`, then the change in the
// words of ChangeBuilder: `attach`, `detach`, `alias` and `unalias`. `build()`
// makes the change to each selected entity in one transaction; if the world
// refuses it for any of them, nothing changes and build() returns the Status.
template <typename WorldType, typename ReadPolicyType, typename ChosenType,
          typename AttachedListType = TypeList<>,
          typename DetachedListType = TypeList<>, bool Aliasing = false>
class ChangeQueryBuilder;

template <typename WorldType, typename ReadPolicyType, typename ChosenType,
          typename... AttachedTypes, typename... DetachedTypes, bool Aliasing>
class [[nodiscard]]
ChangeQueryBuilder<WorldType, ReadPolicyType, ChosenType,
                   TypeList<AttachedTypes...>, TypeList<DetachedTypes...>,
                   Aliasing>
    final {
 public:
  using SpatialType = typename WorldType::SpatialComponent;
  using DistanceType = distance_of_t<SpatialType>;

  // Keeps a reference to `world` until the utterance is built.
  explicit ChangeQueryBuilder(Depend<WorldType> world) : world_{world.get()} {}

  // Selects every entity of archetype `NextType`, or every entity with
  // component `NextType`.
  template <typename NextType>
    requires std::is_void_v<ChosenType>
  auto each() && {
    static_assert(
        Archetypal<NextType> || ReadPolicyType::template can_read<NextType>,
        "Declare this component in the system's AllowComponentList "
        "to select by it.");
    return ChangeQueryBuilder<WorldType, ReadPolicyType, NextType, TypeList<>,
                              TypeList<>, false>{Depend(*world_)};
  }

  // Keeps only entities whose spatial component is within `radius` of
  // `center`.
  auto within(const SpatialType& center, DistanceType radius) &&
    requires(!std::is_void_v<ChosenType>)
  {
    static_assert(ReadPolicyType::template can_read<SpatialType>,
                  "Declare the spatial component in the system's "
                  "AllowComponentList to query space.");
    query_.near(center, radius);
    return std::move(*this);
  }

  // Keeps only entities for which `predicate(Entity)` is true. Predicates add
  // up.
  template <typename PredicateType>
  auto where(PredicateType&& predicate) &&
    requires(!std::is_void_v<ChosenType>)
  {
    query_.keep(std::forward<PredicateType>(predicate));
    return std::move(*this);
  }

  // Keeps only entities that will have `ComponentType` once pending commands
  // apply.
  template <typename ComponentType>
  auto having() &&
    requires(!std::is_void_v<ChosenType>)
  {
    check_readable<ComponentType>();
    query_.template having<ComponentType>();
    return std::move(*this);
  }

  // Keeps only entities that will lack `ComponentType` once pending commands
  // apply.
  template <typename ComponentType>
  auto lacking() &&
    requires(!std::is_void_v<ChosenType>)
  {
    check_readable<ComponentType>();
    query_.template lacking<ComponentType>();
    return std::move(*this);
  }

  // Attaches a copy of `component` to every selected entity.
  template <typename ArgumentType>
  auto attach(ArgumentType&& component) &&
    requires(!std::is_void_v<ChosenType>)
  {
    using ComponentType = std::remove_cvref_t<ArgumentType>;
    check_component<ComponentType>();
    return ChangeQueryBuilder<WorldType, ReadPolicyType, ChosenType,
                              TypeList<AttachedTypes..., ComponentType>,
                              TypeList<DetachedTypes...>, Aliasing>{
        Depend(*world_), std::move(query_),
        std::tuple_cat(
            std::move(components_),
            std::tuple<ComponentType>{std::forward<ArgumentType>(component)}),
        std::move(aliases_)};
  }

  // Detaches `ComponentType` from every selected entity.
  template <typename ComponentType>
  auto detach() &&
    requires(!std::is_void_v<ChosenType>)
  {
    check_component<ComponentType>();
    return ChangeQueryBuilder<
        WorldType, ReadPolicyType, ChosenType, TypeList<AttachedTypes...>,
        TypeList<DetachedTypes..., ComponentType>, Aliasing>{
        Depend(*world_), std::move(query_), std::move(components_),
        std::move(aliases_)};
  }

  // Gives every selected entity the alias `alias`.
  auto alias(Alias alias) &&
    requires(!std::is_void_v<ChosenType>)
  {
    aliases_.given.push_back(std::move(alias));
    return ChangeQueryBuilder<WorldType, ReadPolicyType, ChosenType,
                              TypeList<AttachedTypes...>,
                              TypeList<DetachedTypes...>, true>{
        Depend(*world_), std::move(query_), std::move(components_),
        std::move(aliases_)};
  }

  // Takes the alias `alias` from every selected entity.
  auto unalias(Alias alias) &&
    requires(!std::is_void_v<ChosenType>)
  {
    aliases_.taken.push_back(std::move(alias));
    return ChangeQueryBuilder<WorldType, ReadPolicyType, ChosenType,
                              TypeList<AttachedTypes...>,
                              TypeList<DetachedTypes...>, true>{
        Depend(*world_), std::move(query_), std::move(components_),
        std::move(aliases_)};
  }

  auto build() && -> std::expected<std::size_t, Status>
    requires(!std::is_void_v<ChosenType>)
  {
    static_assert(
        sizeof...(AttachedTypes) + sizeof...(DetachedTypes) > 0 || Aliasing,
        "A change must attach, detach, alias or unalias something.");
    std::vector<Entity> selected = query_.select(InOut(*world_));
    auto transaction = world_->transaction();
    for (Entity entity : selected) {
      RETURN_IF_UNEXPECTED((ChangeBuilder<WorldType, TypeList<AttachedTypes...>,
                                          TypeList<DetachedTypes...>, Aliasing>{
                                entity, components_, aliases_, Depend(*world_)})
                               .build());
    }
    transaction.commit();
    return selected.size();
  }

 private:
  template <typename, typename, typename, typename, typename, bool>
  friend class ChangeQueryBuilder;

  template <typename ComponentType>
  static constexpr auto check_readable() -> void {
    static_assert(contains_v<typename WorldType::ComponentList, ComponentType>,
                  "This component is not in the world's component list.");
    static_assert(ReadPolicyType::template can_read<ComponentType>,
                  "Declare this component in the system's AllowComponentList "
                  "to select by it.");
  }

  struct Unchosen final {};
  using QueryType = std::conditional_t<std::is_void_v<ChosenType>, Unchosen,
                                       Query<WorldType, ChosenType>>;

  ChangeQueryBuilder(Depend<WorldType> world, QueryType query,
                     std::tuple<AttachedTypes...> components,
                     AliasChanges aliases)
      : world_{world.get()},
        query_{std::move(query)},
        components_{std::move(components)},
        aliases_{std::move(aliases)} {}

  // The same checks as ChangeBuilder's, made where the word is spoken.
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

  WorldType* world_ = nullptr;
  QueryType query_;
  std::tuple<AttachedTypes...> components_;
  AliasChanges aliases_;
};

}  // namespace simon::framework

// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <expected>
#include <format>
#include <functional>
#include <limits>
#include <map>
#include <optional>
#include <string>
#include <string_view>
#include <tuple>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <variant>
#include <vector>

#include "base/core.hpp"
#include "framework/archetype.hpp"
#include "framework/build_error.hpp"
#include "framework/command.hpp"
#include "framework/component_store.hpp"
#include "framework/entity.hpp"
#include "framework/entity_builder.hpp"
#include "framework/name.hpp"
#include "framework/query_builder.hpp"
#include "framework/spatial.hpp"
#include "framework/spatial_index.hpp"
#include "framework/type_list.hpp"
#include "framework/vocabulary.hpp"
#include "framework/world_builder.hpp"

namespace simon::framework {

struct SystemRunner;
struct ContinuousRunner;

// Unlocks mutable store access. Only the scheduler, which runs systems, and
// the integrator of continuous state can construct one.
class SchedulerKey final {
 private:
  SchedulerKey() = default;
  friend struct SystemRunner;
  friend struct ContinuousRunner;
};

// The entity database: a factory of builders (the only way to write) and a
// query interface over the result (the only way to read).
//
// A world's type is declared by its spatial component, its other components
// and its archetypes, every one of which it may create. A world is built by
// the builder `set_up()` returns, which sizes it by how many of each archetype
// it holds, and fills the caller's world:
//
//   using World = World<Kinematics, TypeList<Control, Health>,
//                       TypeList<archetype::Drone, archetype::Blast>>;
//   World world;  // Empty until built.
//   std::expected<void, Status> built = World::set_up().numbered(1)
//       .holding<archetype::Drone>(1000).build(Out(world));
//
// A world never moves. Builders and ProjectedWorld keep a pointer to it, and
// its stores never reallocate, so nothing that refers to a world can dangle
// while it lives.
//
// Everything in it has a Name ({kind, instance}), an Identity computed from the
// Name ("/world/1/entity/2"), and any number of Aliases ("ego"). See name.hpp.
template <Spatial SpatialType,         //
          typename ComponentListType,  //
          typename ArchetypeListType>
class World;

template <Spatial SpatialType,            //
          typename... ComponentTypes,     //
          typename... ArchetypeTypes>     //
class World<SpatialType,                  //
            TypeList<ComponentTypes...>,  //
            TypeList<ArchetypeTypes...>>
    final {
 public:
  using SpatialComponent = SpatialType;
  using ComponentList = TypeList<  //
      SpatialType, EntityArchetype, Parent, ComponentTypes...>;
  using ArchetypeList = TypeList<ArchetypeTypes...>;
  using Command = command_for_t<ComponentList>;

  static_assert(is_unique_v<ComponentList>,
                "Each component may appear once in a world's component list.");
  static_assert((Archetypal<ArchetypeTypes> && ...),
                "Every archetype in a world's archetype list must be an "
                "Archetype.");
  static_assert(is_unique_v<ArchetypeList>,
                "Each archetype may appear once in a world's archetype list.");
  static_assert(sizeof...(ArchetypeTypes) <
                    std::numeric_limits<std::uint8_t>::max(),
                "A world has fewer than 255 archetypes.");
  static_assert(
      [] {
        std::array<std::string_view, sizeof...(ArchetypeTypes)> names{
            ArchetypeTypes::name...};
        std::ranges::sort(names);
        return std::ranges::adjacent_find(names) == names.end();
      }(),
      "Each archetype in a world's archetype list needs its own name.");
  static_assert((is_subset_v<typename ArchetypeTypes::PermittedComponentList,
                             ComponentList> &&
                 ...),
                "An archetype requires or allows a component that is not in "
                "the world's component list.");

  DECLARE_COPY_DELETE(World);
  DECLARE_MOVE_DELETE(World);

  World() { initialize(Configuration{}); }
  ~World() = default;

  // Starts the utterance that builds a world of this type. See SetUpBuilder.
  static auto set_up() { return SetUpBuilder<World>{}; }

  //-- Write -------------------------------------------------------------------

  // Creates an entity of `ArchetypeType`, with an optional alias.
  template <Archetypal ArchetypeType>
  auto create(Alias alias = {}) {
    static_assert(contains_v<ArchetypeList, ArchetypeType>,
                  "This archetype is not in the world's archetype list.");
    return CreateBuilder<World, ArchetypeType, true>{
        std::move(alias), std::nullopt, {}, Depend(*this)};
  }
  auto change(Entity entity) {
    return ChangeBuilder<World>{entity, {}, {}, Depend(*this)};
  }
  // Makes the same change to every entity a query selects. See
  // ChangeQueryBuilder.
  auto change() {
    return ChangeQueryBuilder<World, ReadAnything, void, TypeList<>, TypeList<>,
                              false>{Depend(*this)};
  }
  auto destroy(Entity entity) {
    return DestroyBuilder<World>{entity, Depend(*this)};
  }
  // Destroys every entity a query selects. See DestroyQueryBuilder.
  auto destroy() {
    return DestroyQueryBuilder<World, ReadAnything, void>{Depend(*this)};
  }

  // Applies every pending command, in the order it was recorded. Builders
  // validated each command against the state the world would be in, so this
  // cannot fail.
  auto sync() -> void {
    CHECK_PRECONDITION(transaction_depth_ == 0);  // Commit or roll back first.
    std::vector<Command> commands = std::exchange(commands_, {});
    for (Command& command : commands) {
      std::visit([&](auto& operation) { this->apply(InOut(operation)); },
                 command);
    }
    std::apply([](auto&... plan) { (plan.clear(), ...); }, plans_);
    for (std::uint32_t index : destroying_list_) {
      destroying_[index] = false;
    }
    destroying_list_.clear();
  }

  auto pending() const -> std::size_t { return commands_.size(); }

  // Reorders the entities of `ArchetypeType` in every store it requires by
  // `key(Entity)`, least first, ties keeping their order, so that systems
  // walking them read them in that order: vehicles in road order, say, so
  // that neighbors on the road are neighbors in memory. Slots change, so it
  // runs between steps, with nothing pending.
  template <Archetypal ArchetypeType, typename KeyType>
  auto reorder(KeyType&& key) -> void {
    CHECK_PRECONDITION(commands_.empty() && transaction_depth_ == 0);
    constexpr std::size_t ARCHETYPE = index_of_v<ArchetypeList, ArchetypeType>;
    using Key = std::invoke_result_t<KeyType&, Entity>;
    const auto& archetypes = std::get<ComponentStore<EntityArchetype>>(stores_);
    std::size_t segment = segment_of<EntityArchetype>(ARCHETYPE);
    std::vector<std::pair<Key, std::uint32_t>> keyed;
    keyed.reserve(archetypes.segment_size(segment));
    for (std::size_t ordinal = 0; ordinal < archetypes.chunks_in(segment);
         ++ordinal) {
      auto chunk = archetypes.chunk(segment, ordinal);
      for (std::size_t i = 0; i < chunk.size; ++i) {
        keyed.emplace_back(key(chunk.owners[i]),
                           static_cast<std::uint32_t>(keyed.size()));
      }
    }
    std::ranges::stable_sort(keyed, {}, &std::pair<Key, std::uint32_t>::first);
    std::vector<std::uint32_t> order;
    order.reserve(keyed.size());
    for (const auto& [_, local] : keyed) {
      order.push_back(local);
    }
    std::apply(
        [&]<typename... StoreTypes>(StoreTypes&... stores) {
          auto permute = [&]<typename ComponentType>(
                             ComponentStore<ComponentType>& store) {
            if constexpr (archetype_requires<ComponentType>(ARCHETYPE)) {
              store.permute(segment_of<ComponentType>(ARCHETYPE), order);
              if constexpr (std::is_same_v<ComponentType, SpatialType>) {
                spatial_index_current_ = false;
              }
            }
          };
          (permute(stores), ...);
        },
        stores_);
  }

  // Groups utterances so they take effect together or not at all. Until it
  // commits, everything they planned can be rolled back: entities reserved and
  // their names, aliases given and taken, planned attachments, detachments
  // and destructions, and their commands. A transaction that ends without
  // commit() rolls back, so an early return undoes the group:
  //
  //   auto transaction = world.transaction();
  //   RETURN_OR_ASSIGN(Entity asset, world.create<Asset>().build());
  //   RETURN_IF_UNEXPECTED(world.create<Radar>().under(asset).build());
  //   transaction.commit();
  //
  // Transactions nest: an inner commit keeps its work only if every
  // enclosing transaction commits too. End every transaction before sync().
  // Entity names are never reused, so a rolled-back entity leaves a gap in
  // them, and an alias taken and then restored goes to the end of its lists.
  class Transaction final {
   public:
    DECLARE_COPY_DELETE(Transaction);
    DECLARE_MOVE_DELETE(Transaction);
    ~Transaction() {
      if (open_) {
        roll_back();
      }
    }

    // Keeps what the transaction planned.
    auto commit() -> void {
      CHECK_PRECONDITION(open_);
      open_ = false;
      world_->end_transaction();
    }

    // Undoes what the transaction planned, newest first.
    auto roll_back() -> void {
      CHECK_PRECONDITION(open_);
      open_ = false;
      world_->roll_back_to(mark_);
      world_->end_transaction();
    }

   private:
    friend class World;

    // The transaction's start: the counts of commands and undo steps then.
    struct Mark final {
      std::size_t commands = 0;
      std::size_t undo = 0;
    };

    explicit Transaction(World& world)
        : world_{&world}, mark_{world.begin_transaction()} {}

    World* world_ = nullptr;
    Mark mark_;
    bool open_ = true;
  };

  // Opens a transaction. See Transaction.
  auto transaction() -> Transaction { return Transaction{*this}; }

  //-- Read: entities and components -----------------------------------------

  auto alive(Entity entity) const -> bool { return entities_.alive(entity); }
  auto size() const -> std::size_t { return entities_.size(); }

  template <typename ComponentType>
  auto store_of() const -> const ComponentStore<ComponentType>& {
    static_assert(contains_v<ComponentList, ComponentType>,
                  "This component is not in the world's component list.");
    return std::get<ComponentStore<ComponentType>>(stores_);
  }

  // Known as soon as the entity is created, before sync.
  auto archetype_of(Entity entity) const -> Name {
    CHECK_PRECONDITION(alive(entity));
    return archetype_names_[archetype_of_index_[entity.index]];
  }

  // The entity `entity` was created under, if any. It may no longer be alive.
  auto parent_of(Entity entity) const -> std::optional<Entity> {
    const Parent* parent = store_of<Parent>().maybe_component_of(entity);
    return parent ? std::optional{parent->entity} : std::nullopt;
  }

  // Visits every entity whose spatial entity-component is within `radius` of
  // `center`, as `visit(Entity, const SpatialType&)`.
  //
  // Spatial queries go through an index, rebuilt on the first query after the
  // spatial store changes, which is why they are not const.
  template <typename VisitorType>
  auto within(const SpatialType& center, distance_of_t<SpatialType> radius,
              VisitorType&& visit) -> void {
    const ComponentStore<SpatialType>& spatial = refresh_spatial_index();
    spatial_index_.within(
        coordinates(center), coordinate_length(center, radius),
        [&](std::uint32_t slot) {
          visit(spatial.owner_at(slot), spatial.component_at(slot));
        });
  }

  // The entity nearest `center`, within `radius`, for which
  // `accept(Entity, const SpatialType&)` is true. Ties go to the entity in the
  // lowest store slot.
  template <typename AcceptType>
  auto nearest(const SpatialType& center, distance_of_t<SpatialType> radius,
               AcceptType&& accept) -> std::optional<Entity> {
    const ComponentStore<SpatialType>& spatial = refresh_spatial_index();
    std::optional<std::uint32_t> slot = spatial_index_.nearest(
        coordinates(center), coordinate_length(center, radius),
        [&](std::uint32_t candidate) {
          return accept(spatial.owner_at(candidate),
                        spatial.component_at(candidate));
        });
    return slot ? std::optional{spatial.owner_at(*slot)} : std::nullopt;
  }

  //-- Read: names, identities and aliases -----------------------------------

  auto number() const -> std::uint32_t { return number_; }
  auto name() const -> Name { return Name{Kind::WORLD, number_}; }

  // An entity's name. Known as soon as the entity is created, before sync.
  auto name_of(Entity entity) const -> Name {
    CHECK_PRECONDITION(alive(entity));
    return Name{Kind::ENTITY, instance_of_index_[entity.index]};
  }

  template <typename ComponentType>
  static constexpr auto name_of() -> Name {
    return Name{Kind::COMPONENT, component_number<ComponentType>()};
  }

  // The name of `entity`'s `ComponentType`, which it need not have yet.
  template <typename ComponentType>
  auto name_of(Entity entity) const -> Name {
    return entity_component_name(component_number<ComponentType>(),
                                 name_of(entity).instance);
  }

  // The live entity an entity or entity-component name refers to.
  auto entity_of(Name name) const -> std::optional<Entity> {
    if (name.kind != static_cast<std::uint32_t>(Kind::ENTITY) &&
        !is_entity_component(name)) {
      return std::nullopt;
    }
    auto iter = entity_of_instance_.find(name.instance);
    return iter != entity_of_instance_.end() ? std::optional{iter->second}
                                             : std::nullopt;
  }

  auto format_identity(Name name) const -> Identity {
    return framework::format_identity(number_, name);
  }

  // The name an identity refers to, if it names something in this world.
  auto find_name_of(const Identity& identity) const -> std::optional<Name> {
    std::optional<ParsedIdentity> parsed = parse_identity(identity);
    if (!parsed || parsed->world != number_ || !exists(parsed->name)) {
      return std::nullopt;
    }
    return parsed->name;
  }

  // Every name with `alias`, in the order the aliases were given.
  auto find_name_of(const Alias& alias) const -> std::vector<Name> {
    std::vector<Name> names;
    auto [begin, end] = aliases_.equal_range(alias);
    for (auto iter = begin; iter != end; ++iter) {
      names.push_back(iter->second);
    }
    return names;
  }

  // Every alias `name` has, in the order they were given.
  auto aliases_of(Name name) const -> std::vector<Alias> {
    auto iter = aliases_of_name_.find(name);
    return iter != aliases_of_name_.end() ? iter->second : std::vector<Alias>{};
  }

  // A one-line description for a console, e.g.
  // "/world/1/entity/2 (red) archetype ball: Kinematics, Collider".
  auto describe(Name name) const -> std::string {
    std::string text = format_identity(name).string();
    std::vector<Alias> aliases = aliases_of(name);
    for (std::size_t i = 0; i < aliases.size(); ++i) {
      text += std::format("{}{}", i == 0 ? " (" : ", ", aliases[i]);
    }
    text += aliases.empty() ? "" : ")";
    if (std::optional<Entity> entity = entity_of(name);
        entity && name.kind == static_cast<std::uint32_t>(Kind::ENTITY)) {
      std::vector<Alias> archetype = aliases_of(archetype_of(*entity));
      text += std::format(" archetype {}:",
                          archetype.empty() ? Alias{"?"} : archetype.front());
      bool first = true;
      for_each_type(ComponentList{}, [&]<typename ComponentType>() {
        if (!is_built_in_v<ComponentType> &&
            store_of<ComponentType>().contains(*entity)) {
          std::vector<Alias> component = aliases_of(name_of<ComponentType>());
          text += std::format("{} {}", first ? "" : ",", component.back());
          first = false;
        }
      });
    }
    return text;
  }

  //-- Archetypes and segments, for the framework -----------------------------

  // Whether archetype number `archetype` (its position in ArchetypeList)
  // requires, or requires or allows, `ComponentType`. Every archetype
  // requires EntityArchetype and allows Parent.
  template <typename ComponentType>
  static constexpr auto archetype_requires(std::size_t archetype) -> bool {
    constexpr std::array<bool, sizeof...(ArchetypeTypes)> TABLE{
        (contains_v<typename ArchetypeTypes::RequiredComponentList,
                    ComponentType> ||
         std::is_same_v<ComponentType, EntityArchetype>)...};
    return TABLE[archetype];
  }
  template <typename ComponentType>
  static constexpr auto archetype_permits(std::size_t archetype) -> bool {
    constexpr std::array<bool, sizeof...(ArchetypeTypes)> TABLE{
        (contains_v<typename ArchetypeTypes::PermittedComponentList,
                    ComponentType> ||
         is_built_in_v<ComponentType>)...};
    return TABLE[archetype];
  }

  // Each store has a segment per archetype that requires its component, in
  // archetype order, then one for entities whose archetype only allows it.

  template <typename ComponentType>
  static constexpr auto segments_of() -> std::size_t {
    std::size_t required = 0;
    for (std::size_t archetype = 0; archetype < sizeof...(ArchetypeTypes);
         ++archetype) {
      required += archetype_requires<ComponentType>(archetype);
    }
    return required + 1;
  }

  // The segment an entity of archetype number `archetype` uses in the store
  // of `ComponentType`.
  template <typename ComponentType>
  static constexpr auto segment_of(std::size_t archetype) -> std::size_t {
    if (!archetype_requires<ComponentType>(archetype)) {
      return segments_of<ComponentType>() - 1;
    }
    std::size_t segment = 0;
    for (std::size_t before = 0; before < archetype; ++before) {
      segment += archetype_requires<ComponentType>(before);
    }
    return segment;
  }

  //-- Systems ----------------------------------------------------------------

  // A store a system writes. Handing out the spatial store marks the spatial
  // index stale, since the system may move things.
  template <typename ComponentType>
  auto mutable_store_of(SchedulerKey) -> ComponentStore<ComponentType>& {
    static_assert(contains_v<ComponentList, ComponentType>,
                  "This component is not in the world's component list.");
    if constexpr (std::is_same_v<ComponentType, SpatialType>) {
      spatial_index_current_ = false;
    }
    return std::get<ComponentStore<ComponentType>>(stores_);
  }

 private:
  friend class SetUpBuilder<World>;

  // A world's size, worked out by SetUpBuilder from its count of each
  // archetype.
  struct Configuration final {
    std::uint32_t number = 0;  // The world's instance in its Name and Identity.
    std::size_t entities = 0;  // Entity capacity.
    // Each store's capacity, by component number.
    std::array<std::size_t, ComponentList::size> capacities{};
    // The edge of a spatial index cell, in the spatial component's coordinate
    // unit, or none to size cells at every rebuild.
    std::optional<double> cell_size;
  };

  // Fills the world as `configuration` describes, discarding everything it
  // held: entities, names, aliases and pending commands. Pointers into its
  // stores no longer refer to anything.
  auto initialize(const Configuration& configuration) -> void {
    CHECK_PRECONDITION(transaction_depth_ == 0);
    number_ = configuration.number;
    entities_ = EntityTable{configuration.entities};
    stores_ = {store_for<SpatialType>(configuration),
               store_for<EntityArchetype>(configuration),
               store_for<Parent>(configuration),
               store_for<ComponentTypes>(configuration)...};
    commands_.clear();
    plans_ = {Plan<SpatialType>{configuration.entities},
              Plan<EntityArchetype>{configuration.entities},
              Plan<Parent>{configuration.entities},
              Plan<ComponentTypes>{configuration.entities}...};
    std::size_t points =
        configuration.capacities[component_number<SpatialType>()];
    spatial_index_ = configuration.cell_size
                         ? SpatialIndex{points, *configuration.cell_size}
                         : SpatialIndex{points};
    spatial_index_current_ = false;
    destroying_.assign(configuration.entities, false);
    destroying_list_.clear();
    next_entity_instance_ = 0;
    instance_of_index_.assign(configuration.entities, 0);
    archetype_of_index_.assign(configuration.entities, 0);
    entity_of_instance_.clear();
    next_archetype_instance_ = 0;
    archetypes_.clear();
    archetype_names_ = {};
    aliases_.clear();
    aliases_of_name_.clear();
    // Components are aliased by their type names, qualified and short.
    for_each_type(ComponentList{}, [&]<typename ComponentType>() {
      std::string type_name = lib::to_type_string<ComponentType>();
      Name name = name_of<ComponentType>();
      give_alias(name, Alias{type_name});
      if (std::size_t colons = type_name.rfind("::");
          colons != std::string::npos) {
        give_alias(name, Alias{type_name.substr(colons + 2)});
      }
    });
  }

  template <typename, Archetypal, bool, typename...>
  friend class CreateBuilder;
  template <typename, typename, typename, bool>
  friend class ChangeBuilder;
  template <typename>
  friend class DestroyBuilder;
  template <typename, typename>
  friend class Query;

  template <typename ComponentType>
  static constexpr auto component_number() -> std::uint32_t {
    static_assert(contains_v<ComponentList, ComponentType>,
                  "This component is not in the world's component list.");
    return static_cast<std::uint32_t>(index_of_v<ComponentList, ComponentType>);
  }

  auto exists(Name name) const -> bool {
    switch (static_cast<Kind>(std::min(
        name.kind, static_cast<std::uint32_t>(Kind::ENTITY_COMPONENT)))) {
      case Kind::WORLD:
        return name.instance == number_;
      case Kind::ARCHETYPE:
        return name.instance < next_archetype_instance_;
      case Kind::COMPONENT:
        return name.instance < ComponentList::size;
      case Kind::SYSTEM:
        // Systems belong to schedulers, which the world does not see.
        return true;
      case Kind::ENTITY:
        return entity_of(name).has_value();
      case Kind::ENTITY_COMPONENT:
        return component_of(name) < ComponentList::size &&
               entity_of(name).has_value() &&
               has_component_number(*entity_of(name), component_of(name));
      case Kind::NONE:
        break;
    }
    return false;
  }

  // The spatial store, after bringing the index up to date with it. The
  // index's slots are positions in this store.
  auto refresh_spatial_index() -> const ComponentStore<SpatialType>& {
    const ComponentStore<SpatialType>& spatial = store_of<SpatialType>();
    if (!spatial_index_current_) {
      spatial_index_.rebuild([&](auto&& insert) {
        spatial.for_each_slot(
            [&](std::uint32_t slot, Entity, const SpatialType& component) {
              insert(slot, coordinates(component));
            });
      });
      spatial_index_current_ = true;
    }
    return spatial;
  }

  // Every store of a world uses the same chunk size, so an archetype's
  // segments line up chunk for chunk.
  static auto chunk_size_of(const Configuration& configuration) -> std::size_t {
    return ComponentStore<EntityArchetype>::default_chunk_size(
        configuration.entities);
  }

  template <typename ComponentType>
  static auto store_for(const Configuration& configuration)
      -> ComponentStore<ComponentType> {
    return ComponentStore<ComponentType>{
        configuration.capacities[component_number<ComponentType>()],
        configuration.entities, segments_of<ComponentType>(),
        chunk_size_of(configuration)};
  }

  auto has_component_number(Entity entity, std::uint32_t number) const -> bool {
    bool found = false;
    for_each_type(ComponentList{}, [&]<typename ComponentType>() {
      found = found || (component_number<ComponentType>() == number &&
                        store_of<ComponentType>().contains(entity));
    });
    return found;
  }

  template <Archetypal ArchetypeType>
  auto archetype_name() -> Name {
    auto [iter, inserted] =
        archetypes_.try_emplace(std::string{ArchetypeType::name});
    if (inserted) {
      iter->second = Name{Kind::ARCHETYPE, next_archetype_instance_++};
      give_alias(iter->second, ArchetypeType::name);
      archetype_names_[index_of_v<ArchetypeList, ArchetypeType>] = iter->second;
    }
    return iter->second;
  }

  auto give_alias(Name name, const Alias& alias) -> void {
    aliases_.emplace(alias, name);
    aliases_of_name_[name].push_back(alias);
  }

  auto take_alias(Name name, const Alias& alias) -> void {
    auto [begin, end] = aliases_.equal_range(alias);
    for (auto iter = begin; iter != end; ++iter) {
      if (iter->second == name) {
        aliases_.erase(iter);
        break;
      }
    }
    std::vector<Alias>& given = aliases_of_name_[name];
    std::erase(given, alias);
    if (given.empty()) {
      aliases_of_name_.erase(name);
    }
  }

  auto has_alias(Name name, const Alias& alias) const -> bool {
    auto iter = aliases_of_name_.find(name);
    return iter != aliases_of_name_.end() &&
           std::ranges::contains(iter->second, alias);
  }

  //-- Transactions ------------------------------------------------------------

  auto begin_transaction() -> typename Transaction::Mark {
    ++transaction_depth_;
    return {.commands = commands_.size(), .undo = undo_.size()};
  }

  auto roll_back_to(const typename Transaction::Mark& mark) -> void {
    while (undo_.size() > mark.undo) {
      undo_.back()();
      undo_.pop_back();
    }
    commands_.erase(
        commands_.begin() + static_cast<std::ptrdiff_t>(mark.commands),
        commands_.end());
  }

  // Forgets the undo steps once the outermost transaction ends.
  auto end_transaction() -> void {
    CHECK_PRECONDITION(transaction_depth_ > 0);
    if (--transaction_depth_ == 0) {
      undo_.clear();
    }
  }

  // Records how to undo a planned change, while a transaction is open.
  template <typename UndoType>
  auto remember(UndoType&& undo) -> void {
    if (transaction_depth_ > 0) {
      undo_.emplace_back(std::forward<UndoType>(undo));
    }
  }

  // Records the plan for `ComponentType` at `index` before it changes.
  template <typename ComponentType>
  auto remember_plan(std::uint32_t index) -> void {
    if (transaction_depth_ == 0) {
      return;
    }
    Plan<ComponentType>& planned = plan<ComponentType>();
    remember([this, index, change = planned.change[index],
              touched = planned.touched.size(), growth = planned.growth] {
      Plan<ComponentType>& restored = plan<ComponentType>();
      restored.change[index] = change;
      restored.touched.resize(touched);
      restored.growth = growth;
    });
  }

  //-- Planned state: what the world will be once pending commands apply -------

  // For one component: which entities will gain or lose it, and how much the
  // store will have grown once pending commands apply. Commands apply in the
  // order they were recorded, so the growth after the last of them bounds
  // the store when one more is appended: attaches add one, and detaches and
  // destroys of an entity that will have the component take one away.
  struct PlanState {
    explicit PlanState(std::size_t entity_capacity = 0)
        : change(entity_capacity, 0) {}

    auto mark(std::uint32_t index, std::int8_t value) -> void {
      if (change[index] == 0) {
        touched.push_back(index);
      }
      change[index] = value;
    }
    auto clear() -> void {
      for (std::uint32_t index : touched) {
        change[index] = 0;
      }
      touched.clear();
      growth = 0;
    }

    std::vector<std::int8_t>
        change;  // +1 will be attached, -1 will be detached.
    std::vector<std::uint32_t> touched;
    std::int64_t growth = 0;
  };

  template <typename ComponentType>
  struct Plan final : PlanState {
    using PlanState::PlanState;
  };

  template <typename ComponentType>
  auto plan() -> Plan<ComponentType>& {
    return std::get<Plan<ComponentType>>(plans_);
  }

  auto will_be_alive(Entity entity) const -> bool {
    return alive(entity) && !destroying_[entity.index];
  }

  template <typename ComponentType>
  auto will_have(Entity entity) const -> bool {
    std::int8_t change =
        std::get<Plan<ComponentType>>(plans_).change[entity.index];
    return change != 0 ? change > 0
                       : store_of<ComponentType>().contains(entity);
  }

  template <typename ComponentType>
  auto has_room() const -> bool {
    const ComponentStore<ComponentType>& components = store_of<ComponentType>();
    return static_cast<std::int64_t>(components.size()) +
               std::get<Plan<ComponentType>>(plans_).growth <
           static_cast<std::int64_t>(components.capacity());
  }

  template <typename... CheckedTypes>
  auto check_room() const -> std::optional<Status> {
    std::optional<Status> failure;
    [[maybe_unused]] auto check = [&]<typename ComponentType>() {
      if (!failure && !has_room<ComponentType>()) {
        failure = lib::raise(BuildError::COMPONENT_CAPACITY_EXHAUSTED,
                             std::format("The {} store is full.",
                                         lib::to_type_string<ComponentType>()));
      }
    };
    (check.template operator()<CheckedTypes>(), ...);
    return failure;
  }

  static auto check_alias(const Alias& alias) -> std::optional<Status> {
    if (alias.empty()) {
      return lib::raise(BuildError::ALIAS_INVALID, "An alias cannot be empty.");
    }
    return std::nullopt;
  }

  //-- Builders ----------------------------------------------------------------

  template <Archetypal ArchetypeType, typename... InitialTypes>
  auto build_create(const Alias& alias, std::optional<Entity> parent,
                    std::tuple<InitialTypes...> components)
      -> std::expected<Entity, Status> {
    if (entities_.size() == entities_.capacity()) {
      return std::unexpected(
          lib::raise(BuildError::ENTITY_CAPACITY_EXHAUSTED,
                     "The world is out of entity capacity."));
    }
    if (parent && !will_be_alive(*parent)) {
      return std::unexpected(lib::raise(BuildError::ENTITY_NOT_ALIVE,
                                        "The parent entity is not alive."));
    }
    if (std::optional<Status> failure = check_room<InitialTypes...>()) {
      return std::unexpected(*failure);
    }
    CHECK_PRECONDITION(next_entity_instance_ <
                       std::numeric_limits<std::uint32_t>::max());

    Entity entity = entities_.create();
    std::uint32_t instance = next_entity_instance_++;
    instance_of_index_[entity.index] = instance;
    archetype_of_index_[entity.index] =
        static_cast<std::uint8_t>(index_of_v<ArchetypeList, ArchetypeType>);
    entity_of_instance_.emplace(instance, entity);
    remember([this, entity, instance] {
      entity_of_instance_.erase(instance);
      entities_.destroy(entity);
    });
    if (!alias.empty()) {
      give_alias(Name{Kind::ENTITY, instance}, alias);
      remember([this, instance, alias] {
        take_alias(Name{Kind::ENTITY, instance}, alias);
      });
    }

    record_attach(
        entity, EntityArchetype{.archetype = archetype_name<ArchetypeType>()});
    if (parent) {
      record_attach(entity, Parent{.entity = *parent});
    }
    std::apply(
        [&](auto&... component) {
          (record_attach(entity, std::move(component)), ...);
        },
        components);
    return entity;
  }

  template <typename... AttachedTypes, typename... DetachedTypes>
  auto build_change(Entity entity, std::tuple<AttachedTypes...> components,
                    TypeList<DetachedTypes...>, const AliasChanges& aliases)
      -> std::expected<void, Status> {
    if (!will_be_alive(entity)) {
      return std::unexpected(
          lib::raise(BuildError::ENTITY_NOT_ALIVE, "The entity is not alive."));
    }
    std::optional<Status> failure;
    std::uint8_t archetype = archetype_of_index_[entity.index];
    [[maybe_unused]] auto permit = [&]<typename ComponentType>(bool attaching) {
      if (failure) {
        return;
      }
      std::string type_name = lib::to_type_string<ComponentType>();
      if (attaching && !archetype_permits<ComponentType>(archetype)) {
        failure = lib::raise(
            BuildError::COMPONENT_NOT_PERMITTED,
            std::format("The entity's archetype neither requires nor allows "
                        "a {}.",
                        type_name));
      } else if (!attaching && archetype_requires<ComponentType>(archetype)) {
        failure = lib::raise(
            BuildError::COMPONENT_REQUIRED,
            std::format("The entity's archetype requires a {}.", type_name));
      }
    };
    (permit.template operator()<AttachedTypes>(true), ...);
    (permit.template operator()<DetachedTypes>(false), ...);
    [[maybe_unused]] auto require = [&]<typename ComponentType>(
                                        bool present, BuildError condition,
                                        const char* message) {
      if (!failure && will_have<ComponentType>(entity) != present) {
        std::string type_name = lib::to_type_string<ComponentType>();
        failure = lib::raise(
            condition, std::vformat(message, std::make_format_args(type_name)));
      }
    };
    (require.template operator()<AttachedTypes>(
         false, BuildError::COMPONENT_ALREADY_ATTACHED,
         "The entity already has a {}."),
     ...);
    (require.template operator()<DetachedTypes>(
         true, BuildError::COMPONENT_NOT_ATTACHED, "The entity has no {}."),
     ...);
    if (!failure) {
      failure = check_room<AttachedTypes...>();
    }

    Name name = name_of(entity);
    for (std::size_t i = 0; !failure && i < aliases.given.size(); ++i) {
      const Alias& alias = aliases.given[i];
      failure = check_alias(alias);
      if (!failure && (has_alias(name, alias) ||
                       std::count(aliases.given.begin(),
                                  aliases.given.begin() + i, alias))) {
        failure = lib::raise(
            BuildError::ALIAS_ALREADY_GIVEN,
            std::format("The entity already has the alias {}.", alias));
      }
    }
    for (std::size_t i = 0; !failure && i < aliases.taken.size(); ++i) {
      const Alias& alias = aliases.taken[i];
      if (!has_alias(name, alias) ||
          std::count(aliases.taken.begin(), aliases.taken.begin() + i, alias)) {
        failure = lib::raise(
            BuildError::ALIAS_NOT_GIVEN,
            std::format("The entity does not have the alias {}.", alias));
      }
    }
    if (failure) {
      return std::unexpected(*failure);
    }

    std::apply(
        [&](auto&... component) {
          (record_attach(entity, std::move(component)), ...);
        },
        components);
    (record_detach<DetachedTypes>(entity), ...);
    // Aliases are an index beside the stores, not store shape, so they change
    // immediately.
    for (const Alias& alias : aliases.taken) {
      take_alias(name, alias);
      remember([this, name, alias] { give_alias(name, alias); });
    }
    for (const Alias& alias : aliases.given) {
      give_alias(name, alias);
      remember([this, name, alias] { take_alias(name, alias); });
    }
    return {};
  }

  auto build_destroy(Entity entity) -> std::expected<void, Status> {
    if (!will_be_alive(entity)) {
      return std::unexpected(
          lib::raise(BuildError::ENTITY_NOT_ALIVE, "The entity is not alive."));
    }
    // Every component the entity will have makes room when it is destroyed.
    for_each_type(ComponentList{}, [&]<typename ComponentType>() {
      if (will_have<ComponentType>(entity)) {
        remember_plan<ComponentType>(entity.index);
        --plan<ComponentType>().growth;
      }
    });
    destroying_[entity.index] = true;
    destroying_list_.push_back(entity.index);
    remember([this, index = entity.index] {
      destroying_[index] = false;
      destroying_list_.pop_back();
    });
    commands_.push_back(DestroyCommand{entity});
    return {};
  }

  // The entity's archetype must already be recorded, since it picks the
  // segment.
  template <typename ComponentType>
  auto record_attach(Entity entity, ComponentType component) -> void {
    remember_plan<ComponentType>(entity.index);
    plan<ComponentType>().mark(entity.index, +1);
    ++plan<ComponentType>().growth;
    commands_.push_back(AttachCommand<ComponentType>{
        entity, std::move(component),
        segment_of<ComponentType>(archetype_of_index_[entity.index])});
  }

  template <typename ComponentType>
  auto record_detach(Entity entity) -> void {
    remember_plan<ComponentType>(entity.index);
    plan<ComponentType>().mark(entity.index, -1);
    --plan<ComponentType>().growth;
    commands_.push_back(DetachCommand<ComponentType>{entity});
  }

  //-- Applying commands -------------------------------------------------------

  template <typename ComponentType>
  auto apply(InOut<AttachCommand<ComponentType>> command) -> void {
    CHECK_INVARIANT(alive(command->entity));
    spatial_index_current_ =
        spatial_index_current_ && !std::is_same_v<ComponentType, SpatialType>;
    std::get<ComponentStore<ComponentType>>(stores_).append(
        command->entity, std::move(command->component), command->segment);
  }

  template <typename ComponentType>
  auto apply(InOut<DetachCommand<ComponentType>> command) -> void {
    CHECK_INVARIANT(alive(command->entity));
    spatial_index_current_ =
        spatial_index_current_ && !std::is_same_v<ComponentType, SpatialType>;
    std::get<ComponentStore<ComponentType>>(stores_).erase(command->entity);
  }

  auto apply(InOut<DestroyCommand> command) -> void {
    Entity entity = command->entity;
    CHECK_INVARIANT(alive(entity));
    Name name = name_of(entity);
    for (const Alias& alias : aliases_of(name)) {
      take_alias(name, alias);
    }
    entity_of_instance_.erase(name.instance);
    spatial_index_current_ =
        spatial_index_current_ && !store_of<SpatialType>().contains(entity);
    std::apply(
        [entity](auto&... store) {
          ((store.contains(entity) ? store.erase(entity) : void()), ...);
        },
        stores_);
    entities_.destroy(entity);
  }

  // Each is filled by initialize.
  std::uint32_t number_ = 0;
  EntityTable entities_{0};
  std::tuple<ComponentStore<SpatialType>, ComponentStore<EntityArchetype>,
             ComponentStore<Parent>, ComponentStore<ComponentTypes>...>
      stores_;
  std::vector<Command> commands_;
  std::tuple<Plan<SpatialType>, Plan<EntityArchetype>, Plan<Parent>,
             Plan<ComponentTypes>...>
      plans_;
  // Current while its slots match the spatial store. See within().
  SpatialIndex spatial_index_{0};
  bool spatial_index_current_ = false;
  std::vector<bool> destroying_;
  std::vector<std::uint32_t> destroying_list_;

  // The undo steps for open transactions' plans, oldest first.
  std::vector<std::function<void()>> undo_;
  std::size_t transaction_depth_ = 0;

  // Names. Entity instances are never reused; Entity indices are.
  std::uint32_t next_entity_instance_ = 0;
  std::vector<std::uint32_t> instance_of_index_;
  // Each live entity's archetype, as its position in ArchetypeList.
  std::vector<std::uint8_t> archetype_of_index_;
  std::unordered_map<std::uint32_t, Entity> entity_of_instance_;
  std::uint32_t next_archetype_instance_ = 0;
  std::map<std::string, Name, std::less<>> archetypes_;
  // Each archetype's Name, by its position in ArchetypeList, once created.
  std::array<Name, sizeof...(ArchetypeTypes)> archetype_names_{};

  // Aliases, many-to-many. A multimap keeps equal aliases in the order given.
  std::multimap<Alias, Name> aliases_;
  std::unordered_map<Name, std::vector<Alias>> aliases_of_name_;
};

}  // namespace simon::framework

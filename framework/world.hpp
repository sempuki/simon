// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

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
#include "framework/builder.hpp"
#include "framework/command.hpp"
#include "framework/component_store.hpp"
#include "framework/entity.hpp"
#include "framework/name.hpp"
#include "framework/spatial_index.hpp"
#include "framework/type_list.hpp"

namespace simon::framework {

// What a world needs to know about space: a distance, a pose, and, for its
// spatial index, plain coordinates. The world indexes every entity-component
// of its spatial type. Distances may carry units; `coordinate_length` gives
// one as a plain number in the same unit as `coordinates`.
template <typename Type>
concept Spatial = requires(const Type& a, const Type& b) {
  { coordinates(a) } -> std::same_as<Coordinates>;
  { coordinate_length(a, distance(a, b)) } -> std::same_as<double>;
  { distance(a, b) < distance(a, b) } -> std::convertible_to<bool>;
  pose(a);
};

template <Spatial Type>
using distance_of_t = decltype(distance(std::declval<const Type&>(),
                                        std::declval<const Type&>()));

template <typename WorldType>
class SetUpBuilder;

template <typename WorldType, typename ChosenType>
class Query;
template <typename WorldType, typename ChosenType>
class DestroyQueryBuilder;
template <typename WorldType, typename ChosenType, typename AttachedListType,
          typename DetachedListType, bool Aliasing>
class ChangeQueryBuilder;

// Unlocks mutable store access. Only the scheduler, which runs systems, can
// construct one.
class SchedulerKey final {
 private:
  SchedulerKey() = default;
  friend struct SystemRunner;
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
//       .holding<archetype::Drone>(1000).build(lib::Out(world));
//
// A world never moves. Builders and WorldAccess keep a pointer to it, and its
// stores never reallocate, so nothing that refers to a world can dangle while
// it lives.
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
  using ArchetypeList = TypeList<  //
      ArchetypeTypes...>;
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

  // An empty world, which holds nothing until a builder builds it.
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
        std::move(alias), std::nullopt, {}, lib::Depend(*this)};
  }
  auto change(Entity entity) {
    return ChangeBuilder<World>{entity, {}, {}, lib::Depend(*this)};
  }
  // Makes the same change to every entity a query selects. See
  // ChangeQueryBuilder.
  auto change() {
    return ChangeQueryBuilder<World, void, TypeList<>, TypeList<>, false>{
        lib::Depend(*this)};
  }
  auto destroy(Entity entity) {
    return DestroyBuilder<World>{entity, lib::Depend(*this)};
  }
  // Destroys every entity a query selects. See DestroyQueryBuilder.
  auto destroy() {
    return DestroyQueryBuilder<World, void>{lib::Depend(*this)};
  }

  // Applies every pending command, in the order it was recorded. Builders
  // validated each command against the state the world would be in, so this
  // cannot fail.
  void sync() {
    CHECK_PRECONDITION(transaction_depth_ == 0);  // Commit or roll back first.
    std::vector<Command> commands = std::exchange(commands_, {});
    for (Command& command : commands) {
      std::visit([&](auto& operation) { this->apply(lib::InOut(operation)); },
                 command);
    }
    std::apply([](auto&... plan) { (plan.clear(), ...); }, plans_);
    for (std::uint32_t index : destroying_list_) {
      destroying_[index] = false;
    }
    destroying_list_.clear();
  }

  std::size_t pending() const { return commands_.size(); }

  // Groups utterances so they take effect together or not at all. Until it
  // commits, everything they planned can be rolled back: entities reserved and
  // their names, aliases given and taken, planned attachments, detachments
  // and destructions, and their commands. A transaction that ends without
  // commit() rolls back, so an early return undoes the group:
  //
  //   auto transaction = world.transaction();
  //   ASSIGN_OR_RETURN(Entity asset, world.create<Asset>().build());
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
    void commit() {
      CHECK_PRECONDITION(open_);
      open_ = false;
      world_->end_transaction();
    }

    // Undoes what the transaction planned, newest first.
    void roll_back() {
      CHECK_PRECONDITION(open_);
      open_ = false;
      world_->roll_back_to(mark_);
      world_->end_transaction();
    }

   private:
    friend class World;

    // Where the transaction began: how many commands and undo steps there
    // were.
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
  Transaction transaction() { return Transaction{*this}; }

  //-- Read: entities and components -----------------------------------------

  bool alive(Entity entity) const { return entities_.alive(entity); }
  std::size_t size() const { return entities_.size(); }

  template <typename ComponentType>
  const ComponentStore<ComponentType>& store_of() const {
    static_assert(contains_v<ComponentList, ComponentType>,
                  "This component is not in the world's component list.");
    return std::get<ComponentStore<ComponentType>>(stores_);
  }

  // Known as soon as the entity is created, before sync.
  Name archetype_of(Entity entity) const {
    CHECK_PRECONDITION(alive(entity));
    return archetype_names_[archetype_of_index_[entity.index]];
  }

  // The entity `entity` was created under, if any. It may no longer be alive.
  std::optional<Entity> parent_of(Entity entity) const {
    const Parent* parent = store_of<Parent>().try_component_of(entity);
    return parent ? std::optional{parent->entity} : std::nullopt;
  }

  // Visits every entity whose spatial entity-component is within `radius` of
  // `center`, as `visit(Entity, const SpatialType&)`.
  //
  // Spatial queries go through an index, rebuilt on the first query after the
  // spatial store changes, which is why they are not const.
  template <typename VisitorType>
  void within(const SpatialType& center, distance_of_t<SpatialType> radius,
              VisitorType&& visit) {
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
  std::optional<Entity> nearest(const SpatialType& center,
                                distance_of_t<SpatialType> radius,
                                AcceptType&& accept) {
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

  std::uint32_t number() const { return number_; }
  Name name() const { return Name{Kind::WORLD, number_}; }

  // An entity's name. Known as soon as the entity is created, before sync.
  Name name_of(Entity entity) const {
    CHECK_PRECONDITION(alive(entity));
    return Name{Kind::ENTITY, instance_of_index_[entity.index]};
  }

  template <typename ComponentType>
  static constexpr Name name_of() {
    return Name{Kind::COMPONENT, component_number<ComponentType>()};
  }

  // The name of `entity`'s `ComponentType`, which it need not have yet.
  template <typename ComponentType>
  Name name_of(Entity entity) const {
    return entity_component_name(component_number<ComponentType>(),
                                 name_of(entity).instance);
  }

  // The live entity an entity or entity-component name refers to.
  std::optional<Entity> entity_of(Name name) const {
    if (name.kind != static_cast<std::uint32_t>(Kind::ENTITY) &&
        !is_entity_component(name)) {
      return std::nullopt;
    }
    auto iter = entity_of_instance_.find(name.instance);
    return iter != entity_of_instance_.end() ? std::optional{iter->second}
                                             : std::nullopt;
  }

  Identity identity_of(Name name) const {
    return framework::identity_of(number_, name);
  }

  // The name an identity refers to, if it names something in this world.
  std::optional<Name> find_name_of(const Identity& identity) const {
    std::optional<ParsedIdentity> parsed = parse_identity(identity);
    if (!parsed || parsed->world != number_ || !exists(parsed->name)) {
      return std::nullopt;
    }
    return parsed->name;
  }

  // Every name with `alias`, in the order the aliases were given.
  std::vector<Name> find_name_of(const Alias& alias) const {
    std::vector<Name> names;
    auto [begin, end] = aliases_.equal_range(alias);
    for (auto iter = begin; iter != end; ++iter) {
      names.push_back(iter->second);
    }
    return names;
  }

  // Every alias `name` has, in the order they were given.
  std::vector<Alias> aliases_of(Name name) const {
    auto iter = aliases_of_name_.find(name);
    return iter != aliases_of_name_.end() ? iter->second : std::vector<Alias>{};
  }

  // A one-line description for a console, e.g.
  // "/world/1/entity/2 (red) archetype ball: Kinematics, Collider".
  std::string describe(Name name) const {
    std::string text = identity_of(name).string();
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
  static constexpr bool archetype_requires(std::size_t archetype) {
    constexpr std::array<bool, sizeof...(ArchetypeTypes)> TABLE{
        (contains_v<typename ArchetypeTypes::RequiredComponentList,
                    ComponentType> ||
         std::is_same_v<ComponentType, EntityArchetype>)...};
    return TABLE[archetype];
  }
  template <typename ComponentType>
  static constexpr bool archetype_permits(std::size_t archetype) {
    constexpr std::array<bool, sizeof...(ArchetypeTypes)> TABLE{
        (contains_v<typename ArchetypeTypes::PermittedComponentList,
                    ComponentType> ||
         is_built_in_v<ComponentType>)...};
    return TABLE[archetype];
  }

  // Each store has a segment per archetype that requires its component, in
  // archetype order, then one for entities whose archetype only allows it.

  template <typename ComponentType>
  static constexpr std::size_t segments_of() {
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
  static constexpr std::size_t segment_of(std::size_t archetype) {
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
  ComponentStore<ComponentType>& mutable_store_of(SchedulerKey) {
    static_assert(contains_v<ComponentList, ComponentType>,
                  "This component is not in the world's component list.");
    if constexpr (std::is_same_v<ComponentType, SpatialType>) {
      spatial_index_current_ = false;
    }
    return std::get<ComponentStore<ComponentType>>(stores_);
  }

 private:
  friend class SetUpBuilder<World>;

  // How big a world is, worked out by SetUpBuilder from how many of each
  // archetype it holds.
  struct Configuration final {
    std::uint32_t number = 0;  // The world's instance in its Name and Identity.
    std::size_t entities = 0;  // Entity capacity.
    // Each store's capacity, by component number.
    std::array<std::size_t, ComponentList::size> capacities{};
    // The edge of a spatial index cell, in the spatial component's coordinate
    // unit.
    double cell_size = 1.0;
  };

  // Fills the world as `configuration` describes, discarding everything it
  // held: entities, names, aliases and pending commands. Pointers into its
  // stores no longer refer to anything.
  void initialize(const Configuration& configuration) {
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
    spatial_index_ =
        SpatialIndex{configuration.capacities[component_number<SpatialType>()],
                     configuration.cell_size};
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
  static constexpr std::uint32_t component_number() {
    static_assert(contains_v<ComponentList, ComponentType>,
                  "This component is not in the world's component list.");
    return static_cast<std::uint32_t>(index_of_v<ComponentList, ComponentType>);
  }

  bool exists(Name name) const {
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
  const ComponentStore<SpatialType>& refresh_spatial_index() {
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
  static std::size_t chunk_size_of(const Configuration& configuration) {
    return ComponentStore<EntityArchetype>::default_chunk_size(
        configuration.entities);
  }

  template <typename ComponentType>
  static ComponentStore<ComponentType> store_for(
      const Configuration& configuration) {
    return ComponentStore<ComponentType>{
        configuration.capacities[component_number<ComponentType>()],
        configuration.entities, segments_of<ComponentType>(),
        chunk_size_of(configuration)};
  }

  bool has_component_number(Entity entity, std::uint32_t number) const {
    bool found = false;
    for_each_type(ComponentList{}, [&]<typename ComponentType>() {
      found = found || (component_number<ComponentType>() == number &&
                        store_of<ComponentType>().contains(entity));
    });
    return found;
  }

  template <Archetypal ArchetypeType>
  Name archetype_name() {
    auto [iter, inserted] =
        archetypes_.try_emplace(std::string{ArchetypeType::name});
    if (inserted) {
      iter->second = Name{Kind::ARCHETYPE, next_archetype_instance_++};
      give_alias(iter->second, ArchetypeType::name);
      archetype_names_[index_of_v<ArchetypeList, ArchetypeType>] = iter->second;
    }
    return iter->second;
  }

  void give_alias(Name name, const Alias& alias) {
    aliases_.emplace(alias, name);
    aliases_of_name_[name].push_back(alias);
  }

  void take_alias(Name name, const Alias& alias) {
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

  bool has_alias(Name name, const Alias& alias) const {
    auto iter = aliases_of_name_.find(name);
    return iter != aliases_of_name_.end() &&
           std::ranges::contains(iter->second, alias);
  }

  //-- Transactions ------------------------------------------------------------

  typename Transaction::Mark begin_transaction() {
    ++transaction_depth_;
    return {.commands = commands_.size(), .undo = undo_.size()};
  }

  void roll_back_to(const typename Transaction::Mark& mark) {
    while (undo_.size() > mark.undo) {
      undo_.back()();
      undo_.pop_back();
    }
    commands_.erase(
        commands_.begin() + static_cast<std::ptrdiff_t>(mark.commands),
        commands_.end());
  }

  // Forgets the undo steps once the outermost transaction ends.
  void end_transaction() {
    CHECK_PRECONDITION(transaction_depth_ > 0);
    if (--transaction_depth_ == 0) {
      undo_.clear();
    }
  }

  // Records how to undo a planned change, while a transaction is open.
  template <typename UndoType>
  void remember(UndoType&& undo) {
    if (transaction_depth_ > 0) {
      undo_.emplace_back(std::forward<UndoType>(undo));
    }
  }

  // Records the plan for `ComponentType` at `index` before it changes.
  template <typename ComponentType>
  void remember_plan(std::uint32_t index) {
    if (transaction_depth_ == 0) {
      return;
    }
    Plan<ComponentType>& planned = plan<ComponentType>();
    remember([this, index, change = planned.change[index],
              touched = planned.touched.size(), attaching = planned.attaching] {
      Plan<ComponentType>& restored = plan<ComponentType>();
      restored.change[index] = change;
      restored.touched.resize(touched);
      restored.attaching = attaching;
    });
  }

  //-- Planned state: what the world will be once pending commands apply -------

  // For one component: which entities will gain or lose it, and how many
  // entity-components are waiting to be attached.
  struct PlanState {
    explicit PlanState(std::size_t entity_capacity = 0)
        : change(entity_capacity, 0) {}

    std::vector<std::int8_t>
        change;  // +1 will be attached, -1 will be detached.
    std::vector<std::uint32_t> touched;
    std::size_t attaching = 0;

    void mark(std::uint32_t index, std::int8_t value) {
      if (change[index] == 0) {
        touched.push_back(index);
      }
      change[index] = value;
    }
    void clear() {
      for (std::uint32_t index : touched) {
        change[index] = 0;
      }
      touched.clear();
      attaching = 0;
    }
  };

  template <typename ComponentType>
  struct Plan final : PlanState {
    using PlanState::PlanState;
  };

  template <typename ComponentType>
  Plan<ComponentType>& plan() {
    return std::get<Plan<ComponentType>>(plans_);
  }

  bool will_be_alive(Entity entity) const {
    return alive(entity) && !destroying_[entity.index];
  }

  template <typename ComponentType>
  bool will_have(Entity entity) const {
    std::int8_t change =
        std::get<Plan<ComponentType>>(plans_).change[entity.index];
    return change != 0 ? change > 0
                       : store_of<ComponentType>().contains(entity);
  }

  template <typename ComponentType>
  bool has_room() const {
    const ComponentStore<ComponentType>& components = store_of<ComponentType>();
    return components.size() + std::get<Plan<ComponentType>>(plans_).attaching <
           components.capacity();
  }

  template <typename... CheckedTypes>
  std::optional<Status> check_room() const {
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

  static std::optional<Status> check_alias(const Alias& alias) {
    if (alias.empty()) {
      return lib::raise(BuildError::ALIAS_INVALID, "An alias cannot be empty.");
    }
    return std::nullopt;
  }

  //-- Builders ----------------------------------------------------------------

  template <Archetypal ArchetypeType, typename... InitialTypes>
  std::expected<Entity, Status> build_create(
      const Alias& alias, std::optional<Entity> parent,
      std::tuple<InitialTypes...> components) {
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
  std::expected<void, Status> build_change(
      Entity entity, std::tuple<AttachedTypes...> components,
      TypeList<DetachedTypes...>, const AliasChanges& aliases) {
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

  std::expected<void, Status> build_destroy(Entity entity) {
    if (!will_be_alive(entity)) {
      return std::unexpected(
          lib::raise(BuildError::ENTITY_NOT_ALIVE, "The entity is not alive."));
    }
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
  void record_attach(Entity entity, ComponentType component) {
    remember_plan<ComponentType>(entity.index);
    plan<ComponentType>().mark(entity.index, +1);
    ++plan<ComponentType>().attaching;
    commands_.push_back(AttachCommand<ComponentType>{
        entity, std::move(component),
        segment_of<ComponentType>(archetype_of_index_[entity.index])});
  }

  template <typename ComponentType>
  void record_detach(Entity entity) {
    remember_plan<ComponentType>(entity.index);
    plan<ComponentType>().mark(entity.index, -1);
    commands_.push_back(DetachCommand<ComponentType>{entity});
  }

  //-- Applying commands -------------------------------------------------------

  template <typename ComponentType>
  void apply(lib::InOut<AttachCommand<ComponentType>> command) {
    CHECK_INVARIANT(alive(command->entity));
    spatial_index_current_ =
        spatial_index_current_ && !std::is_same_v<ComponentType, SpatialType>;
    std::get<ComponentStore<ComponentType>>(stores_).append(
        command->entity, std::move(command->component), command->segment);
  }

  template <typename ComponentType>
  void apply(lib::InOut<DetachCommand<ComponentType>> command) {
    CHECK_INVARIANT(alive(command->entity));
    spatial_index_current_ =
        spatial_index_current_ && !std::is_same_v<ComponentType, SpatialType>;
    std::get<ComponentStore<ComponentType>>(stores_).erase(command->entity);
  }

  void apply(lib::InOut<DestroyCommand> command) {
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
  SpatialIndex spatial_index_{0, 1.0};
  bool spatial_index_current_ = false;
  std::vector<bool> destroying_;
  std::vector<std::uint32_t> destroying_list_;

  // How to undo what open transactions planned, oldest first.
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

// Builds a world. How many of each archetype it holds at once sizes it: the
// entity capacity is the total, and each component's store holds as many as
// every archetype that requires or allows the component. Nothing is reserved
// per archetype; capacity is pooled. Start with `World::set_up()`, and build
// into the caller's world, since a world never moves:
//
//   World world;
//   std::expected<void, Status> built =
//       World::set_up()
//           .numbered(1)
//           .holding<archetype::RedDrone>(drones)
//           .holding<archetype::Track>(drones)
//           .cells_of(250.0 * model::meter)
//           .build(lib::Out(world));
template <typename WorldType>
class [[nodiscard]] SetUpBuilder final {
 public:
  using SpatialType = typename WorldType::SpatialComponent;
  using ArchetypeList = typename WorldType::ArchetypeList;
  using ComponentList = typename WorldType::ComponentList;

  // The world's instance in its Name and Identity. Zero unless given.
  SetUpBuilder numbered(std::uint32_t number) && {
    number_ = number;
    return std::move(*this);
  }

  // Holds `count` more entities of `ArchetypeType` alive at once. Holdings add
  // up, so a scenario can count each thing that creates the archetype.
  template <Archetypal ArchetypeType>
  SetUpBuilder holding(std::size_t count) && {
    static_assert(contains_v<ArchetypeList, ArchetypeType>,
                  "This archetype is not in the world's archetype list.");
    std::size_t& holding = holdings_[index_of_v<ArchetypeList, ArchetypeType>];
    holding = add_saturating(holding, count);
    return std::move(*this);
  }

  // The edge of a spatial index cell. About the radius of a typical query
  // works well. One coordinate unit unless given.
  SetUpBuilder cells_of(distance_of_t<SpatialType> size) && {
    static_assert(std::default_initializable<SpatialType>,
                  "Sizing cells needs a default spatial component to convert "
                  "the distance with.");
    cell_size_ = coordinate_length(SpatialType{}, size);
    return std::move(*this);
  }

  // Fills `world` as planned, discarding everything it held. A refused plan
  // leaves `world` as it was.
  std::expected<void, Status> build(lib::Out<WorldType> world) && {
    if (!(cell_size_ > 0.0) || !std::isfinite(cell_size_)) {
      return std::unexpected(
          lib::raise(BuildError::CELL_SIZE_INVALID,
                     "A world's spatial index cells must have a positive, "
                     "finite size."));
    }
    typename WorldType::Configuration configuration{.number = number_,
                                                    .cell_size = cell_size_};
    for (std::size_t holding : holdings_) {
      configuration.entities = add_saturating(configuration.entities, holding);
    }
    // Every store indexes its pool with 32 bits, with a partly filled chunk
    // per segment to spare.
    constexpr std::size_t MOST =
        std::size_t{std::numeric_limits<std::uint32_t>::max()} / 2;
    if (configuration.entities > MOST) {
      return std::unexpected(
          lib::raise(BuildError::CAPACITY_TOO_LARGE,
                     std::format("A world holds at most {} entities, not {}.",
                                 MOST, configuration.entities)));
    }
    for_each_type(ComponentList{}, [&]<typename ComponentType>() {
      std::size_t& capacity =
          configuration.capacities[index_of_v<ComponentList, ComponentType>];
      for (std::size_t archetype = 0; archetype < holdings_.size();
           ++archetype) {
        if (WorldType::template archetype_permits<ComponentType>(archetype)) {
          capacity = add_saturating(capacity, holdings_[archetype]);
        }
      }
    });
    world->initialize(configuration);
    return {};
  }

 private:
  // Sums that would wrap stay at the largest size, so build() refuses them.
  static std::size_t add_saturating(std::size_t a, std::size_t b) {
    return b > std::numeric_limits<std::size_t>::max() - a
               ? std::numeric_limits<std::size_t>::max()
               : a + b;
  }

  std::uint32_t number_ = 0;
  // How many of each archetype, by its position in ArchetypeList.
  std::array<std::size_t, ArchetypeList::size> holdings_{};
  double cell_size_ = 1.0;
};

// Which entities a query form selects. `each<ChosenType>()` chose them by
// archetype, if ChosenType is one, or else by component; `near` and `keep`
// narrow them. Entities already planned for destruction are never selected.
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
  void near(const SpatialType& center, DistanceType radius) {
    near_ = Near{.center = center, .radius = radius};
  }

  // Keeps only entities for which `predicate(Entity)` is true.
  template <typename PredicateType>
  void keep(PredicateType&& predicate) {
    predicates_.emplace_back(std::forward<PredicateType>(predicate));
  }

  // The selected entities, in a deterministic order: the archetype's segment
  // or the component's store, or the spatial index's order when narrowed by
  // `near`.
  std::vector<Entity> select(lib::InOut<WorldType> world) const {
    std::vector<Entity> selected;
    auto consider = [&](Entity entity) {
      if (world->will_be_alive(entity) &&
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

  static bool is_chosen(const WorldType& world, Entity entity) {
    if constexpr (Archetypal<ChosenType>) {
      return world.archetype_of_index_[entity.index] ==
             index_of_v<typename WorldType::ArchetypeList, ChosenType>;
    } else {
      return world.template store_of<ChosenType>().contains(entity);
    }
  }

  // Visits every chosen entity: an archetype's segment of the EntityArchetype
  // store, or every owner in the component's store.
  template <typename VisitorType>
  static void for_each_chosen(const WorldType& world, VisitorType&& visit) {
    if constexpr (Archetypal<ChosenType>) {
      const auto& archetypes = world.template store_of<EntityArchetype>();
      constexpr std::size_t SEGMENT =
          WorldType::template segment_of<EntityArchetype>(
              index_of_v<typename WorldType::ArchetypeList, ChosenType>);
      for (std::size_t ordinal = 0; ordinal < archetypes.chunks_in(SEGMENT);
           ++ordinal) {
        auto chunk = archetypes.chunk(SEGMENT, ordinal);
        for (std::size_t i = 0; i < chunk.size; ++i) {
          visit(chunk.owners[i]);
        }
      }
    } else {
      world.template store_of<ChosenType>().for_each(
          [&](Entity owner, const ChosenType&) { visit(owner); });
    }
  }

  std::optional<Near> near_;
  std::vector<std::function<bool(Entity)>> predicates_;
};

// Destroys every entity a query selects, atomically, and returns how many:
//
//   world.destroy()
//       .each<archetype::RedDrone>()            // or a component:
//       each<Health>() .within(asset_kinematics, 500.0 * meter)
//       .where([&](Entity drone) { return drone != spared; })
//       .build();
//
// `each` comes first. `build()` plans every destruction in one transaction,
// so it destroys all of the selected entities or none.
template <typename WorldType, typename ChosenType>
class [[nodiscard]] DestroyQueryBuilder final {
 public:
  using SpatialType = typename WorldType::SpatialComponent;
  using DistanceType = distance_of_t<SpatialType>;

  // Keeps a reference to `world` until the utterance is built.
  explicit DestroyQueryBuilder(lib::Depend<WorldType> world)
      : world_{world.get()} {}

  // Selects every entity of archetype `NextType`, or every entity with
  // component `NextType`.
  template <typename NextType>
    requires std::is_void_v<ChosenType>
  auto each() && {
    return DestroyQueryBuilder<WorldType, NextType>{lib::Depend(*world_)};
  }

  // Keeps only entities whose spatial component is within `radius` of
  // `center`.
  auto within(const SpatialType& center, DistanceType radius) &&
    requires(!std::is_void_v<ChosenType>)
  {
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

  std::expected<std::size_t, Status> build() &&
    requires(!std::is_void_v<ChosenType>)
  {
    std::vector<Entity> selected = query_.select(lib::InOut(*world_));
    auto transaction = world_->transaction();
    for (Entity entity : selected) {
      RETURN_IF_UNEXPECTED(world_->destroy(entity).build());
    }
    transaction.commit();
    return selected.size();
  }

 private:
  template <typename, typename>
  friend class DestroyQueryBuilder;

  struct Unchosen final {};
  using QueryType = std::conditional_t<std::is_void_v<ChosenType>, Unchosen,
                                       Query<WorldType, ChosenType>>;

  // Never null once constructed; Depend checks it.
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
//       .detach<Health>()
//       .alias("hostile")
//       .build();
//
// `each` comes first, then any of `within` and `where`, then the change in the
// words of ChangeBuilder: `attach`, `detach`, `alias` and `unalias`. `build()`
// makes the change to each selected entity in one transaction; if the world
// refuses it for any of them, nothing changes and build() returns the Status.
template <typename WorldType, typename ChosenType,
          typename AttachedListType = TypeList<>,
          typename DetachedListType = TypeList<>, bool Aliasing = false>
class ChangeQueryBuilder;

template <typename WorldType, typename ChosenType, typename... AttachedTypes,
          typename... DetachedTypes, bool Aliasing>
class [[nodiscard]]
ChangeQueryBuilder<WorldType, ChosenType, TypeList<AttachedTypes...>,
                   TypeList<DetachedTypes...>, Aliasing>
    final {
 public:
  using SpatialType = typename WorldType::SpatialComponent;
  using DistanceType = distance_of_t<SpatialType>;

  // Keeps a reference to `world` until the utterance is built.
  explicit ChangeQueryBuilder(lib::Depend<WorldType> world)
      : world_{world.get()} {}

  // Selects every entity of archetype `NextType`, or every entity with
  // component `NextType`.
  template <typename NextType>
    requires std::is_void_v<ChosenType>
  auto each() && {
    return ChangeQueryBuilder<WorldType, NextType>{lib::Depend(*world_)};
  }

  // Keeps only entities whose spatial component is within `radius` of
  // `center`.
  auto within(const SpatialType& center, DistanceType radius) &&
    requires(!std::is_void_v<ChosenType>)
  {
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

  // Attaches a copy of `component` to every selected entity.
  template <typename ArgumentType>
  auto attach(ArgumentType&& component) &&
    requires(!std::is_void_v<ChosenType>)
  {
    using ComponentType = std::remove_cvref_t<ArgumentType>;
    check_component<ComponentType>();
    return ChangeQueryBuilder<WorldType, ChosenType,
                              TypeList<AttachedTypes..., ComponentType>,
                              TypeList<DetachedTypes...>, Aliasing>{
        lib::Depend(*world_), std::move(query_),
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
    return ChangeQueryBuilder<WorldType, ChosenType, TypeList<AttachedTypes...>,
                              TypeList<DetachedTypes..., ComponentType>,
                              Aliasing>{lib::Depend(*world_), std::move(query_),
                                        std::move(components_),
                                        std::move(aliases_)};
  }

  // Gives every selected entity the alias `alias`.
  auto alias(Alias alias) &&
    requires(!std::is_void_v<ChosenType>)
  {
    aliases_.given.push_back(std::move(alias));
    return ChangeQueryBuilder<WorldType, ChosenType, TypeList<AttachedTypes...>,
                              TypeList<DetachedTypes...>, true>{
        lib::Depend(*world_), std::move(query_), std::move(components_),
        std::move(aliases_)};
  }

  // Takes the alias `alias` from every selected entity.
  auto unalias(Alias alias) &&
    requires(!std::is_void_v<ChosenType>)
  {
    aliases_.taken.push_back(std::move(alias));
    return ChangeQueryBuilder<WorldType, ChosenType, TypeList<AttachedTypes...>,
                              TypeList<DetachedTypes...>, true>{
        lib::Depend(*world_), std::move(query_), std::move(components_),
        std::move(aliases_)};
  }

  std::expected<std::size_t, Status> build() &&
    requires(!std::is_void_v<ChosenType>)
  {
    static_assert(
        sizeof...(AttachedTypes) + sizeof...(DetachedTypes) > 0 || Aliasing,
        "A change must attach, detach, alias or unalias something.");
    std::vector<Entity> selected = query_.select(lib::InOut(*world_));
    auto transaction = world_->transaction();
    for (Entity entity : selected) {
      RETURN_IF_UNEXPECTED(
          (ChangeBuilder<WorldType, TypeList<AttachedTypes...>,
                         TypeList<DetachedTypes...>, Aliasing>{
               entity, components_, aliases_, lib::Depend(*world_)})
              .build());
    }
    transaction.commit();
    return selected.size();
  }

 private:
  template <typename, typename, typename, typename, bool>
  friend class ChangeQueryBuilder;

  struct Unchosen final {};
  using QueryType = std::conditional_t<std::is_void_v<ChosenType>, Unchosen,
                                       Query<WorldType, ChosenType>>;

  ChangeQueryBuilder(lib::Depend<WorldType> world, QueryType query,
                     std::tuple<AttachedTypes...> components,
                     AliasChanges aliases)
      : world_{world.get()},
        query_{std::move(query)},
        components_{std::move(components)},
        aliases_{std::move(aliases)} {}

  // The same checks as ChangeBuilder's, made where the word is spoken.
  template <typename ComponentType>
  static constexpr void check_component() {
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
  QueryType query_;
  std::tuple<AttachedTypes...> components_;
  AliasChanges aliases_;
};

}  // namespace simon::framework

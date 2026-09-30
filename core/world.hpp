// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <algorithm>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <expected>
#include <format>
#include <limits>
#include <map>
#include <optional>
#include <string>
#include <string_view>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <variant>
#include <vector>

#include "base/core.hpp"
#include "core/archetype.hpp"
#include "core/builder.hpp"
#include "core/command.hpp"
#include "core/entity.hpp"
#include "core/name.hpp"
#include "core/status.hpp"
#include "core/store.hpp"
#include "core/type_list.hpp"

namespace simon::core {

// What a world needs to know about space: a distance and a pose. The world
// indexes every entity-component of its spatial type. Distances may carry
// units; they only need to be ordered.
template <typename Type>
concept Spatial = requires(const Type& a, const Type& b) {
  { distance(a, b) < distance(a, b) } -> std::convertible_to<bool>;
  pose(a);
};

template <Spatial Type>
using distance_of_t = decltype(distance(std::declval<const Type&>(), std::declval<const Type&>()));

struct WorldConfiguration {
  std::uint32_t number = 0;    // The world's instance in its Name and Identity.
  std::size_t entities = 0;    // Entity capacity.
  std::size_t components = 0;  // Capacity of each component store.
};

// Grants mutable store access. Only the code that runs systems constructs it.
class SystemAccess final {
 private:
  SystemAccess() = default;
  friend struct SystemRunner;
};

// The entity database: a factory of builders (the only way to write) and a
// query interface over the result (the only way to read).
//
// Everything in it has a Name ({kind, instance}), an Identity computed from the
// Name ("/world/1/entity/2"), and any number of Aliases ("ego"). See name.hpp.
template <Spatial SpatialType, typename... Components>
class World final {
 public:
  using SpatialComponent = SpatialType;
  using ComponentList = TypeList<SpatialType, EntityArchetype, Parent, Components...>;
  using Command = command_for_t<ComponentList>;

  static_assert(is_unique_v<ComponentList>,
                "Each component may appear once in a world's component list.");

  DECLARE_COPY_DELETE(World);
  DECLARE_MOVE_DELETE(World);  // Builders hold a pointer to their world.

  explicit World(WorldConfiguration configuration)
      : number_{configuration.number},
        entities_{configuration.entities},
        stores_{Store<SpatialType>{configuration.components, configuration.entities},
                Store<EntityArchetype>{configuration.entities, configuration.entities},
                Store<Parent>{configuration.entities, configuration.entities},
                Store<Components>{configuration.components, configuration.entities}...},
        plans_{Plan<SpatialType>{configuration.entities},
               Plan<EntityArchetype>{configuration.entities},
               Plan<Parent>{configuration.entities},
               Plan<Components>{configuration.entities}...},
        destroying_(configuration.entities, false),
        instance_of_index_(configuration.entities, 0) {
    // Component types are aliased by their type names, qualified and short.
    for_each_type(ComponentList{}, [&]<typename Component>() {
      std::string type_name = lib::to_type_string<Component>();
      Name name = name_of<Component>();
      give_alias(name, type_name);
      if (std::size_t colons = type_name.rfind("::"); colons != std::string::npos) {
        give_alias(name, type_name.substr(colons + 2));
      }
    });
  }
  ~World() = default;

  //-- Write --------------------------------------------------------------------

  // Creates an entity of `Archetype`, with an optional alias.
  template <ArchetypeType Archetype>
  auto create(std::string_view alias = {}) {
    return CreateBuilder<World, Archetype, true>{lib::Depend<World>{*this}, std::string{alias},
                                                 std::nullopt, {}};
  }
  auto change(Entity entity) {
    return ChangeBuilder<World>{lib::Depend<World>{*this}, entity, {}, {}};
  }
  auto destroy(Entity entity) { return DestroyBuilder<World>{lib::Depend<World>{*this}, entity}; }

  // Applies every pending command, in the order it was recorded. Builders
  // validated each command against the state the world would be in, so this
  // cannot fail.
  void sync() {
    std::vector<Command> commands = std::exchange(commands_, {});
    for (Command& command : commands) {
      std::visit([&](auto& operation) { this->apply(lib::InOut(operation)); }, command);
    }
    std::apply([](auto&... plan) { (plan.clear(), ...); }, plans_);
    for (std::uint32_t index : destroying_list_) {
      destroying_[index] = false;
    }
    destroying_list_.clear();
  }

  std::size_t pending() const { return commands_.size(); }

  //-- Read: entities and components -----------------------------------------

  bool alive(Entity entity) const { return entities_.alive(entity); }
  std::size_t size() const { return entities_.size(); }

  template <typename Component>
  const Store<Component>& store() const {
    static_assert(contains_v<ComponentList, Component>,
                  "This component is not in the world's component list.");
    return std::get<Store<Component>>(stores_);
  }

  Name archetype_of(Entity entity) const {
    return store<EntityArchetype>().get(entity).archetype;
  }

  // The entity `entity` was created under, if any. It may no longer be alive.
  std::optional<Entity> parent_of(Entity entity) const {
    const Parent* parent = store<Parent>().try_get(entity);
    return parent ? std::optional{parent->entity} : std::nullopt;
  }

  // Visits every entity whose spatial entity-component is within `radius` of
  // `center`, as `visit(Entity, const SpatialType&)`.
  template <typename Visitor>
  void within(const SpatialType& center, distance_of_t<SpatialType> radius,
              Visitor&& visit) const {
    const Store<SpatialType>& spatial = store<SpatialType>();
    for (std::size_t i = 0; i < spatial.size(); ++i) {
      if (distance(center, spatial.data(i)) <= radius) {
        visit(spatial.owner(i), spatial.data(i));
      }
    }
  }

  //-- Read: names, identities and aliases -----------------------------------

  std::uint32_t number() const { return number_; }
  Name name() const { return Name{Kind::WORLD, number_}; }

  // An entity's name. Known as soon as the entity is created, before sync.
  Name name_of(Entity entity) const {
    CHECK_PRECONDITION(alive(entity));
    return Name{Kind::ENTITY, instance_of_index_[entity.index]};
  }

  template <typename Component>
  static constexpr Name name_of() {
    return Name{Kind::COMPONENT, component_number<Component>()};
  }

  // The name of `entity`'s `Component`, which it need not have yet.
  template <typename Component>
  Name name_of(Entity entity) const {
    return entity_component_name(component_number<Component>(), name_of(entity).instance);
  }

  // The live entity an entity or entity-component name refers to.
  std::optional<Entity> entity_of(Name name) const {
    if (name.kind != static_cast<std::uint32_t>(Kind::ENTITY) && !is_entity_component(name)) {
      return std::nullopt;
    }
    auto iter = entity_of_instance_.find(name.instance);
    return iter != entity_of_instance_.end() ? std::optional{iter->second} : std::nullopt;
  }

  std::string identity_of(Name name) const { return core::identity_of(number_, name); }

  // The name an identity refers to, if it names something in this world.
  std::optional<Name> find(std::string_view identity) const {
    std::optional<ParsedIdentity> parsed = parse_identity(identity);
    if (!parsed || parsed->world != number_ || !exists(parsed->name)) {
      return std::nullopt;
    }
    return parsed->name;
  }

  // Every name with `alias`, in the order the aliases were given.
  std::vector<Name> find_alias(std::string_view alias) const {
    std::vector<Name> names;
    auto [begin, end] = aliases_.equal_range(std::string{alias});
    for (auto iter = begin; iter != end; ++iter) {
      names.push_back(iter->second);
    }
    return names;
  }

  // Every alias `name` has, in the order they were given.
  std::vector<std::string> aliases_of(Name name) const {
    auto iter = aliases_of_name_.find(name);
    return iter != aliases_of_name_.end() ? iter->second : std::vector<std::string>{};
  }

  // A one-line description for a console, e.g.
  // "/world/1/entity/2 (red) archetype ball: Kinematics, Collider".
  std::string describe(Name name) const {
    std::string text = identity_of(name);
    std::vector<std::string> aliases = aliases_of(name);
    for (std::size_t i = 0; i < aliases.size(); ++i) {
      text += std::format("{}{}", i == 0 ? " (" : ", ", aliases[i]);
    }
    text += aliases.empty() ? "" : ")";
    if (std::optional<Entity> entity = entity_of(name);
        entity && name.kind == static_cast<std::uint32_t>(Kind::ENTITY)) {
      std::vector<std::string> archetype = aliases_of(archetype_of(*entity));
      text += std::format(" archetype {}:", archetype.empty() ? "?" : archetype.front());
      bool first = true;
      for_each_type(ComponentList{}, [&]<typename Component>() {
        if (!is_built_in_v<Component> && store<Component>().contains(*entity)) {
          std::vector<std::string> component = aliases_of(name_of<Component>());
          text += std::format("{} {}", first ? "" : ",", component.back());
          first = false;
        }
      });
    }
    return text;
  }

  //-- Systems ------------------------------------------------------------------

  template <typename Component>
  Store<Component>& mutable_store(SystemAccess) {
    static_assert(contains_v<ComponentList, Component>,
                  "This component is not in the world's component list.");
    return std::get<Store<Component>>(stores_);
  }

 private:
  template <typename, ArchetypeType, bool, typename...>
  friend class CreateBuilder;
  template <typename, typename, typename, bool>
  friend class ChangeBuilder;
  template <typename>
  friend class DestroyBuilder;

  template <typename Component>
  static constexpr std::uint32_t component_number() {
    static_assert(contains_v<ComponentList, Component>,
                  "This component is not in the world's component list.");
    return static_cast<std::uint32_t>(index_of_v<ComponentList, Component>);
  }

  bool exists(Name name) const {
    switch (static_cast<Kind>(std::min(name.kind,
                                       static_cast<std::uint32_t>(Kind::ENTITY_COMPONENT)))) {
      case Kind::WORLD:
        return name.instance == number_;
      case Kind::ARCHETYPE:
        return name.instance < next_archetype_instance_;
      case Kind::COMPONENT:
        return name.instance < ComponentList::size;
      case Kind::SYSTEM:
        return true;  // Systems belong to schedulers, which the world does not see.
      case Kind::ENTITY:
        return entity_of(name).has_value();
      case Kind::ENTITY_COMPONENT:
        return component_of(name) < ComponentList::size && entity_of(name).has_value() &&
               has_component_number(*entity_of(name), component_of(name));
      case Kind::NONE:
        break;
    }
    return false;
  }

  bool has_component_number(Entity entity, std::uint32_t number) const {
    bool found = false;
    for_each_type(ComponentList{}, [&]<typename Component>() {
      found = found || (component_number<Component>() == number &&
                        store<Component>().contains(entity));
    });
    return found;
  }

  template <ArchetypeType Archetype>
  Name archetype_name() {
    auto [iter, inserted] = archetypes_.try_emplace(std::string{Archetype::name});
    if (inserted) {
      iter->second = Name{Kind::ARCHETYPE, next_archetype_instance_++};
      give_alias(iter->second, Archetype::name);
    }
    return iter->second;
  }

  void give_alias(Name name, std::string_view alias) {
    aliases_.emplace(std::string{alias}, name);
    aliases_of_name_[name].emplace_back(alias);
  }

  void take_alias(Name name, std::string_view alias) {
    auto [begin, end] = aliases_.equal_range(std::string{alias});
    for (auto iter = begin; iter != end; ++iter) {
      if (iter->second == name) {
        aliases_.erase(iter);
        break;
      }
    }
    std::vector<std::string>& given = aliases_of_name_[name];
    std::erase(given, std::string{alias});
    if (given.empty()) {
      aliases_of_name_.erase(name);
    }
  }

  bool has_alias(Name name, std::string_view alias) const {
    auto iter = aliases_of_name_.find(name);
    return iter != aliases_of_name_.end() && std::ranges::contains(iter->second, alias);
  }

  //-- Planned state: what the world will be once pending commands apply --------

  // For one component: which entities will gain or lose it, and how many
  // entity-components are waiting to be attached.
  struct PlanState {
    explicit PlanState(std::size_t entity_capacity) : change(entity_capacity, 0) {}

    std::vector<std::int8_t> change;  // +1 will be attached, -1 will be detached.
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

  template <typename Component>
  struct Plan : PlanState {
    using PlanState::PlanState;
  };

  template <typename Component>
  Plan<Component>& plan() {
    return std::get<Plan<Component>>(plans_);
  }

  bool will_be_alive(Entity entity) const {
    return alive(entity) && !destroying_[entity.index];
  }

  template <typename Component>
  bool will_have(Entity entity) const {
    std::int8_t change = std::get<Plan<Component>>(plans_).change[entity.index];
    return change != 0 ? change > 0 : store<Component>().contains(entity);
  }

  template <typename Component>
  bool has_room() const {
    const Store<Component>& components = store<Component>();
    return components.size() + std::get<Plan<Component>>(plans_).attaching <
           components.capacity();
  }

  template <typename... Checked>
  std::optional<Status> check_room() const {
    std::optional<Status> failure;
    auto check = [&]<typename Component>() {
      if (!failure && !has_room<Component>()) {
        failure = lib::raise(
            BuildCondition::COMPONENT_CAPACITY_EXHAUSTED,
            std::format("The {} store is full.", lib::to_type_string<Component>()));
      }
    };
    (check.template operator()<Checked>(), ...);
    return failure;
  }

  static std::optional<Status> check_alias(std::string_view alias) {
    if (alias.empty()) {
      return lib::raise(BuildCondition::ALIAS_INVALID, "An alias cannot be empty.");
    }
    return std::nullopt;
  }

  //-- Builders -----------------------------------------------------------------

  template <ArchetypeType Archetype, typename... Initial>
  std::expected<Entity, Status> build_create(const std::string& alias,
                                             std::optional<Entity> parent,
                                             std::tuple<Initial...> components) {
    if (entities_.size() == entities_.capacity()) {
      return std::unexpected(lib::raise(BuildCondition::ENTITY_CAPACITY_EXHAUSTED,
                                        "The world is out of entity capacity."));
    }
    if (parent && !will_be_alive(*parent)) {
      return std::unexpected(lib::raise(BuildCondition::ENTITY_NOT_ALIVE,
                                        "The parent entity is not alive."));
    }
    if (std::optional<Status> failure = check_room<Initial...>()) {
      return std::unexpected(*failure);
    }
    CHECK_PRECONDITION(next_entity_instance_ < std::numeric_limits<std::uint32_t>::max());

    Entity entity = entities_.create();
    std::uint32_t instance = next_entity_instance_++;
    instance_of_index_[entity.index] = instance;
    entity_of_instance_.emplace(instance, entity);
    if (!alias.empty()) {
      give_alias(Name{Kind::ENTITY, instance}, alias);
    }

    record_attach(entity, EntityArchetype{.archetype = archetype_name<Archetype>()});
    if (parent) {
      record_attach(entity, Parent{.entity = *parent});
    }
    std::apply([&](auto&... component) { (record_attach(entity, std::move(component)), ...); },
               components);
    return entity;
  }

  template <typename... Attached, typename... Detached>
  std::expected<void, Status> build_change(Entity entity, std::tuple<Attached...> components,
                                           TypeList<Detached...>,
                                           const AliasChanges& aliases) {
    if (!will_be_alive(entity)) {
      return std::unexpected(
          lib::raise(BuildCondition::ENTITY_NOT_ALIVE, "The entity is not alive."));
    }
    std::optional<Status> failure;
    auto require = [&]<typename Component>(bool present, BuildCondition condition,
                                           const char* message) {
      if (!failure && will_have<Component>(entity) != present) {
        std::string type_name = lib::to_type_string<Component>();
        failure = lib::raise(condition,
                             std::vformat(message, std::make_format_args(type_name)));
      }
    };
    (require.template operator()<Attached>(false, BuildCondition::COMPONENT_ALREADY_ATTACHED,
                                            "The entity already has a {}."),
     ...);
    (require.template operator()<Detached>(true, BuildCondition::COMPONENT_NOT_ATTACHED,
                                            "The entity has no {}."),
     ...);
    if (!failure) {
      failure = check_room<Attached...>();
    }

    Name name = name_of(entity);
    for (std::size_t i = 0; !failure && i < aliases.given.size(); ++i) {
      const std::string& alias = aliases.given[i];
      failure = check_alias(alias);
      if (!failure && (has_alias(name, alias) ||
                       std::count(aliases.given.begin(), aliases.given.begin() + i, alias))) {
        failure = lib::raise(BuildCondition::ALIAS_ALREADY_GIVEN,
                             std::format("The entity already has the alias {}.", alias));
      }
    }
    for (std::size_t i = 0; !failure && i < aliases.taken.size(); ++i) {
      const std::string& alias = aliases.taken[i];
      if (!has_alias(name, alias) ||
          std::count(aliases.taken.begin(), aliases.taken.begin() + i, alias)) {
        failure = lib::raise(BuildCondition::ALIAS_NOT_GIVEN,
                             std::format("The entity does not have the alias {}.", alias));
      }
    }
    if (failure) {
      return std::unexpected(*failure);
    }

    std::apply([&](auto&... component) { (record_attach(entity, std::move(component)), ...); },
               components);
    (record_detach<Detached>(entity), ...);
    // Aliases are an index beside the stores, not store shape, so they change
    // immediately.
    for (const std::string& alias : aliases.taken) {
      take_alias(name, alias);
    }
    for (const std::string& alias : aliases.given) {
      give_alias(name, alias);
    }
    return {};
  }

  std::expected<void, Status> build_destroy(Entity entity) {
    if (!will_be_alive(entity)) {
      return std::unexpected(
          lib::raise(BuildCondition::ENTITY_NOT_ALIVE, "The entity is not alive."));
    }
    destroying_[entity.index] = true;
    destroying_list_.push_back(entity.index);
    commands_.push_back(DestroyCommand{entity});
    return {};
  }

  template <typename Component>
  void record_attach(Entity entity, Component component) {
    plan<Component>().mark(entity.index, +1);
    ++plan<Component>().attaching;
    commands_.push_back(AttachCommand<Component>{entity, std::move(component)});
  }

  template <typename Component>
  void record_detach(Entity entity) {
    plan<Component>().mark(entity.index, -1);
    commands_.push_back(DetachCommand<Component>{entity});
  }

  //-- Applying commands --------------------------------------------------------

  template <typename Component>
  void apply(lib::InOut<AttachCommand<Component>> command) {
    CHECK_INVARIANT(alive(command->entity));
    std::get<Store<Component>>(stores_).append(command->entity,
                                               std::move(command->component));
  }

  template <typename Component>
  void apply(lib::InOut<DetachCommand<Component>> command) {
    CHECK_INVARIANT(alive(command->entity));
    std::get<Store<Component>>(stores_).erase(command->entity);
  }

  void apply(lib::InOut<DestroyCommand> command) {
    Entity entity = command->entity;
    CHECK_INVARIANT(alive(entity));
    Name name = name_of(entity);
    for (const std::string& alias : aliases_of(name)) {
      take_alias(name, alias);
    }
    entity_of_instance_.erase(name.instance);
    std::apply(
        [entity](auto&... store) {
          ((store.contains(entity) ? store.erase(entity) : void()), ...);
        },
        stores_);
    entities_.destroy(entity);
  }

  std::uint32_t number_;
  EntityTable entities_;
  std::tuple<Store<SpatialType>, Store<EntityArchetype>, Store<Parent>,
             Store<Components>...>
      stores_;
  std::vector<Command> commands_;
  std::tuple<Plan<SpatialType>, Plan<EntityArchetype>, Plan<Parent>, Plan<Components>...>
      plans_;
  std::vector<bool> destroying_;
  std::vector<std::uint32_t> destroying_list_;

  // Names. Entity instances are never reused; Entity indices are.
  std::uint32_t next_entity_instance_ = 0;
  std::vector<std::uint32_t> instance_of_index_;
  std::unordered_map<std::uint32_t, Entity> entity_of_instance_;
  std::uint32_t next_archetype_instance_ = 0;
  std::map<std::string, Name, std::less<>> archetypes_;

  // Aliases, many-to-many. A multimap keeps equal aliases in the order given.
  std::multimap<std::string, Name> aliases_;
  std::unordered_map<Name, std::vector<std::string>> aliases_of_name_;
};

}  // namespace simon::core

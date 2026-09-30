// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <cstddef>
#include <type_traits>

namespace simon::core {

template <typename... Types>
struct TypeList {
  static constexpr std::size_t size = sizeof...(Types);
};

// Whether `Type` is one of `List`'s types.
template <typename List, typename Type>
struct Contains;

template <typename... Types, typename Type>
struct Contains<TypeList<Types...>, Type>
    : std::bool_constant<(std::is_same_v<Types, Type> || ...)> {};

template <typename List, typename Type>
inline constexpr bool contains_v = Contains<List, Type>::value;

// The position of the first `Type` in `List`, or `List::size` when absent.
template <typename List, typename Type>
struct IndexOf;

template <typename Type>
struct IndexOf<TypeList<>, Type> : std::integral_constant<std::size_t, 0> {};

template <typename First, typename... Rest, typename Type>
struct IndexOf<TypeList<First, Rest...>, Type>
    : std::integral_constant<
          std::size_t, std::is_same_v<First, Type>
                           ? 0
                           : 1 + IndexOf<TypeList<Rest...>, Type>::value> {};

template <typename List, typename Type>
inline constexpr std::size_t index_of_v = IndexOf<List, Type>::value;

// Whether no type appears twice in `List`.
template <typename List>
struct IsUnique;

template <>
struct IsUnique<TypeList<>> : std::true_type {};

template <typename First, typename... Rest>
struct IsUnique<TypeList<First, Rest...>>
    : std::bool_constant<!contains_v<TypeList<Rest...>, First> &&
                         IsUnique<TypeList<Rest...>>::value> {};

template <typename List>
inline constexpr bool is_unique_v = IsUnique<List>::value;

// Concatenates type lists.
template <typename... Lists>
struct Concatenate;

template <>
struct Concatenate<> {
  using type = TypeList<>;
};

template <typename... Types>
struct Concatenate<TypeList<Types...>> {
  using type = TypeList<Types...>;
};

template <typename... First, typename... Second, typename... Rest>
struct Concatenate<TypeList<First...>, TypeList<Second...>, Rest...> {
  using type =
      typename Concatenate<TypeList<First..., Second...>, Rest...>::type;
};

template <typename... Lists>
using concatenate_t = typename Concatenate<Lists...>::type;

// Applies `Transform` to each type.
template <typename List, template <typename> typename Transform>
struct Map;

template <typename... Types, template <typename> typename Transform>
struct Map<TypeList<Types...>, Transform> {
  using type = TypeList<Transform<Types>...>;
};

template <typename List, template <typename> typename Transform>
using map_t = typename Map<List, Transform>::type;

// Whether every type in `Subset` is in `Superset`.
template <typename Subset, typename Superset>
struct IsSubset;

template <typename... Types, typename Superset>
struct IsSubset<TypeList<Types...>, Superset>
    : std::bool_constant<(contains_v<Superset, Types> && ...)> {};

template <typename Subset, typename Superset>
inline constexpr bool is_subset_v = IsSubset<Subset, Superset>::value;

// Whether `First` and `Second` share any type.
template <typename First, typename Second>
struct Intersects;

template <typename... Types, typename Second>
struct Intersects<TypeList<Types...>, Second>
    : std::bool_constant<(contains_v<Second, Types> || ...)> {};

template <typename First, typename Second>
inline constexpr bool intersects_v = Intersects<First, Second>::value;

// Calls `visit.template operator()<Type>()` for each type, in order.
template <typename... Types, typename Visitor>
constexpr void for_each_type(TypeList<Types...>, Visitor&& visit) {
  (visit.template operator()<Types>(), ...);
}

}  // namespace simon::core

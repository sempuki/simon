// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <cstddef>
#include <type_traits>

namespace simon::framework {

template <typename... Types>
struct TypeList final {
  static constexpr std::size_t size = sizeof...(Types);
};

// Whether `Type` is one of `ListType`'s types.
template <typename ListType, typename Type>
struct Contains;

template <typename... Types, typename Type>
struct Contains<TypeList<Types...>, Type> final
    : std::bool_constant<(std::is_same_v<Types, Type> || ...)> {};

template <typename ListType, typename Type>
inline constexpr bool contains_v = Contains<ListType, Type>::value;

// The position of the first `Type` in `ListType`, or `ListType::size` when
// absent.
template <typename ListType, typename Type>
struct IndexOf;

template <typename Type>
struct IndexOf<TypeList<>, Type> final
    : std::integral_constant<std::size_t, 0> {};

template <typename FirstType, typename... RestTypes, typename Type>
struct IndexOf<TypeList<FirstType, RestTypes...>, Type> final
    : std::integral_constant<
          std::size_t, std::is_same_v<FirstType, Type>
                           ? 0
                           : 1 + IndexOf<TypeList<RestTypes...>, Type>::value> {
};

template <typename ListType, typename Type>
inline constexpr std::size_t index_of_v = IndexOf<ListType, Type>::value;

// Whether no type appears twice in `ListType`.
template <typename ListType>
struct IsUnique;

template <>
struct IsUnique<TypeList<>> final : std::true_type {};

template <typename FirstType, typename... RestTypes>
struct IsUnique<TypeList<FirstType, RestTypes...>> final
    : std::bool_constant<!contains_v<TypeList<RestTypes...>, FirstType> &&
                         IsUnique<TypeList<RestTypes...>>::value> {};

template <typename ListType>
inline constexpr bool is_unique_v = IsUnique<ListType>::value;

// Concatenates type lists.
template <typename... ListTypes>
struct Concatenate;

template <>
struct Concatenate<> final {
  using type = TypeList<>;
};

template <typename... Types>
struct Concatenate<TypeList<Types...>> final {
  using type = TypeList<Types...>;
};

template <typename... FirstType, typename... SecondType, typename... RestTypes>
struct Concatenate<TypeList<FirstType...>, TypeList<SecondType...>,
                   RestTypes...>
    final {
  using type = typename Concatenate<TypeList<FirstType..., SecondType...>,
                                    RestTypes...>::type;
};

template <typename... ListTypes>
using concatenate_t = typename Concatenate<ListTypes...>::type;

// Applies `TransformType` to each type.
template <typename ListType, template <typename> typename TransformType>
struct Map;

template <typename... Types, template <typename> typename TransformType>
struct Map<TypeList<Types...>, TransformType> final {
  using type = TypeList<TransformType<Types>...>;
};

template <typename ListType, template <typename> typename TransformType>
using map_t = typename Map<ListType, TransformType>::type;

// Whether every type in `SubsetType` is in `SupersetType`.
template <typename SubsetType, typename SupersetType>
struct IsSubset;

template <typename... Types, typename SupersetType>
struct IsSubset<TypeList<Types...>, SupersetType> final
    : std::bool_constant<(contains_v<SupersetType, Types> && ...)> {};

template <typename SubsetType, typename SupersetType>
inline constexpr bool is_subset_v = IsSubset<SubsetType, SupersetType>::value;

// Whether `FirstType` and `SecondType` share any type.
template <typename FirstType, typename SecondType>
struct Intersects;

template <typename... Types, typename SecondType>
struct Intersects<TypeList<Types...>, SecondType> final
    : std::bool_constant<(contains_v<SecondType, Types> || ...)> {};

template <typename FirstType, typename SecondType>
inline constexpr bool intersects_v = Intersects<FirstType, SecondType>::value;

// `ListType` without repeats, keeping each type's first position.
template <typename ResultType, typename ListType>
struct Unique;

template <typename ResultType>
struct Unique<ResultType, TypeList<>> final {
  using type = ResultType;
};

template <typename... ResultTypes, typename FirstType, typename... RestTypes>
struct Unique<TypeList<ResultTypes...>, TypeList<FirstType, RestTypes...>>
    final {
  using type = typename Unique<
      std::conditional_t<contains_v<TypeList<ResultTypes...>, FirstType>,
                         TypeList<ResultTypes...>,
                         TypeList<ResultTypes..., FirstType>>,
      TypeList<RestTypes...>>::type;
};

template <typename ListType>
using unique_t = typename Unique<TypeList<>, ListType>::type;

// `ListType` without the types in `RemoveListType`.
template <typename ListType, typename RemoveListType>
struct Without;

template <typename... Types, typename RemoveListType>
struct Without<TypeList<Types...>, RemoveListType> final {
  using type =
      concatenate_t<std::conditional_t<contains_v<RemoveListType, Types>,
                                       TypeList<>, TypeList<Types>>...>;
};

template <typename ListType, typename RemoveListType>
using without_t = typename Without<ListType, RemoveListType>::type;

// Calls `visit.template operator()<Type>()` for each type, in order.
template <typename... Types, typename VisitorType>
constexpr auto for_each_type(TypeList<Types...>, VisitorType&& visit) -> void {
  (visit.template operator()<Types>(), ...);
}

}  // namespace simon::framework

// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <source_location>

namespace simon::framework::contract {
struct ContractError {
  std::source_location origin;
};

struct PreconditionError final : public ContractError {};
struct PostconditionError final : public ContractError {};
struct InvariantError final : public ContractError {};
}  // namespace simon::framework::contract

#define EXPECTS(expr)                                                                \
  [&] {                                                                              \
    if (!(expr)) [[unlikely]] {                                                      \
      throw framework::contract::PreconditionError{std::source_location::current()}; \
    }                                                                                \
  }()

#define ENSURES(expr)                                                                 \
  [&] {                                                                               \
    if (!(expr)) [[unlikely]] {                                                       \
      throw framework::contract::PostconditionError{std::source_location::current()}; \
    }                                                                                 \
  }()

#define ASSERT(expr)                                                              \
  [&] {                                                                           \
    if (!(expr)) [[unlikely]] {                                                   \
      throw framework::contract::InvariantError{std::source_location::current()}; \
    }                                                                             \
  }()

// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include "base/math.hpp"
#include "framework/component.hpp"

namespace simon::component {

struct Movement;

struct Physical final : public framework::Component<Physical> {
  double radius;
  double wind_resistance_factor = 1.0;
  Movement* movement = nullptr;
};

}  // namespace simon::component


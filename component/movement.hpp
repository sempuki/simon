// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include "base/math.hpp"
#include "framework/component.hpp"

namespace simon::component {

struct Environment;
struct Physical;
struct Controls;

struct Movement final : public framework::Component<Movement> {
  Vec3 position;
  Vec3 velocity;

  Environment* environment = nullptr;
  Physical* physical = nullptr;
  Controls* controls = nullptr;
};

}  // namespace simon::component


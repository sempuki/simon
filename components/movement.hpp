// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include "core/math.hpp"
#include "framework/component.hpp"

namespace simon::components {

class Environment;
class Physical;
class Controls;

struct Movement final : public framework::Component<Movement> {
  Vec3 position;
  Vec3 velocity;

  Environment* environment = nullptr;
  Physical* physical = nullptr;
  Controls* controls = nullptr;
};

}  // namespace simon::components


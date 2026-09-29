// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include "base/math.hpp"
#include "framework/component.hpp"

namespace simon {
using lib::Vec2;
using lib::Vec3;
}  // namespace simon

namespace simon::component {

struct Controls final : public framework::Component<Controls> {
  Vec3 acceleration;
};

}  // namespace simon::component


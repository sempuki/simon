// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include "base/math.hpp"
#include "framework/component.hpp"

namespace simon::component {

struct Environment final : public framework::Component<Environment> {
  Vec3 wind;
};

}  // namespace simon::component


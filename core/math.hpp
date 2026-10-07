// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include "Eigen/Core"
#include "Eigen/Geometry"

// Eigen's fixed-size types, in doubles, declared in simon's namespace so that
// simon names them plainly.
namespace simon {

using Vector3 = Eigen::Vector3d;
using Vector4 = Eigen::Vector4d;
using Matrix3 = Eigen::Matrix3d;
using Quaternion = Eigen::Quaterniond;
using AngleAxis = Eigen::AngleAxisd;

}  // namespace simon

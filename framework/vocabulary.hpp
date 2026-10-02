// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "base/core.hpp"

// The vocabulary every part of simon speaks, declared in simon's namespace so
// that simon names it plainly.
namespace simon {

// lib's parameter vocabulary: arguments a function writes (Out), reads and
// writes (InOut), or keeps a reference to (Depend).
using lib::Depend;
using lib::InOut;
using lib::Out;

// Eigen's fixed-size types, in doubles.
using Vector3 = Eigen::Vector3d;
using Vector4 = Eigen::Vector4d;
using Matrix3 = Eigen::Matrix3d;
using Quaternion = Eigen::Quaterniond;
using AngleAxis = Eigen::AngleAxisd;

}  // namespace simon

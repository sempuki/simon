// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include "Eigen/Dense"

namespace simon {

template <std::size_t Rows, std::size_t Cols>
using Matrix = Eigen::Matrix<double, Rows, Cols>;

template <std::size_t Size>
using Vector = Eigen::Matrix<double, Size, 1>;

using Vec3 = Vector<3>;
using Vec2 = Vector<2>;

}  // namespace simon


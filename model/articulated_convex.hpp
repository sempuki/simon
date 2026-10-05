// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <cstdint>
#include <span>

#include "model/articulated.hpp"
#include "model/articulated_collision.hpp"

// Contacts of the convex primitives MuJoCo's analytic colliders leave out:
// an ellipsoid with anything, a cylinder with a capsule, a box or a
// cylinder. MuJoCo finds them by its native Gilbert–Johnson–Keerthi
// distance and expanding polytope (see model/REFERENCES.md), then, where
// two faces meet, by clipping one against the other for up to four
// contacts, or by turning the geoms a little to find more.
namespace simon::model {

// The contacts of two geoms within `margin` by MuJoCo's general convex
// collider, the first's type no later than the second's, their bounding
// radii given (mjc_Convex, mjc_PlaneConvex); writes at most
// MAX_PAIR_CONTACTS and returns how many.
auto collide_convex(const Geom& first, const GeomFrame& first_frame,
                    double first_radius, const Geom& second,
                    const GeomFrame& second_frame, double second_radius,
                    double margin, std::span<PreContact, MAX_PAIR_CONTACTS> out)
    -> std::uint32_t;

}  // namespace simon::model

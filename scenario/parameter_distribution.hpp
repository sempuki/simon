// Copyright 2026 -- CONTRIBUTORS. See LICENSE.
// Follows esmini 3.8.2, Copyright (c) partners of Simulation Scenarios,
// MPL-2.0; translated to C++ and changed. See NOTICE.md.

#pragma once

#include <cstddef>
#include <string>
#include <vector>

// A scenario's variations, as an ASAM OpenSCENARIO parameter value
// distribution describes them (see model/REFERENCES.md): the scenario file,
// and for each deterministic distribution the values it gives its
// parameters. Every combination of one choice from each distribution is a
// permutation; each runs the scenario with its values in place of the
// file's own declarations. format/openscenario reads it from files.
namespace simon::scenario {

// A value given to one of the scenario's parameters, as text.
struct ParameterAssignment final {
  std::string name;
  std::string value;
};

// One choice in a distribution: the values it gives, several for a
// multi-parameter distribution's value set, one otherwise.
using ParameterChoice = std::vector<ParameterAssignment>;

struct ParameterDistribution final {
  std::string scenario;  // The scenario file, as a path.
  std::vector<std::vector<ParameterChoice>> distributions;
};

// The number of permutations: the product of each distribution's choices,
// none without a distribution.
auto count_permutations(const ParameterDistribution& distribution)
    -> std::size_t;

// The values of permutation `index`, numbered from 0 with the last
// distribution varying fastest, as esmini numbers them; each distribution's
// values in order.
auto find_permutation(const ParameterDistribution& distribution,
                      std::size_t index) -> std::vector<ParameterAssignment>;

}  // namespace simon::scenario

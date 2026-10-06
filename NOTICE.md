# Notices

simon is licensed under the GNU General Public License, version 3 (see
`LICENSE`). Parts of it are translations into C++ of other open-source
projects' code, or follow that code's order of operations so that results
match it. Each such file says so in its first lines and names the project.
This file keeps those projects' copyright notices and terms, as their
licenses require. simon's translations are changed from the originals: they
are rewritten in C++ with simon's types, names and structure.

Every project below is licensed compatibly with GPL-3.0 for this use. The
projects simon only compares itself with, the data it vendors and the
libraries it builds on are credited in each application's `README.md`, in
`3rd_party/README.md` and in the top-level `README.md`.

## MuJoCo

Copyright 2021 DeepMind Technologies Limited. Licensed under the Apache
License, Version 2.0 (https://www.apache.org/licenses/LICENSE-2.0).
https://github.com/google-deepmind/mujoco, version 3.14.0.

Followed by `format/mjcf.{hpp,cpp}`, `model/articulated*.{hpp,cpp}` and
`application/robotic/simulation_systems.{hpp,cpp}`.

## CommonRoad vehicle models

Copyright 2020 Technical University of Munich, Professorship of
Cyber-Physical Systems. https://gitlab.lrz.de/tum-cps/commonroad-vehicle-models,
version 3.0.2. Followed by `model/multibody.{hpp,cpp}`,
`model/single_track.{hpp,cpp}`, `model/tire.{hpp,cpp}` and
`model/vehicle.hpp`.

> BSD 3-Clause License
>
> Redistribution and use in source and binary forms, with or without
> modification, are permitted provided that the following conditions are met:
>
> 1. Redistributions of source code must retain the above copyright notice,
>    this list of conditions and the following disclaimer.
> 2. Redistributions in binary form must reproduce the above copyright
>    notice, this list of conditions and the following disclaimer in the
>    documentation and/or other materials provided with the distribution.
> 3. Neither the name of the copyright holder nor the names of its
>    contributors may be used to endorse or promote products derived from
>    this software without specific prior written permission.
>
> THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
> AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
> IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
> ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
> LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
> CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
> SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
> INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
> CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
> ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
> POSSIBILITY OF SUCH DAMAGE.

## SciPy

Copyright (c) 2001-2002 Enthought, Inc. 2003, SciPy Developers. All rights
reserved. https://scipy.org, version 1.18. `savgol_filter`'s handling of
edges and even windows is followed by `model/driving_metrics.{hpp,cpp}`,
under the BSD 3-Clause License, whose conditions and disclaimer are as given
for CommonRoad above.

## nuPlan devkit

Copyright 2021 Motional. Licensed under the Apache License, Version 2.0.
https://github.com/motional/nuplan-devkit, version 1.2.2. Its comfort
metrics and time to collision are followed by
`model/driving_metrics.{hpp,cpp}` and
`application/automotive/scenario_batch.cpp`.

## esmini

Copyright (c) partners of Simulation Scenarios. Licensed under the Mozilla
Public License, Version 2.0 (https://mozilla.org/MPL/2.0/), whose section 3.3
lets the parts of simon that follow it be distributed under GPL-3.0; esmini
is not marked "Incompatible With Secondary Licenses". Its source is at
https://github.com/esmini/esmini, version 3.8.2. Followed by
`model/polyline.{hpp,cpp}`, `scenario/storyboard.{hpp,cpp}`,
`scenario/parameter_distribution.hpp`, `scenario/transition.hpp`,
`format/openscenario.cpp` and
`application/automotive/simulation_systems.{hpp,cpp}`.

## Eclipse SUMO

Copyright (C) 2001-2026 German Aerospace Center (DLR) and others. Licensed
under EPL-2.0 OR GPL-2.0-or-later; simon uses it under GPL-2.0-or-later,
taken as GPL-3.0. https://eclipse.dev/sumo, version 1.27.1. `MSCFModel_IDM`
and `MSLink` are followed by `model/traffic_control.{hpp,cpp}` and
`model/right_of_way.cpp`.

## MovSim

Copyright (C) 2010, 2011, 2012 by Arne Kesting, Martin Treiber, Ralph Germ,
Martin Budden. Licensed under the GNU General Public License, version 3 or
(at your option) any later version. https://github.com/movsim/movsim. Its
approach to yellow lights, `TrafficLightApproaching`, is followed by
`model/traffic_control.{hpp,cpp}`.

## JSBSim

Copyright (C) 2000 Jon S. Berndt and the JSBSim authors. Licensed under the
GNU Lesser General Public License, version 2 or (at your option) any later
version, as its source files state.
https://github.com/JSBSim-Team/jsbsim, version 1.3.1. The behavior of its
components (FGTurbine, the flight control components, FGMassBalance,
FGPropulsion, FGTrim and FGAuxiliary) is followed by `model/turbine`,
`model/flight_control`, `model/mass_balance`, `model/propulsion`,
`model/trim`, `model/atmosphere`, `model/sensing` and
`model/rigid_aircraft`.

## REBOUND

Copyright (c) 2011 Hanno Rein, Shangfei Liu. Licensed under the GNU General
Public License, version 3. https://github.com/hannorein/rebound,
version 5.2.2. Its direct summation, `reb_gravity_basic_calculate_acceleration`,
is followed in its order of operations by `model/gravity.cpp`.

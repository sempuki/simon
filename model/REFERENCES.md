# References

The sources of the math and the formats in `core/`, `model/`, `format/`,
`scenario/` and `framework/`, files named from `model/` but where a directory
is given. Each algorithm's comment
names its source in brief, and the full reference is here. Where a comment
says the math is derived here, it shows the derivation. A source is given in
full where it first appears, and as "above" after.

## Rotations and rigid motions

| Where | What | Source |
|---|---|---|
| `core/lie.hpp` | SO(3) and SE(3) as Lie groups: the exponential and logarithm, the adjoint, the left and right Jacobians and their inverses | J. Solà, J. Deray and D. Atchuthan, "A micro Lie theory for state estimation in robotics", arXiv:1812.01537, 2018, sections 4 and 7 and appendices A and B |
| `core/lie.hpp` | Twists rotation first, and the motion and force cross products, the bracket ad and its dual | R. Featherstone, *Rigid Body Dynamics Algorithms*, Springer, 2008, sections 2.9 and 2.10; K. M. Lynch and F. C. Park, *Modern Robotics*, Cambridge University Press, 2017, section 8.2 |
| `core/lie.hpp` | The rotation of a vector about an axis, as a quaternion | O. Rodrigues, "Des lois géométriques qui régissent les déplacements d'un système solide dans l'espace", *Journal de Mathématiques Pures et Appliquées* 5, 1840, pp. 380-440 |
| `core/lie.hpp` | The block of the SE(3) Jacobian that couples translation to rotation | T. D. Barfoot, *State Estimation for Robotics*, Cambridge University Press, 2017, equation 7.86 |

## The Earth and the air

| Where | What | Source |
|---|---|---|
| `earth/atmosphere.hpp` | The standard atmosphere, layered by geopotential altitude | *U.S. Standard Atmosphere, 1976*, NOAA, NASA and USAF, NOAA-S/T 76-1562, 1976 |
| `earth/atmosphere.hpp` | Calibrated airspeed: a pitot tube's impact pressure, isentropic below Mach 1 and behind a normal shock (Rayleigh) above it, read back at sea level | W. M. Olson, *Aircraft Performance Flight Testing*, AFFTC-TIH-99-01, USAF Air Force Flight Test Center, 2000, sections 4.6 to 4.8, equations 4.12 and 4.17; H. W. Liepmann and A. E. Puckett, *Introduction to Aerodynamics of a Compressible Fluid*, Wiley, 1947, for the Rayleigh pitot formula; both as JSBSim 1.3.1's FGAuxiliary cites and implements them |
| `earth/earth.hpp` | The WGS84 ellipsoid, its constants and its rotation | *Department of Defense World Geodetic System 1984*, NGA.STND.0036_1.0.0_WGS84, National Geospatial-Intelligence Agency, 2014, which keeps the defining constants of NIMA TR8350.2, 3rd edition, amendment 1, 2000 |
| `earth/earth.hpp` | Gravitation to the J2 zonal harmonic | D. A. Vallado, *Fundamentals of Astrodynamics and Applications*, 4th edition, Microcosm Press, 2013, on the central body's gravity field |
| `earth/earth.hpp` | Geodetic latitude and altitude from ECEF, in closed form | M. Heikkinen, "Geschlossene Formeln zur Berechnung räumlicher geodätischer Koordinaten aus rechtwinkligen Koordinaten", *Zeitschrift für Vermessungswesen* 107, 1982, pp. 207-211 |
| `earth/earth_test.cpp` | Geodetic latitude by fixed-point iteration, the independent check | B. R. Bowring, "Transformation from spatial to geographical coordinates", *Survey Review* 23(181), 1976, pp. 323-327 |
| `aircraft/frames.hpp` | The rate at which a body keeps level over the ellipsoid: the north-east-down frame's transport rate, from the radii of curvature | D. H. Titterton and J. L. Weston, *Strapdown Inertial Navigation Technology*, 2nd edition, IEE and AIAA, 2004 |
| `earth/wind.hpp` | Turbulence intensities and scale lengths, and the Dryden spectra of the linear and angular gusts | *Military Specification: Flying Qualities of Piloted Airplanes*, MIL-F-8785C, 1980, section 3.7 |
| `earth/wind.cpp` | The air's rotation in gusts: q = -dw/dx and r = dv/dx, the negative of MIL-F-8785's rotary gusts, which perturb the aircraft's rates relative to the air | M. M. Madden, "Verifying implementation of the Dryden turbulence model and MIL-F-8785 gust gradient", AIAA Modeling and Simulation Technologies Conference, 2018, equations 6 to 8; MIL-F-8785C, above |
| `earth/wind.cpp` | Sampling a filter driven by white noise exactly over a step | C. F. Van Loan, "Computing integrals involving the matrix exponential", *IEEE Transactions on Automatic Control* 23(3), 1978, pp. 395-404 |
| `earth/wind.cpp` | A lag whose input moves in a straight line over a step | The triangle-hold equivalent, G. F. Franklin, J. D. Powell and M. L. Workman, *Digital Control of Dynamic Systems*, 3rd edition, Addison-Wesley, 1998, section 6.3.2 |
| `core/random.hpp` | SplitMix64 | G. L. Steele Jr., D. Lea and C. H. Flood, "Fast splittable pseudorandom number generators", OOPSLA 2014, pp. 453-472; its constants as in S. Vigna's [splitmix64.c](https://prng.di.unimi.it/splitmix64.c) |
| `core/random.hpp` | Normal numbers from uniform ones | G. E. P. Box and M. E. Muller, "A note on the generation of random normal deviates", *Annals of Mathematical Statistics* 29(2), 1958, pp. 610-611 |

## Aircraft

| Where | What | Source |
|---|---|---|
| `rigid_body.hpp` | Six-degree-of-freedom equations of motion in an inertial frame, with a quaternion attitude | B. L. Stevens and F. L. Lewis, *Aircraft Control and Simulation*, 2nd edition, Wiley, 2003, section 1.5 |
| `aircraft/frames.hpp` | The rate of a body's air velocity in body axes | Derived from Stevens and Lewis's equations of motion, above, by the rule for a vector's rate in a turning frame |
| `aircraft/sensing.cpp` | The acceleration of a point fixed in a rigid body: the pilot's eye | Stevens and Lewis, above, section 1.2, velocity and acceleration in moving frames; JSBSim 1.3.1's FGAuxiliary |
| `aircraft/aerodynamics.hpp` | Forces from wind axes to body axes | Stevens and Lewis, above, section 2.3 |
| `aircraft/aerodynamics.hpp` | Coefficient build-up of aerodynamic forces and moments | J. S. Berndt and the JSBSim Development Team, [*JSBSim Reference Manual*](https://jsbsim.sourceforge.net/JSBSimReferenceManual.pdf), version 1.0, 2011, on aerodynamics |
| `aircraft/turbine.hpp` | A turbine's spools, thrust and fuel flow, and reheat lit by the throttle | The JSBSim Reference Manual, above, on the turbine; JSBSim 1.3.1's FGTurbine |
| `aircraft/flight_control.hpp` | Flight control blocks: summers, gains, scheduled gains, surface scales, kinematic actuators, switches, PIDs and functions | The JSBSim Reference Manual, above, on flight control components; JSBSim 1.3.1's FGSummer, FGGain, FGKinemat, FGSwitch, FGPID and FGFCSFunction |
| `aircraft/rigid_aircraft.cpp` | The rate of angle of attack from the rate of the air velocity | The derivative of atan2(w, u), as JSBSim 1.3.1's FGAuxiliary has it |
| `aircraft/mass_balance.hpp` | Inertia about the center of mass: each mass's terms by the inertia tensor's definition, which for the empty aircraft's own inertia is the parallel axis theorem in tensor form | H. Goldstein, C. Poole and J. Safko, *Classical Mechanics*, 3rd edition, Addison-Wesley, 2002, section 5.3; JSBSim 1.3.1's FGMassBalance |
| `aircraft/propulsion.hpp` | Fuel drawn from each engine's feed tanks in equal shares | JSBSim 1.3.1's FGPropulsion |
| `aircraft/trim.hpp` | A trim's unknowns, each paired with the acceleration it balances | JSBSim 1.3.1's full trim, FGTrim |
| `aircraft/trim.hpp` | The pitch that gives a flight-path angle at an angle of attack and bank, with no sideslip | The rate-of-climb constraint, Stevens and Lewis, above, section 3.6, steady-state flight |
| `aircraft/trim.hpp` | Newton's method with a finite-difference Jacobian | J. E. Dennis and R. B. Schnabel, *Numerical Methods for Unconstrained Optimization and Nonlinear Equations*, SIAM, 1996, section 5.4 |
| `aircraft/flight_path.hpp` | Point-mass flight-path equations: speed, flight-path angle and heading under load factor, bank and thrust | D. G. Hull, *Fundamentals of Airplane Flight Mechanics*, Springer, 2007, chapter 2, three-dimensional flight |
| `aircraft/flight_path.hpp` | A drag polar whose least drag falls at a lift other than zero | D. P. Raymer, *Aircraft Design: A Conceptual Approach*, 6th edition, AIAA, 2018, section 12.6 |
| `guidance.hpp` | True proportional navigation: acceleration across the line of sight | P. Zarchan, *Tactical and Strategic Missile Guidance*, 6th edition, AIAA, 2012, chapter 2; P. K. Shukla and P. R. Mahapatra, "The proportional navigation dilemma: pure or true?", *IEEE Transactions on Aerospace and Electronic Systems* 26(2), 1990, pp. 382-392, for the name |

## Numerics and control

| Where | What | Source |
|---|---|---|
| `control.hpp` | Exact discretization of a first-order lag | The zero-order-hold equivalent, Franklin, Powell and Workman, above, section 6.3.1 |
| `control.hpp` | A PI controller that stops integrating at a limit, against windup | K. J. Åström and T. Hägglund, *Advanced PID Control*, ISA, 2006, section 3.5 |
| `aircraft/flight_path.hpp` | The single pass: speed and angles first, then position along the new velocity | Symplectic Euler, E. Hairer, C. Lubich and G. Wanner, *Geometric Numerical Integration*, 2nd edition, Springer, 2006, section I.1.2 |
| `kinematics.hpp` | The midpoint method | E. Hairer, S. P. Nørsett and G. Wanner, *Solving Ordinary Differential Equations I*, 2nd edition, Springer, 1993, section II.1 |
| `framework/continuous.hpp` | Euler, midpoint and Runge-Kutta 4 as Butcher tableaus | J. C. Butcher, *Numerical Methods for Ordinary Differential Equations*, 3rd edition, Wiley, 2016 |
| `road/road.cpp` | Gauss-Legendre quadrature with 8 points: the roots of the Legendre polynomial and their weights | F. W. J. Olver et al., eds., [*NIST Digital Library of Mathematical Functions*](https://dlmf.nist.gov), section 3.5(v); the values to double precision by mpmath, and as M. Abramowitz and I. A. Stegun, *Handbook of Mathematical Functions*, NBS, 1964, table 25.4, print them to 15 digits |
| `road/road_test.cpp` | The Fresnel integrals, for the clothoid | The NIST Digital Library of Mathematical Functions, above, section 7.2(iii); C(1) and S(1) to double precision by mpmath, and as Abramowitz and Stegun, above, table 7.7, print them to 7 digits |
| `road/road.cpp` | Turning the road's normal about its tangent by the superelevation | Rodrigues' rotation formula, Goldstein, Poole and Safko, above, section 4.7 |
| `application/hello/hello.cpp` | Contact between balls as a linear spring and dashpot | P. A. Cundall and O. D. L. Strack, "A discrete numerical model for granular assemblies", *Géotechnique* 29(1), 1979, pp. 47-65 |
| `application/hello/hello.cpp` | A linear spring-dashpot contact's overlap over time, and the restitution it gives | T. Schwager and T. Pöschel, "Coefficient of restitution and linear-dashpot model revisited", *Granular Matter* 9, 2007, pp. 465-469 |
| `application/hello/hello.hpp` | Semi-implicit Euler: contact forces taken at the positions after moving | Symplectic Euler, Hairer, Lubich and Wanner, above, section I.1.2 |

## Roads, vehicles and tires

| Where | What | Source |
|---|---|---|
| `road/road.hpp` | Roads: the reference line's geometries, elevation, superelevation, lane offset, lane sections and widths, and road coordinates; signals, objects and their outlines in road and local coordinates (heading, pitch and roll, z-y'-x''), signal controllers, and junction priorities and controllers | *ASAM OpenDRIVE*, version 1.8, ASAM e.V., 2023 |
| `road/placement.cpp` | Moving at t from a reference line of curvature kappa: s changes by the distance over 1 - kappa t | Derived here, from an arc's length at radius 1 / kappa - t |
| `vehicle/single_track.hpp` | The kinematic single-track model | M. Althoff and G. Würsching, "CommonRoad: Vehicle Models", version 2020a, Technical University of Munich, 2020, and its code, commonroad-vehicle-models 3.0.2 |
| `vehicle/vehicle.hpp` | Vehicles' steering and acceleration limits, and the parameters of CommonRoad's vehicles 1, 2 and 3; their multibody parameters, CommonRoad's table 6, from R. W. Allen et al., *Vehicle Dynamic Stability and Rollover*, DOT HS 807 956, US Department of Transportation, 1992, table E-5 | Althoff and Würsching, above |
| `vehicle/tire.hpp` | The tire's coefficients, CommonRoad's table 7, from MSC Software's *Adams/Tire* help, 2011, on PAC2002 | Althoff and Würsching, above |
| `vehicle/tire.cpp` | The Magic Formula 5.2, as PAC2002, for pure and combined slip, its scaling factors 1 and turn slip left out, in CommonRoad's subset | H. B. Pacejka, *Tyre and Vehicle Dynamics*, 1st edition, Butterworth-Heinemann, 2002, chapter 4; E. Kuiper and J. J. M. van Oosten, "The PAC2002 advanced handling tire model", *Vehicle System Dynamics* 45(S1), 2007, pp. 153-167; Althoff and Würsching, above |
| `format/tire_file.cpp` | The TNO tire property file (.tir) in the PAC2002 format: its sections, keys and coefficients | Kuiper and van Oosten, above; as Project Chrono 10.0.0's ChPac02Tire reads it |
| `vehicle/tire.cpp` | Mirroring a tire about its wheel plane for the other side | Derived here: forces at (kappa, -alpha, -gamma), the lateral force and aligning moment turned; Chrono flips the same asymmetric coefficients |
| `vehicle/single_track.cpp` | The dynamic single-track model, its tires linear in slip angle with load transfer, and the drift model, with Magic Formula tires, wheel spin, and its blend into the kinematic model at a crawl | Althoff and Würsching, above, sections 7 and 8 |
| `vehicle/single_track.cpp` | The slip angle's rate at a crawl, beta = atan(tan(delta) b / l) | Derived here, by the chain rule; the comment shows it |
| `vehicle/multibody.cpp` | The multibody model: sprung body, unsprung axles, suspension, compliant roll-axis pins, tire springs and camber, and wheel spin | Althoff and Würsching, above, section 9, after Allen et al.'s vehicle dynamics model, above |
| `vehicle/multibody.cpp`, `vehicle/single_track.cpp` | Each wheel's forces turned by its own steer, and the tires' aligning moments, in the yaw moment about the center of gravity | Derived here, as the moments of each wheel's forces about the center of gravity |
| `application/automotive/maneuver_test.cpp` | Steady-state circular driving and the understeer gradient | *Passenger cars: Steady-state circular driving behaviour, Open-loop test methods*, ISO 4138:2021 |
| `application/automotive/maneuver_test.cpp` | The step steer and its yaw rate response time and overshoot | *Road vehicles: Lateral transient response test methods, Open-loop test methods*, ISO 7401:2011 |
| `application/automotive/maneuver_test.cpp` | The sine with dwell, its yaw rate ratios 1 s and 1.75 s after the steer and its lateral displacement 1.07 s after it begins | *Federal Motor Vehicle Safety Standard No. 126: Electronic stability control systems*, 49 CFR 571.126, S5.2 |
| `application/automotive/maneuver_test.cpp` | The Sedan the models are checked against | R. Serban, M. Taylor, D. Negrut and A. Tasora, "Chrono::Vehicle: template-based ground vehicle modelling and simulation", *International Journal of Vehicle Performance* 5(1), 2019, pp. 18-39; Project Chrono 10.0.0 |

## Traffic

| Where | What | Source |
|---|---|---|
| `traffic/traffic.hpp` | The Intelligent Driver Model | M. Treiber, A. Hennecke and D. Helbing, "Congested traffic states in empirical observations and microscopic simulations", *Physical Review E* 62(2), 2000, pp. 1805-1824; in the form of M. Treiber and A. Kesting, *Traffic Flow Dynamics*, Springer, 2013, chapter 11 |
| `traffic/traffic.hpp` | MOBIL, the lane-changing model, with a bias toward the right lane | A. Kesting, M. Treiber and D. Helbing, "General lane-changing model MOBIL for car-following models", *Transportation Research Record* 1999, 2007, pp. 86-94; Treiber and Kesting, above, chapter 14 |
| `traffic/signals.cpp` | Stopping for a yellow light, unless braking to the line by the IDM reaches 4 m/s^2 or a stop at 6 m/s^2 is impossible | movsim's `TrafficLightApproaching`, github.com/movsim/movsim at 7fe4162 (GPL-3.0), core/src/main/java/org/movsim/simulator/vehicles/longitudinalmodel/TrafficLightApproaching.java |
| `traffic/signals.cpp` | Braking for a stop: the IDM behind a standing leader with no minimum gap, stopping at once within 1 cm; and stopping 1 m short of a signal's line | SUMO 1.27.1 (EPL-2.0 OR GPL-2.0-or-later), `MSCFModel_IDM::stopSpeed` in src/microsim/cfmodels/MSCFModel_IDM.cpp, and the vehicle type's `jmStoplineGap`, 1 m by default |
| `traffic/right_of_way.cpp` | Who gives way: the junction's `<priority>`, give-way and stop signs (catalog 205 and 206), a left turn to oncoming traffic, and otherwise to the right | *ASAM OpenDRIVE*, version 1.8, above; Germany's *Straßenverkehrs-Ordnung*, §8 (priority to the right) and §9 (turning left gives way to oncoming traffic) |
| `traffic/right_of_way.cpp` | Gap acceptance: a minor driver enters a gap if the next vehicle with priority is at least the critical gap away when it reaches the junction | W. Harders, *Die Leistungsfähigkeit nicht signalgeregelter städtischer Verkehrsknoten*, Straßenbau und Straßenverkehrstechnik 76, 1968; Transportation Research Board, *Highway Capacity Manual*, 7th edition, 2022, chapter 20, two-way stop control |
| `traffic/right_of_way.cpp` | Merging lanes meet where their middles come within a car's width, and a vehicle on a merging lane leads the other's driver | SUMO 1.27.1 (EPL-2.0 OR GPL-2.0-or-later), `MSLink` in src/microsim/MSLink.cpp: a lane with the same target has its conflict where the two diverge, and `getLeaderInfo` returns vehicles on foe lanes as leaders |
| `road/walking_graph.cpp` | The walking graph from sidewalk lanes and crosswalk objects | *ASAM OpenDRIVE*, version 1.8, above |
| `road/walking_graph.cpp` | Shortest routes | E. W. Dijkstra, "A note on two problems in connexion with graphs", *Numerische Mathematik* 1, 1959, pp. 269-271 |
| `application/automotive/simulation.hpp` | Free walking speeds: normal, mean 1.34 m/s, standard deviation 0.26 m/s | U. Weidmann, *Transporttechnik der Fussgänger*, Schriftenreihe des IVT 90, ETH Zürich, 1993 |
| `application/automotive/simulation_systems.hpp` | A pedestrian's critical gap at an unsignalized crossing, t_c = L / S_p + t_s, its length over the walking speed plus a start-up time; checked against the delay where drivers do not yield, (e^(v t_c) - v t_c - 1) / v | Transportation Research Board, *Highway Capacity Manual*, 7th edition, 2022, chapter 20, two-way stop control, pedestrian mode; after D. Adams, "Road traffic considered as a random series", *Journal of the Institution of Civil Engineers* 4, 1936, pp. 121-130 |
| `application/automotive/crossing_test.cpp` | A pedestrian's delay at a signalized crossing, (C - g)^2 / 2C, for pedestrians arriving evenly | Transportation Research Board, *Highway Capacity Manual*, 7th edition, 2022, chapter 19, signalized intersections, pedestrian mode |

## Scenarios

| Where | What | Source |
|---|---|---|
| `scenario/openscenario.hpp`, `format/openscenario.hpp` | Scenarios: parameters and expressions, catalogs, positions, actions, conditions, triggers and the storyboard | *ASAM OpenSCENARIO XML*, version 1.3, ASAM e.V., 2024; expressions as version 1.1 defines them |
| `scenario/storyboard.hpp` | The storyboard's element states and transitions, event priorities, condition edges and delays | ASAM OpenSCENARIO, above; where it leaves the order open, esmini 3.8.2's ScenarioEngine |
| `scenario/storyboard.cpp` | Distances in conditions: straight and signed, along or across the entity's heading, or along or across its road | ASAM OpenSCENARIO, above, RelativeDistanceType and CoordinateSystem; esmini 3.8.2's Object::Distance |
| `scenario/storyboard.cpp` | Traffic signal controllers: phases in turn, a delay after the reference controller's first phase, a controller action entering a phase, and the phase condition | ASAM OpenSCENARIO, above, TrafficSignalController, TrafficSignalControllerAction and TrafficSignalControllerCondition |
| `model/road/polyline.cpp` | Following a trajectory's polyline held to its line: straight between vertices, each heading along its next segment, the heading blended across a corner within 2 m or half the segment | esmini 3.8.2's PolyLineShape::CalculatePolyLine and PolyLineBase::EvaluateSegmentByLocalS, with corner interpolation |
| `model/road/placement.cpp` | A route's roads: the shortest way by lane length between its waypoints on the lane graph | E. W. Dijkstra, above; ASAM OpenSCENARIO, above, Route and its shortest strategy |
| `scenario/transition.hpp` | The transition shapes, step, linear, cubic and sinusoidal, and their peak rates | ASAM OpenSCENARIO, above, TransitionDynamics; esmini's OSCPrivateAction for the peak rates |
| `scenario/parameter_distribution.hpp` | Deterministic parameter value distributions and their permutations | ASAM OpenSCENARIO, above, ParameterValueDistribution; esmini 3.8.2's OSCParameterDistribution for the permutations' order |

## Measures of a run

| Where | What | Source |
|---|---|---|
| `collision.hpp` | Overlap of two boxes by the separating axis theorem | C. Ericson, *Real-Time Collision Detection*, Morgan Kaufmann, 2005, sections 4.4.1 and 5.2.1; S. Gottschalk, M. C. Lin and D. Manocha, "OBBTree: A hierarchical structure for rapid interference detection", SIGGRAPH 1996, pp. 171-180, where the test for boxes comes from |
| `collision.hpp` | The gap between two boxes apart: the least distance from a corner of either to an edge of the other | Ericson, above, section 5.1.2, the distance of a point to a segment |
| `traffic/driving_metrics.hpp` | Comfort, its signals and bounds, and time to collision | H. Caesar et al., "nuPlan: A closed-loop ML-based planning benchmark for autonomous vehicles", arXiv:2106.11810, 2021; the metrics and state extractors of the nuplan-devkit repository at its v1.2.2 tag |
| `traffic/driving_metrics.cpp` | The Savitzky-Golay filter and its derivatives | A. Savitzky and M. J. E. Golay, "Smoothing and differentiation of data by simplified least squares procedures", *Analytical Chemistry* 36(8), 1964, pp. 1627-1639; SciPy 1.18's `savgol_filter` for the edges and even windows |

The aeronautic application's references, which compare simon with JSBSim, are
described in `application/aeronautic/reference/README.md`.

## Gravity

| Where | What | Source |
|---|---|---|
| `gravity/gravity.hpp` | The Newtonian constant of gravitation, G = 6.67430 x 10^-11 m^3 kg^-1 s^-2 | E. Tiesinga, P. J. Mohr, D. B. Newell and B. N. Taylor, "CODATA recommended values of the fundamental physical constants: 2018", *Reviews of Modern Physics* 93, 025010, 2021 |
| `gravity/gravity.hpp` | The astronomical unit, exactly 149,597,870,700 m | IAU 2012 Resolution B2 |
| `gravity/gravity.hpp` | The parsec, exactly 648000 / pi astronomical units | IAU 2015 Resolution B2 |
| `gravity/gravity.hpp` | The nominal solar mass parameter, GM = 1.3271244 x 10^20 m^3 s^-2 | A. Prša et al., "Nominal values for selected solar and planetary quantities: IAU 2015 Resolution B3", *The Astronomical Journal* 152(2), 41, 2016 |
| `gravity/gravity.hpp` | Plummer softening of gravity between bodies, and the force error it trades for | W. Dehnen, "Towards optimal softening in three-dimensional N-body codes – I. Minimizing the force error", *MNRAS* 324, 2001, pp. 273-291 |
| `gravity/gravity.cpp` | Direct summation, in REBOUND's order of operations | H. Rein and S.-F. Liu, "REBOUND: an open-source multi-purpose N-body code for collisional dynamics", *Astronomy & Astrophysics* 537, A128, 2012; REBOUND 5.2.2, src/gravity.c (reb_gravity_basic_calculate_acceleration) |
| `gravity/gravity.cpp` | Barnes and Hut's tree: cubic cells split into octants until each holds one body, an unopened cell pulling as a point at its center of mass, cells opened when their width exceeds the opening angle times the distance | J. Barnes and P. Hut, "A hierarchical O(N log N) force-calculation algorithm", *Nature* 324, 1986, pp. 446-449; the opening test as REBOUND 5.2.2's tree has it, src/tree.c |
| `gravity/galaxy.cpp` | A parabolic orbit's position over time: Barker's equation, solved in closed form | Vallado, above, on Barker's equation and the parabolic form of Kepler's equation |
| `gravity/galaxy.cpp`, `application/galactic/simulation.cpp` | Restricted encounters: two masses on a parabolic orbit, one with a disk of 120 test particles in rings of 12, 18, 24, 30 and 36 at 0.2 to 0.6 of the pericenter distance, from 10^9 years before pericenter | A. Toomre and J. Toomre, "Galactic bridges and tails", *The Astrophysical Journal* 178, 1972, pp. 623-666, section II |
| `gravity/galaxy.cpp` | Disk galaxies as bodies: an exponential disk sech^2 thick (2.1); the halo's isotropic dispersion from the Jeans equation in the halo's and disk's mass (2.14), its speeds Gaussian and below 0.95 of escape; the disk's sigma_R^2 proportional to exp(-R / h) (2.21), sigma_z^2 = pi G Sigma z_0 (2.22), sigma_R set by Q at a reference radius (2.23), kappa (2.24), sigma_phi^2 = sigma_R^2 kappa^2 / 4 Omega^2 (2.26) and the mean rotation from asymmetric drift (2.28) | L. Hernquist, "N-body realizations of compound galaxies", *The Astrophysical Journal Supplement Series* 86, 1993, pp. 389-400, section 2 |
| `gravity/galaxy.cpp` | The halo's profile, M(r) = M r^2 / (r + a)^2, used in place of Hernquist's (1993) isothermal halo | L. Hernquist, "An analytical model for spherical galaxies and bulges", *The Astrophysical Journal* 356, 1990, pp. 359-364 |
| `gravity/galaxy.cpp` | A thin exponential disk's circular speed, 4 pi G Sigma_0 h y^2 [I0 K0 - I1 K1](y), y = R / 2h | K. C. Freeman, "On the disks of spiral and S0 galaxies", *The Astrophysical Journal* 160, 1970, pp. 811-830 |
| `gravity/galaxy.cpp` | A stellar disk's critical radial dispersion, 3.36 G Sigma / kappa, and Q | A. Toomre, "On the gravitational stability of a disk of stars", *The Astrophysical Journal* 139, 1964, pp. 1217-1238 |
| `application/galactic/simulation_systems.hpp` | The leapfrog in kick-drift-kick form | V. Springel, "The cosmological simulation code GADGET-2", *MNRAS* 364, 2005, pp. 1105-1134; Hairer, Lubich and Wanner, above |
| `gravity/galaxy.hpp` | Plummer's sphere: its density, mass profile and energy | H. C. Plummer, "On the problem of distribution in globular star clusters", *MNRAS* 71(5), 1911, pp. 460-470 |
| `gravity/galaxy.cpp` | Sampling Plummer's sphere: radii from the mass profile, speeds by von Neumann's rejection from q^2 (1 - q^2)^(7/2), directions uniform on the sphere | S. J. Aarseth, M. Hénon and R. Wielen, "A comparison of numerical methods for the study of star cluster dynamics", *Astronomy & Astrophysics* 37, 1974, pp. 183-187, appendix, equations A1 to A6 |
| `gravity/galaxy.hpp` | The crossing time G M^(5/2) / (-2 E)^(3/2) | D. C. Heggie and R. D. Mathieu, "Standardised units and time scales", in *The Use of Supercomputers in Stellar Dynamics*, Lecture Notes in Physics 267, Springer, 1986, p. 233 |
| `application/galactic/gravity_test.cpp` | Kepler's equation, and the position and velocity on an orbit from the eccentric anomaly | C. D. Murray and S. F. Dermott, *Solar System Dynamics*, Cambridge University Press, 1999, chapter 2 |

## Articulated bodies

| Where | What | Source |
|---|---|---|
| `articulated/articulated.hpp` | Kinematic trees of rigid bodies on joints in generalized coordinates, their shapes, actuators and options, as a compiled model holds them | E. Todorov, T. Erez and Y. Tassa, "MuJoCo: A physics engine for model-based control", IROS 2012; MuJoCo 3.14.0 (Apache-2.0), its documentation's Modeling and Computation chapters and mjModel |
| `format/mjcf.cpp` | Compiling MJCF: default classes, orientations, fromto, geom volumes and moments, inertia combined and put on principal axes, limits from ranges, positions at rest | MuJoCo 3.14.0's compiler, src/user/user_objects.cc, user_util.cc and user_model.cc, in their order of operations |
| `format/mjcf.cpp` | Principal axes of inertia by Jacobi rotations, each the symmetric Schur decomposition of the largest off-diagonal element | G. H. Golub and C. F. Van Loan, *Matrix Computations*, 4th edition, Johns Hopkins, 2013, section 8.5; as MuJoCo's mjuu_eig3 keeps them, as a quaternion |
| `articulated/arithmetic.hpp` | Quaternion, three-vector and 3x3 matrix arithmetic: products, rotations, normalization | MuJoCo 3.14.0, src/engine/engine_util_spatial.c, engine_util_blas.c and engine_inline.h, in their order of operations |
| `articulated/dynamics.hpp` | Forward kinematics, inertia and motion at the center of mass, the composite rigid body algorithm, LDLᵀ along the tree, recursive Newton–Euler | R. Featherstone, *Rigid Body Dynamics Algorithms*, Springer, 2008, chapters 4 to 6; as MuJoCo 3.14.0 computes them, src/engine/engine_core_smooth.c and engine_util_spatial.c |
| `articulated/dynamics.hpp` | Semi-implicit Euler with dampers taken implicitly, M + h B | MuJoCo 3.14.0, src/engine/engine_forward.c, mj_Euler, and its documentation's Computation chapter, Numerical integration |
| `articulated/collision.cpp` | Which bodies and geoms may touch, bounding spheres, contact parameters mixed by priority and solmix, contact frames | MuJoCo 3.14.0, src/engine/engine_collision_driver.c (filterBodyPair, mj_filterSphere, mj_contactParam, mj_setContact) and engine_util_spatial.c (mju_makeFrame) |
| `articulated/collision.cpp` | Plane, sphere, capsule and cylinder colliders | MuJoCo 3.14.0, src/engine/engine_collision_primitive.c, in its order of operations |
| `articulated/collision.cpp` | Sphere and capsule against a box; box against box by the separating axis test and the incident face clipped to the reference face | S. Gottschalk, M. C. Lin and D. Manocha, "OBBTree: a hierarchical structure for rapid interference detection", SIGGRAPH 1996; I. E. Sutherland and G. W. Hodgman, "Reentrant polygon clipping", CACM 17(1), 1974; as MuJoCo 3.14.0 computes them, src/engine/engine_collision_box.c |
| `articulated/convex.cpp` | Distance between convex shapes from their support functions, its simplex subalgorithm by signed volumes | E. G. Gilbert, D. W. Johnson and S. S. Keerthi, "A fast procedure for computing the distance between complex objects in three-dimensional space", IEEE Journal of Robotics and Automation 4(2), 1988; M. Montanari, N. Petrinic and E. Barbieri, "Improving the GJK algorithm for faster and more reliable distance queries between convex objects", ACM Transactions on Graphics 36(3), 2017; as MuJoCo 3.14.0 computes it, src/engine/engine_collision_gjk.c |
| `articulated/convex.cpp` | Penetration depth by the expanding polytope, contacts where faces meet by clipping one against the other, more by turning the geoms | G. van den Bergen, "Proximity queries and penetration depth computation on 3D game objects", GDC 2001; I. E. Sutherland and G. W. Hodgman, "Reentrant polygon clipping", CACM 17(1), 1974; as MuJoCo 3.14.0 computes them, src/engine/engine_collision_gjk.c and engine_collision_convex.c (mjc_Convex, mjc_PlaneConvex) |
| `articulated/constraint.cpp` | Soft constraints: each row's impedance from solimp, stiffness and damping from solref, regularization from the inverse inertia at rest; the convex cost in the accelerations | E. Todorov, "Convex and analytically-invertible dynamics with contacts and constraints: Theory and implementation in MuJoCo", ICRA 2014; MuJoCo 3.14.0, src/engine/engine_core_constraint.c (mj_instantiateLimit, mj_instantiateContact, mj_diagApprox, mj_makeImpedance, mj_referenceConstraint, mj_constraintUpdate) and engine_setconst.c |
| `articulated/constraint.cpp` | Newton's method in the accelerations, its exact line search on the piecewise quadratic cost | J. Nocedal and S. J. Wright, *Numerical Optimization*, 2nd edition, Springer, 2006, chapters 3 and 6; as MuJoCo 3.14.0 computes it, src/engine/engine_solver.c (mj_solPrimal, PrimalSearch) |
| `articulated/constraint.cpp` | Projected Gauss–Seidel on the dual, swept in a shuffled order with momentum | Y. Nesterov, "A method of solving a convex programming problem with convergence rate O(1/k²)", Soviet Mathematics Doklady 27, 1983; B. O'Donoghue and E. Candès, "Adaptive restart for accelerated gradient schemes", Foundations of Computational Mathematics 15, 2015; M. E. O'Neill, "PCG: A family of simple fast space-efficient statistically good algorithms for random number generation", HMC-CS-2014-0905, 2014; as MuJoCo 3.14.0 computes it, src/engine/engine_solver.c (solPGS) |
| `application/robotic/simulation_systems.cpp` | Islands of trees joined by constraints, by union and find | R. E. Tarjan, "Efficiency of a good but not linear set union algorithm", JACM 22(2), 1975; as MuJoCo 3.14.0 forms them, src/engine/engine_island.c |
| `articulated/articulated.hpp`, `application/robotic/simulation_systems.cpp` | Fixed tendons: a length summed from joints' positions, its Jacobian, its limits and dry friction, and its inverse inertia at rest | MuJoCo 3.14.0, src/engine/engine_core_smooth.c (mj_tendon), engine_core_constraint.c (mj_instantiateLimit, mj_instantiateFriction) and engine_setconst.c |

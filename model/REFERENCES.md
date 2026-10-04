# References

The sources of the math and the formats in `model/`, `format/`, `scenario/`
and `framework/`, files named from `model/` but where a directory is given. Each algorithm's comment
names its source in brief, and the full reference is here. Where a comment
says the math is derived here, it shows the derivation. A source is given in
full where it first appears, and as "above" after.

## The Earth and the air

| Where | What | Source |
|---|---|---|
| `atmosphere.hpp` | The standard atmosphere, layered by geopotential altitude | *U.S. Standard Atmosphere, 1976*, NOAA, NASA and USAF, NOAA-S/T 76-1562, 1976 |
| `atmosphere.hpp` | Calibrated airspeed: a pitot tube's impact pressure, isentropic below Mach 1 and behind a normal shock (Rayleigh) above it, read back at sea level | W. M. Olson, *Aircraft Performance Flight Testing*, AFFTC-TIH-99-01, USAF Air Force Flight Test Center, 2000, sections 4.6 to 4.8, equations 4.12 and 4.17; H. W. Liepmann and A. E. Puckett, *Introduction to Aerodynamics of a Compressible Fluid*, Wiley, 1947, for the Rayleigh pitot formula; both as JSBSim 1.3.1's FGAuxiliary cites and implements them |
| `earth.hpp` | The WGS84 ellipsoid, its constants and its rotation | *Department of Defense World Geodetic System 1984*, NGA.STND.0036_1.0.0_WGS84, National Geospatial-Intelligence Agency, 2014, which keeps the defining constants of NIMA TR8350.2, 3rd edition, amendment 1, 2000 |
| `earth.hpp` | Gravitation to the J2 zonal harmonic | D. A. Vallado, *Fundamentals of Astrodynamics and Applications*, 4th edition, Microcosm Press, 2013, on the central body's gravity field |
| `earth.hpp` | Geodetic latitude and altitude from ECEF, in closed form | M. Heikkinen, "Geschlossene Formeln zur Berechnung räumlicher geodätischer Koordinaten aus rechtwinkligen Koordinaten", *Zeitschrift für Vermessungswesen* 107, 1982, pp. 207-211 |
| `earth_test.cpp` | Geodetic latitude by fixed-point iteration, the independent check | B. R. Bowring, "Transformation from spatial to geographical coordinates", *Survey Review* 23(181), 1976, pp. 323-327 |
| `frames.hpp` | The rate at which a body keeps level over the ellipsoid: the north-east-down frame's transport rate, from the radii of curvature | D. H. Titterton and J. L. Weston, *Strapdown Inertial Navigation Technology*, 2nd edition, IEE and AIAA, 2004 |
| `wind.hpp` | Turbulence intensities and scale lengths, and the Dryden spectra of the linear and angular gusts | *Military Specification: Flying Qualities of Piloted Airplanes*, MIL-F-8785C, 1980, section 3.7 |
| `wind.cpp` | The air's rotation in gusts: q = -dw/dx and r = dv/dx, the negative of MIL-F-8785's rotary gusts, which perturb the aircraft's rates relative to the air | M. M. Madden, "Verifying implementation of the Dryden turbulence model and MIL-F-8785 gust gradient", AIAA Modeling and Simulation Technologies Conference, 2018, equations 6 to 8; MIL-F-8785C, above |
| `wind.cpp` | Sampling a filter driven by white noise exactly over a step | C. F. Van Loan, "Computing integrals involving the matrix exponential", *IEEE Transactions on Automatic Control* 23(3), 1978, pp. 395-404 |
| `wind.cpp` | A lag whose input moves in a straight line over a step | The triangle-hold equivalent, G. F. Franklin, J. D. Powell and M. L. Workman, *Digital Control of Dynamic Systems*, 3rd edition, Addison-Wesley, 1998, section 6.3.2 |
| `wind.cpp`, `application/automotive/simulation_systems.hpp` | SplitMix64 | G. L. Steele Jr., D. Lea and C. H. Flood, "Fast splittable pseudorandom number generators", OOPSLA 2014, pp. 453-472; its constants as in S. Vigna's [splitmix64.c](https://prng.di.unimi.it/splitmix64.c) |
| `wind.cpp`, `random.hpp` | Normal numbers from uniform ones | G. E. P. Box and M. E. Muller, "A note on the generation of random normal deviates", *Annals of Mathematical Statistics* 29(2), 1958, pp. 610-611 |

## Aircraft

| Where | What | Source |
|---|---|---|
| `rigid_body.hpp` | Six-degree-of-freedom equations of motion in an inertial frame, with a quaternion attitude | B. L. Stevens and F. L. Lewis, *Aircraft Control and Simulation*, 2nd edition, Wiley, 2003, section 1.5 |
| `frames.hpp` | The rate of a body's air velocity in body axes | Derived from Stevens and Lewis's equations of motion, above, by the rule for a vector's rate in a turning frame |
| `sensing.cpp` | The acceleration of a point fixed in a rigid body: the pilot's eye | Stevens and Lewis, above, section 1.2, velocity and acceleration in moving frames; JSBSim 1.3.1's FGAuxiliary |
| `aerodynamics.hpp` | Forces from wind axes to body axes | Stevens and Lewis, above, section 2.3 |
| `aerodynamics.hpp` | Coefficient build-up of aerodynamic forces and moments | J. S. Berndt and the JSBSim Development Team, [*JSBSim Reference Manual*](https://jsbsim.sourceforge.net/JSBSimReferenceManual.pdf), version 1.0, 2011, on aerodynamics |
| `turbine.hpp` | A turbine's spools, thrust and fuel flow, and reheat lit by the throttle | The JSBSim Reference Manual, above, on the turbine; JSBSim 1.3.1's FGTurbine |
| `flight_control.hpp` | Flight control blocks: summers, gains, scheduled gains, surface scales, kinematic actuators, switches, PIDs and functions | The JSBSim Reference Manual, above, on flight control components; JSBSim 1.3.1's FGSummer, FGGain, FGKinemat, FGSwitch, FGPID and FGFCSFunction |
| `rigid_aircraft.cpp` | The rate of angle of attack from the rate of the air velocity | The derivative of atan2(w, u), as JSBSim 1.3.1's FGAuxiliary has it |
| `mass_balance.hpp` | Inertia about the center of mass: each mass's terms by the inertia tensor's definition, which for the empty aircraft's own inertia is the parallel axis theorem in tensor form | H. Goldstein, C. Poole and J. Safko, *Classical Mechanics*, 3rd edition, Addison-Wesley, 2002, section 5.3; JSBSim 1.3.1's FGMassBalance |
| `propulsion.hpp` | Fuel drawn from each engine's feed tanks in equal shares | JSBSim 1.3.1's FGPropulsion |
| `trim.hpp` | A trim's unknowns, each paired with the acceleration it balances | JSBSim 1.3.1's full trim, FGTrim |
| `trim.hpp` | The pitch that gives a flight-path angle at an angle of attack and bank, with no sideslip | The rate-of-climb constraint, Stevens and Lewis, above, section 3.6, steady-state flight |
| `trim.hpp` | Newton's method with a finite-difference Jacobian | J. E. Dennis and R. B. Schnabel, *Numerical Methods for Unconstrained Optimization and Nonlinear Equations*, SIAM, 1996, section 5.4 |
| `flight_path.hpp` | Point-mass flight-path equations: speed, flight-path angle and heading under load factor, bank and thrust | D. G. Hull, *Fundamentals of Airplane Flight Mechanics*, Springer, 2007, chapter 2, three-dimensional flight |
| `flight_path.hpp` | A drag polar whose least drag falls at a lift other than zero | D. P. Raymer, *Aircraft Design: A Conceptual Approach*, 6th edition, AIAA, 2018, section 12.6 |
| `guidance.hpp` | True proportional navigation: acceleration across the line of sight | P. Zarchan, *Tactical and Strategic Missile Guidance*, 6th edition, AIAA, 2012, chapter 2; P. K. Shukla and P. R. Mahapatra, "The proportional navigation dilemma: pure or true?", *IEEE Transactions on Aerospace and Electronic Systems* 26(2), 1990, pp. 382-392, for the name |

## Numerics and control

| Where | What | Source |
|---|---|---|
| `control.hpp` | Exact discretization of a first-order lag | The zero-order-hold equivalent, Franklin, Powell and Workman, above, section 6.3.1 |
| `control.hpp` | A PI controller that stops integrating at a limit, against windup | K. J. Åström and T. Hägglund, *Advanced PID Control*, ISA, 2006, section 3.5 |
| `flight_path.hpp` | The single pass: speed and angles first, then position along the new velocity | Symplectic Euler, E. Hairer, C. Lubich and G. Wanner, *Geometric Numerical Integration*, 2nd edition, Springer, 2006, section I.1.2 |
| `kinematics.hpp` | The midpoint method | E. Hairer, S. P. Nørsett and G. Wanner, *Solving Ordinary Differential Equations I*, 2nd edition, Springer, 1993, section II.1 |
| `framework/continuous.hpp` | Euler, midpoint and Runge-Kutta 4 as Butcher tableaus | J. C. Butcher, *Numerical Methods for Ordinary Differential Equations*, 3rd edition, Wiley, 2016 |
| `road.cpp` | Gauss-Legendre quadrature with 8 points: the roots of the Legendre polynomial and their weights | F. W. J. Olver et al., eds., [*NIST Digital Library of Mathematical Functions*](https://dlmf.nist.gov), section 3.5(v); the values to double precision by mpmath, and as M. Abramowitz and I. A. Stegun, *Handbook of Mathematical Functions*, NBS, 1964, table 25.4, print them to 15 digits |
| `road_test.cpp` | The Fresnel integrals, for the clothoid | The NIST Digital Library of Mathematical Functions, above, section 7.2(iii); C(1) and S(1) to double precision by mpmath, and as Abramowitz and Stegun, above, table 7.7, print them to 7 digits |
| `road.cpp` | Turning the road's normal about its tangent by the superelevation | Rodrigues' rotation formula, Goldstein, Poole and Safko, above, section 4.7 |

## Roads, vehicles and tires

| Where | What | Source |
|---|---|---|
| `road.hpp` | Roads: the reference line's geometries, elevation, superelevation, lane offset, lane sections and widths, and road coordinates; signals, objects and their outlines in road and local coordinates (heading, pitch and roll, z-y'-x''), signal controllers, and junction priorities and controllers | *ASAM OpenDRIVE*, version 1.8, ASAM e.V., 2023 |
| `road_placement.cpp` | Moving at t from a reference line of curvature kappa: s changes by the distance over 1 - kappa t | Derived here, from an arc's length at radius 1 / kappa - t |
| `single_track.hpp` | The kinematic single-track model | M. Althoff and G. Würsching, "CommonRoad: Vehicle Models", version 2020a, Technical University of Munich, 2020, and its code, commonroad-vehicle-models 3.0.2 |
| `vehicle.hpp` | Vehicles' steering and acceleration limits, and the parameters of CommonRoad's vehicles 1, 2 and 3; their multibody parameters, CommonRoad's table 6, from R. W. Allen et al., *Vehicle Dynamic Stability and Rollover*, DOT HS 807 956, US Department of Transportation, 1992, table E-5 | Althoff and Würsching, above |
| `tire.hpp` | The tire's coefficients, CommonRoad's table 7, from MSC Software's *Adams/Tire* help, 2011, on PAC2002 | Althoff and Würsching, above |
| `tire.cpp` | The Magic Formula 5.2, as PAC2002, for pure and combined slip, its scaling factors 1 and turn slip left out, in CommonRoad's subset | H. B. Pacejka, *Tyre and Vehicle Dynamics*, 1st edition, Butterworth-Heinemann, 2002, chapter 4; E. Kuiper and J. J. M. van Oosten, "The PAC2002 advanced handling tire model", *Vehicle System Dynamics* 45(S1), 2007, pp. 153-167; Althoff and Würsching, above |
| `format/tire_file.cpp` | The TNO tire property file (.tir) in the PAC2002 format: its sections, keys and coefficients | Kuiper and van Oosten, above; as Project Chrono 10.0.0's ChPac02Tire reads it |
| `tire.cpp` | Mirroring a tire about its wheel plane for the other side | Derived here: forces at (kappa, -alpha, -gamma), the lateral force and aligning moment turned; Chrono flips the same asymmetric coefficients |
| `single_track.cpp` | The dynamic single-track model, its tires linear in slip angle with load transfer, and the drift model, with Magic Formula tires, wheel spin, and its blend into the kinematic model at a crawl | Althoff and Würsching, above, sections 7 and 8 |
| `single_track.cpp` | The slip angle's rate at a crawl, beta = atan(tan(delta) b / l) | Derived here, by the chain rule; the comment shows it |
| `multibody.cpp` | The multibody model: sprung body, unsprung axles, suspension, compliant roll-axis pins, tire springs and camber, and wheel spin | Althoff and Würsching, above, section 9, after Allen et al.'s vehicle dynamics model, above |
| `multibody.cpp`, `single_track.cpp` | Each wheel's forces turned by its own steer, and the tires' aligning moments, in the yaw moment about the center of gravity | Derived here, as the moments of each wheel's forces about the center of gravity |
| `application/automotive/maneuver_test.cpp` | Steady-state circular driving and the understeer gradient | *Passenger cars: Steady-state circular driving behaviour, Open-loop test methods*, ISO 4138:2021 |
| `application/automotive/maneuver_test.cpp` | The step steer and its yaw rate response time and overshoot | *Road vehicles: Lateral transient response test methods, Open-loop test methods*, ISO 7401:2011 |
| `application/automotive/maneuver_test.cpp` | The sine with dwell, its yaw rate ratios 1 s and 1.75 s after the steer and its lateral displacement 1.07 s after it begins | *Federal Motor Vehicle Safety Standard No. 126: Electronic stability control systems*, 49 CFR 571.126, S5.2 |
| `application/automotive/maneuver_test.cpp` | The Sedan the models are checked against | R. Serban, M. Taylor, D. Negrut and A. Tasora, "Chrono::Vehicle: template-based ground vehicle modelling and simulation", *International Journal of Vehicle Performance* 5(1), 2019, pp. 18-39; Project Chrono 10.0.0 |

## Traffic

| Where | What | Source |
|---|---|---|
| `traffic.hpp` | The Intelligent Driver Model | M. Treiber, A. Hennecke and D. Helbing, "Congested traffic states in empirical observations and microscopic simulations", *Physical Review E* 62(2), 2000, pp. 1805-1824; in the form of M. Treiber and A. Kesting, *Traffic Flow Dynamics*, Springer, 2013, chapter 11 |
| `traffic.hpp` | MOBIL, the lane-changing model, with a bias toward the right lane | A. Kesting, M. Treiber and D. Helbing, "General lane-changing model MOBIL for car-following models", *Transportation Research Record* 1999, 2007, pp. 86-94; Treiber and Kesting, above, chapter 14 |
| `traffic_control.cpp` | Stopping for a yellow light, unless braking to the line by the IDM reaches 4 m/s^2 or a stop at 6 m/s^2 is impossible | movsim's `TrafficLightApproaching`, github.com/movsim/movsim at 7fe4162 (GPL-3.0), core/src/main/java/org/movsim/simulator/vehicles/longitudinalmodel/TrafficLightApproaching.java |
| `traffic_control.cpp` | Braking for a stop: the IDM behind a standing leader with no minimum gap, stopping at once within 1 cm; and stopping 1 m short of a signal's line | SUMO 1.27.1 (EPL-2.0), `MSCFModel_IDM::stopSpeed` in src/microsim/cfmodels/MSCFModel_IDM.cpp, and the vehicle type's `jmStoplineGap`, 1 m by default |
| `right_of_way.cpp` | Who gives way: the junction's `<priority>`, give-way and stop signs (catalog 205 and 206), a left turn to oncoming traffic, and otherwise to the right | *ASAM OpenDRIVE*, version 1.8, above; Germany's *Straßenverkehrs-Ordnung*, §8 (priority to the right) and §9 (turning left gives way to oncoming traffic) |
| `right_of_way.cpp` | Gap acceptance: a minor driver enters a gap if the next vehicle with priority is at least the critical gap away when it reaches the junction | W. Harders, *Die Leistungsfähigkeit nicht signalgeregelter städtischer Verkehrsknoten*, Straßenbau und Straßenverkehrstechnik 76, 1968; Transportation Research Board, *Highway Capacity Manual*, 7th edition, 2022, chapter 20, two-way stop control |
| `right_of_way.cpp` | Merging lanes meet where their middles come within a car's width, and a vehicle on a merging lane leads the other's driver | SUMO 1.27.1 (EPL-2.0), `MSLink` in src/microsim/MSLink.cpp: a lane with the same target has its conflict where the two diverge, and `getLeaderInfo` returns vehicles on foe lanes as leaders |
| `walking_graph.cpp` | The walking graph from sidewalk lanes and crosswalk objects | *ASAM OpenDRIVE*, version 1.8, above |
| `walking_graph.cpp` | Shortest routes | E. W. Dijkstra, "A note on two problems in connexion with graphs", *Numerische Mathematik* 1, 1959, pp. 269-271 |
| `application/automotive/simulation.hpp` | Free walking speeds: normal, mean 1.34 m/s, standard deviation 0.26 m/s | U. Weidmann, *Transporttechnik der Fussgänger*, Schriftenreihe des IVT 90, ETH Zürich, 1993 |

## Scenarios

| Where | What | Source |
|---|---|---|
| `scenario/openscenario.hpp`, `format/openscenario.hpp` | Scenarios: parameters and expressions, catalogs, positions, actions, conditions, triggers and the storyboard | *ASAM OpenSCENARIO XML*, version 1.3, ASAM e.V., 2024; expressions as version 1.1 defines them |
| `scenario/storyboard.hpp` | The storyboard's element states and transitions, event priorities, condition edges and delays | ASAM OpenSCENARIO, above; where it leaves the order open, esmini 3.8.2's ScenarioEngine |
| `scenario/storyboard.cpp` | Distances in conditions: straight and signed, along or across the entity's heading, or along or across its road | ASAM OpenSCENARIO, above, RelativeDistanceType and CoordinateSystem; esmini 3.8.2's Object::Distance |
| `scenario/transition.hpp` | The transition shapes, step, linear, cubic and sinusoidal, and their peak rates | ASAM OpenSCENARIO, above, TransitionDynamics; esmini's OSCPrivateAction for the peak rates |
| `scenario/parameter_distribution.hpp` | Deterministic parameter value distributions and their permutations | ASAM OpenSCENARIO, above, ParameterValueDistribution; esmini 3.8.2's OSCParameterDistribution for the permutations' order |

## Measures of a run

| Where | What | Source |
|---|---|---|
| `collision.hpp` | Overlap of two boxes by the separating axis theorem | C. Ericson, *Real-Time Collision Detection*, Morgan Kaufmann, 2005, sections 4.4.1 and 5.2.1; S. Gottschalk, M. C. Lin and D. Manocha, "OBBTree: A hierarchical structure for rapid interference detection", SIGGRAPH 1996, pp. 171-180, where the test for boxes comes from |
| `collision.hpp` | The gap between two boxes apart: the least distance from a corner of either to an edge of the other | Ericson, above, section 5.1.2, the distance of a point to a segment |
| `driving_metrics.hpp` | Comfort, its signals and bounds, and time to collision | H. Caesar et al., "nuPlan: A closed-loop ML-based planning benchmark for autonomous vehicles", arXiv:2106.11810, 2021; the metrics and state extractors of the nuplan-devkit repository at its v1.2.2 tag |
| `driving_metrics.cpp` | The Savitzky-Golay filter and its derivatives | A. Savitzky and M. J. E. Golay, "Smoothing and differentiation of data by simplified least squares procedures", *Analytical Chemistry* 36(8), 1964, pp. 1627-1639; SciPy 1.18's `savgol_filter` for the edges and even windows |

The aeronautic application's references, which compare simon with JSBSim, are
described in `application/aeronautic/reference/README.md`.

# References

The sources of the math and the formats in `model/`, `format/`, `scenario/`
and `framework/`, files named from `model/` but where a directory is given. Each algorithm's comment
names its source in brief, and the full reference is here. Where a comment
says the math is derived here, it shows the derivation.

| Where | What | Source |
|---|---|---|
| `atmosphere.hpp` | The standard atmosphere, layered by geopotential altitude | *U.S. Standard Atmosphere, 1976*, NOAA, NASA and USAF, NOAA-S/T 76-1562 |
| `atmosphere.hpp` | Calibrated airspeed: a pitot tube's impact pressure, isentropic below Mach 1 and behind a normal shock (Rayleigh) above it, read back at sea level | JSBSim's FGAuxiliary, [JSBSim Reference Manual](https://jsbsim-team.github.io/jsbsim-reference-manual/) |
| `earth.hpp` | The WGS84 ellipsoid, its constants and its rotation | *Department of Defense World Geodetic System 1984*, NIMA TR8350.2, 3rd edition, 2000 |
| `earth.hpp` | Gravitation to the J2 zonal harmonic | D. A. Vallado, *Fundamentals of Astrodynamics and Applications*, 4th edition, 2013, on zonal harmonics |
| `earth.hpp` | Geodetic latitude and altitude from ECEF, in closed form | M. Heikkinen, "Geschlossene Formeln zur Berechnung räumlicher geodätischer Koordinaten aus rechtwinkligen Koordinaten", *Zeitschrift für Vermessungswesen* 107, 1982 |
| `rigid_body.hpp` | Six-degree-of-freedom equations of motion in an inertial frame, with a quaternion attitude | B. L. Stevens and F. L. Lewis, *Aircraft Control and Simulation*, 2nd edition, Wiley, 2003, section 1.5 |
| `aerodynamics.hpp` | Coefficient build-up of aerodynamic forces and moments | JSBSim's aerodynamics model, [JSBSim Reference Manual](https://jsbsim-team.github.io/jsbsim-reference-manual/) |
| `turbine.hpp` | A turbine's spools, thrust and fuel flow, and reheat lit by the throttle | JSBSim's turbine model, FGTurbine, as the [JSBSim Reference Manual](https://jsbsim-team.github.io/jsbsim-reference-manual/) describes it |
| `flight_control.hpp` | Flight control blocks: summers, gains, scheduled gains, surface scales, kinematic actuators, switches, PIDs and functions | JSBSim's flight control components (FGSummer, FGGain, FGKinemat, FGSwitch, FGPID, FGFCSFunction), as the JSBSim Reference Manual describes them |
| `frames.hpp` | The rate at which a body keeps level over the ellipsoid: the north-east-down frame's transport rate, from the radii of curvature | D. H. Titterton and J. L. Weston, *Strapdown Inertial Navigation Technology*, 2nd edition, IET, 2004 |
| `trim.hpp` | A trim's unknowns, each paired with the acceleration it balances | JSBSim's full trim, FGTrim |
| `trim.hpp` | The pitch that gives a flight-path angle at an angle of attack and bank, with no sideslip | The rate-of-climb constraint of steady flight, Stevens and Lewis, above |
| `control.hpp` | Exact discretization of a first-order lag | The zero-order-hold equivalent, as in G. F. Franklin, J. D. Powell and M. Workman, *Digital Control of Dynamic Systems*, 3rd edition, 1998 |
| `flight_path.hpp` | Point-mass flight-path equations | Stevens and Lewis, above |
| `road.hpp` | Roads: the reference line's geometries, elevation, superelevation, lane offset, lane sections and widths, and road coordinates | *ASAM OpenDRIVE*, version 1.8, ASAM e.V., 2023 |
| `road.cpp` | Gauss-Legendre quadrature's nodes and weights | M. Abramowitz and I. A. Stegun, *Handbook of Mathematical Functions*, NBS, 1964, table 25.4 |
| `road_test.cpp` | The Fresnel integrals, for the clothoid | Abramowitz and Stegun, above, table 7.7 |
| `road.cpp` | Turning the road's normal about its tangent by the superelevation | Rodrigues' rotation formula, Goldstein, Poole and Safko, above, section 4.7 |
| `single_track.hpp` | The kinematic single-track model | M. Althoff and G. Wuersching, "CommonRoad: Vehicle Models", version 2020a, Technical University of Munich, 2020, and its code, commonroad-vehicle-models 3.0.2 |
| `vehicle.hpp` | Vehicles' steering and acceleration limits, and the parameters of CommonRoad's vehicles 1, 2 and 3, their multibody parameters from the US Department of Transportation's vehicle dynamics data (table 6) | Althoff and Wuersching, above |
| `tire.hpp` | The tire's coefficients, from the ADAMS handbook (table 7) | Althoff and Wuersching, above |
| `tire.cpp` | The Magic Formula 5.2 for pure and combined slip, its scaling factors 1 and turn slip left out, in CommonRoad's subset | H. B. Pacejka, *Tire and Vehicle Dynamics*, 3rd edition, Butterworth-Heinemann, 2012, chapter 4; Althoff and Wuersching, above |
| `single_track.cpp` | The dynamic single-track model, its tires linear in slip angle with load transfer, and the drift model, with Magic Formula tires, wheel spin, and its blend into the kinematic model at a crawl | Althoff and Wuersching, above |
| `single_track.cpp` | The slip angle's rate at a crawl, beta = atan(tan(delta) b / l) | Derived here, by the chain rule; the comment shows it |
| `format/tire_file.cpp` | The TNO tire property file (.tir) in the PAC2002 format: its sections, keys and coefficients | As MSC ADAMS and Project Chrono read it; Chrono's ChPac02Tire, version 10.0.0 |
| `tire.cpp` | Mirroring a tire about its wheel plane for the other side | Derived here: forces at (kappa, -alpha, -gamma), the lateral force and aligning moment turned; Chrono flips the same asymmetric coefficients |
| `multibody.cpp`, `single_track.cpp` | Each wheel's forces turned by its own steer, and the tires' aligning moments, in the yaw moment about the center of gravity | Derived here, as the moments of each wheel's forces about the center of gravity |
| `scenario/openscenario.hpp`, `format/openscenario.hpp` | Scenarios: parameters and expressions, catalogs, positions, actions, conditions, triggers and the storyboard | *ASAM OpenSCENARIO XML*, version 1.3, ASAM e.V., 2024; expressions as version 1.1 defines them |
| `scenario/storyboard.hpp` | The storyboard's element states and transitions, event priorities, condition edges and delays | ASAM OpenSCENARIO, above; where it leaves the order open, esmini 3.8.2's ScenarioEngine |
| `scenario/transition.hpp` | The transition shapes, step, linear, cubic and sinusoidal, and their peak rates | ASAM OpenSCENARIO, above, TransitionDynamics; esmini's OSCPrivateAction for the peak rates |
| `scenario/parameter_distribution.hpp` | Deterministic parameter value distributions and their permutations | ASAM OpenSCENARIO, above, ParameterValueDistribution; esmini 3.8.2's OSCParameterDistribution for the permutations' order |
| `scenario/storyboard.cpp` | Distances in conditions: straight and signed, along or across the entity's heading, or along or across its road | ASAM OpenSCENARIO, above, RelativeDistanceType and CoordinateSystem; esmini 3.8.2's Object::Distance |
| `collision.hpp` | Overlap of two boxes by the separating axis theorem, and the gap between them | S. Gottschalk, M. C. Lin and D. Manocha, "OBBTree: A hierarchical structure for rapid interference detection", SIGGRAPH 1996; C. Ericson, *Real-Time Collision Detection*, Morgan Kaufmann, 2005, chapter 4 |
| `driving_metrics.hpp` | Comfort, its signals and bounds, and time to collision | H. Caesar et al., "nuPlan: A closed-loop ML-based planning benchmark for autonomous vehicles", arXiv:2106.11810, 2021; nuplan-devkit 1.2.2's metrics and state extractors |
| `driving_metrics.cpp` | The Savitzky-Golay filter and its derivatives | A. Savitzky and M. J. E. Golay, "Smoothing and differentiation of data by simplified least squares procedures", *Analytical Chemistry* 36(8), 1964; SciPy's `savgol_filter` for the edges and even windows |
| `road_placement.cpp` | Moving at t from a reference line of curvature kappa: s changes by the distance over 1 - kappa t | Derived here, from an arc's length at radius 1 / kappa - t |
| `maneuver_test.cpp` | Steady-state circular driving and the understeer gradient | *Passenger cars: Steady-state circular driving behaviour, Open-loop test methods*, ISO 4138 |
| `maneuver_test.cpp` | The step steer and its yaw rate response time and overshoot | *Road vehicles: Lateral transient response test methods, Open-loop test methods*, ISO 7401 |
| `maneuver_test.cpp` | The sine with dwell, its yaw rate ratios 1 s and 1.75 s after the steer and its lateral displacement 1.07 s after it begins | *Federal Motor Vehicle Safety Standard No. 126: Electronic stability control systems*, 49 CFR 571.126 |
| `maneuver_test.cpp` | The Sedan the models are checked against | A. Tasora et al., "Chrono: An Open Source Multi-physics Dynamics Engine", *High Performance Computing in Science and Engineering*, LNCS 9611, Springer, 2016; Project Chrono 10.0.0's Chrono::Vehicle |
| `multibody.cpp` | The multibody model: sprung body, unsprung axles, suspension, compliant roll-axis pins, tire springs and camber, and wheel spin | Althoff and Wuersching, above, after the US Department of Transportation's vehicle dynamics model, as they give it |
| `traffic.hpp` | The Intelligent Driver Model | M. Treiber, A. Hennecke and D. Helbing, "Congested traffic states in empirical observations and microscopic simulations", *Physical Review E* 62(2), 2000; in the form of M. Treiber and A. Kesting, *Traffic Flow Dynamics*, Springer, 2013, chapter 11 |
| `traffic.hpp` | MOBIL, the lane-changing model, with a bias toward the right lane | A. Kesting, M. Treiber and D. Helbing, "General lane-changing model MOBIL for car-following models", *Transportation Research Record* 1999, 2007; Treiber and Kesting, above, chapter 14 |
| `wind.hpp` | Turbulence intensities and scale lengths, and the Dryden spectra of the linear and angular gusts | *Military Specification: Flying Qualities of Piloted Airplanes*, MIL-F-8785C, 1980, section 3.7 |
| `wind.hpp` | Sampling a filter driven by white noise exactly over a step | C. F. Van Loan, "Computing integrals involving the matrix exponential", *IEEE Transactions on Automatic Control* 23(3), 1978 |
| `wind.hpp` | The signs of the air's rotation in gusts | B. Etkin, *Dynamics of Atmospheric Flight*, Wiley, 1972 |
| `wind.hpp` | SplitMix64 | G. L. Steele, D. Lea and C. H. Flood, "Fast splittable pseudorandom number generators", OOPSLA 2014 |
| `wind.cpp` | Normal numbers from uniform ones | G. E. P. Box and M. E. Muller, "A note on the generation of random normal deviates", *Annals of Mathematical Statistics* 29(2), 1958 |
| `wind.cpp` | A lag whose input moves in a straight line over a step | The first-order-hold equivalent, Franklin, Powell and Workman, above |
| `frames.hpp` | The rate of a body's air velocity in body axes | Derived from Stevens and Lewis's equations of motion, above, by the rule for a vector's rate in a turning frame |
| `rigid_aircraft.cpp` | The rate of angle of attack from the rate of the air velocity | The derivative of atan2(w, u), as JSBSim's FGAuxiliary has it |
| `aerodynamics.hpp` | Forces from wind axes to body axes | Stevens and Lewis, above, section 2.3 |
| `sensing.cpp` | The acceleration of a point fixed in a rigid body: the pilot's eye | Stevens and Lewis, above, section 1.3; JSBSim's FGAuxiliary |
| `mass_balance.hpp` | Inertia about the center of mass by the parallel axis theorem | H. Goldstein, C. Poole and J. Safko, *Classical Mechanics*, 3rd edition, Addison-Wesley, 2002, section 5.3; JSBSim's FGMassBalance |
| `propulsion.hpp` | Fuel drawn from each engine's feed tanks in equal shares | JSBSim's FGPropulsion |
| `trim.hpp` | Newton's method with a finite-difference Jacobian | J. E. Dennis and R. B. Schnabel, *Numerical Methods for Unconstrained Optimization and Nonlinear Equations*, SIAM, 1996, chapter 5 |
| `flight_path.hpp` | A drag polar whose least drag falls at a lift other than zero | D. P. Raymer, *Aircraft Design: A Conceptual Approach*, 6th edition, AIAA, 2018, chapter 12 |
| `flight_path.hpp` | The single pass: speed and angles first, then position along the new velocity | Symplectic Euler, E. Hairer, C. Lubich and G. Wanner, *Geometric Numerical Integration*, 2nd edition, Springer, 2006, section I.1 |
| `kinematics.hpp` | The midpoint method | E. Hairer, S. P. Norsett and G. Wanner, *Solving Ordinary Differential Equations I*, 2nd edition, Springer, 1993, section II.1 |
| `framework/continuous.hpp` | Euler, midpoint and Runge-Kutta 4 as Butcher tableaus | J. C. Butcher, *Numerical Methods for Ordinary Differential Equations*, 3rd edition, Wiley, 2016 |
| `control.hpp` | A PI controller that stops integrating at a limit, against windup | K. J. Astrom and T. Hagglund, *Advanced PID Control*, ISA, 2006, section 3.5 |
| `guidance.hpp` | True proportional navigation | P. Zarchan, *Tactical and Strategic Missile Guidance*, 6th edition, AIAA, 2012, chapter 2 |

The aeronautic application's references, which compare simon with JSBSim, are
described in `application/aeronautic/reference/README.md`.

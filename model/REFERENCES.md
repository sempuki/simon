# References

The sources of the math in `model/` and `framework/`. Each algorithm's comment
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

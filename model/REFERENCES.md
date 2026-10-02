# References

The sources of the less obvious math in `model/`. Each algorithm's comment
names its source in brief, and the full reference is here.

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
| `rigid_aircraft.hpp` | The rate at which a body keeps level over the ellipsoid: the north-east-down frame's transport rate, from the radii of curvature | D. H. Titterton and J. L. Weston, *Strapdown Inertial Navigation Technology*, 2nd edition, IET, 2004 |
| `trim.hpp` | A trim's unknowns, each paired with the acceleration it balances | JSBSim's full trim, FGTrim |
| `trim.hpp` | The pitch that gives a flight-path angle at an angle of attack and bank, with no sideslip | The rate-of-climb constraint of steady flight, Stevens and Lewis, above |
| `control.hpp` | Exact discretization of a first-order lag | The zero-order-hold equivalent, as in G. F. Franklin, J. D. Powell and M. Workman, *Digital Control of Dynamic Systems*, 3rd edition, 1998 |
| `flight_path.hpp` | Point-mass flight-path equations | Stevens and Lewis, above |

The flight application's references, which compare simon with JSBSim, are
described in `application/flight/reference/README.md`.

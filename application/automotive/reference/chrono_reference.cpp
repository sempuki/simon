// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

// Records Project Chrono's Magic Formula tire and its Sedan through the ISO
// handling maneuvers, for tire_test and maneuver_test.
//
// Project Chrono (https://projectchrono.org, BSD-3-Clause) models vehicles as
// multibody systems: Chrono::Vehicle's Sedan has a double wishbone front
// suspension, a multilink rear, a rack and pinion, front-wheel drive through
// an engine map and automatic transmission, and Pac02 tires, Chrono's Magic
// Formula 5.2, read from Sedan_Pac02Tire.tir. It drives on flat, rigid
// ground of friction 0.8, the tire's own. This program writes:
//
//   chrono_tires.csv      the Pac02 tire's forces and aligning moment at
//                         slips, slip angles, cambers and loads, combined by
//                         Pacejka's coefficients and by Chrono's friction
//                         ellipsis, its default
//   chrono_sedan.csv      the Sedan's mass, inertia, center of mass, wheel
//                         positions and wheel radius, as Chrono assembles it
//   chrono_maneuvers.csv  also rest, the Sedan braked at rest, pressed down
//                         and rolled by known loads, to measure its
//                         suspension; every sample has each wheel's load,
//                         height in the chassis's frame, camber and steer
//   chrono_maneuvers.csv  the Sedan through the maneuvers, every 0.01 s:
//                         ramp100, a steering ramp at 100 km/h (ISO 4138);
//                         ramp80, the same at 80 km/h (FMVSS 126's slowly
//                         increasing steer); step, a step steer at 80 km/h
//                         (ISO 7401); and dwell25 and dwell50, sines with
//                         dwell at 80 km/h (FMVSS 126)
//
// Chrono 10.0.0, built with its vehicle module, and this file built beside
// it with CMake:
//
//   find_package(Chrono COMPONENTS Vehicle CONFIG)
//   add_executable(chrono_reference chrono_reference.cpp)
//   target_link_libraries(chrono_reference PRIVATE ${CHRONO_TARGETS})
//
//   chrono_reference <chrono data directory> <tire file> <output directory>

#include <cmath>
#include <fstream>
#include <functional>
#include <iostream>
#include <vector>
#include <numbers>
#include <string>

#include "chrono/core/ChDataPath.h"
#include "chrono_models/vehicle/sedan/Sedan.h"
#include "chrono_vehicle/ChVehicleDataPath.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/wheeled_vehicle/tire/ChPac02Tire.h"

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::vehicle::sedan;

namespace {

constexpr double STEP = 1e-3;     // s.
constexpr double SAMPLE = 0.01;   // s.
constexpr double FRICTION = 0.8;  // The tire's own, so its scale is 1.

// Chrono's Pac02 tire, its force calculation opened to a table.
class TireProbe final : public ChPac02Tire {
 public:
  explicit TireProbe(const std::string& file) : ChPac02Tire("probe") {
    SetMFParamsByFile(file);
  }

  // Fx, Fy and Mz at kappa, alpha, Fz and gamma, combined by Pacejka's
  // coefficients or by the friction ellipsis.
  void Compute(double kappa, double alpha, double load, double gamma,
               bool ellipsis, double& fx, double& fy, double& mz) {
    m_use_mode = 4;
    m_use_friction_ellipsis = ellipsis;
    m_states.mu_scale = 1.0;
    m_states.Fz0_prime = m_par.FNOMIN * m_par.LFZO;
    m_states.dfz0 = (load - m_states.Fz0_prime) / m_states.Fz0_prime;
    m_states.dpi = 0.0;
    CalcFxyMz(fx, fy, mz, kappa, alpha, load, gamma);
  }

  double GetTireMass() const override { return 1.0; }
  ChVector3d GetTireInertia() const override { return ChVector3d(1, 1, 1); }
  void SetMFParams() override {}
};

void WriteTires(const std::string& file, const std::string& out) {
  TireProbe probe(file);
  std::ofstream csv(out);
  csv.precision(17);
  csv << "load,kappa,alpha,gamma,fx,fy,mz,ellipsis_fx,ellipsis_fy\n";
  // A grid inside the range where Chrono's clamp on B x leaves the formula
  // alone, and beyond it.
  for (double load : {1500.0, 3000.0, 4850.0, 7000.0}) {
    for (double kappa = -0.3; kappa <= 0.3001; kappa += 0.025) {
      for (double alpha = -0.25; alpha <= 0.2501; alpha += 0.025) {
        for (double gamma : {0.0, 0.03}) {
          double fx = 0, fy = 0, mz = 0, ex = 0, ey = 0, ez = 0;
          probe.Compute(kappa, alpha, load, gamma, false, fx, fy, mz);
          probe.Compute(kappa, alpha, load, gamma, true, ex, ey, ez);
          csv << load << ',' << kappa << ',' << alpha << ',' << gamma << ','
              << fx << ',' << fy << ',' << mz << ',' << ex << ',' << ey
              << '\n';
        }
      }
    }
  }
}

// One maneuver: the steering input in [-1, 1] at a time, and the speed held
// until a time, after which the throttle is released.
struct Maneuver {
  std::string name;
  double speed = 0.0;  // m/s.
  double end = 0.0;    // s.
  double hold_until = 0.0;
  std::function<double(double)> steering;
  // For the tests at rest: braked throughout, with a vertical force at the
  // center of mass and a roll moment on the chassis at each time.
  bool braked = false;
  std::function<ChVector3d(double)> load = [](double) { return VNULL; };
};

// A wheel's camber, its top's lean outward from the chassis's z axis.
double Camber(const ChWheeledVehicle& vehicle, int axle, VehicleSide side) {
  ChVector3d spin = vehicle.GetRot().RotateBack(
      vehicle.GetSpindleRot(axle, side).GetAxisY());
  return side == LEFT ? -std::asin(spin.z()) : std::asin(spin.z());
}

// The angle of a wheel on `axle` and `side` from the chassis's x axis,
// about z.
double SteerAngle(const ChWheeledVehicle& vehicle, VehicleSide side,
                  int axle = 0) {
  ChQuaternion<> chassis = vehicle.GetRot();
  ChQuaternion<> spindle = vehicle.GetSpindleRot(axle, side);
  ChVector3d spin = chassis.RotateBack(spindle.GetAxisY());
  return std::atan2(-spin.x(), spin.y());
}

// Drives `maneuver`, writing its samples, and returns the steering input at
// which v r first reaches 0.3 g, if it does.
double Drive(const Maneuver& maneuver, std::ofstream& csv, bool write_sedan,
             const std::string& sedan_out) {
  Sedan sedan;
  sedan.SetContactMethod(ChContactMethod::SMC);
  sedan.SetChassisCollisionType(CollisionType::NONE);
  sedan.SetChassisFixed(false);
  sedan.SetInitPosition(ChCoordsys<>(ChVector3d(0, 0, 0.4), QUNIT));
  sedan.SetInitFwdVel(maneuver.speed);
  sedan.SetTireType(TireModelType::PAC02);
  sedan.SetTireStepSize(STEP);
  sedan.Initialize();
  ChWheeledVehicle& vehicle = sedan.GetVehicle();

  RigidTerrain terrain(sedan.GetSystem());
  auto material = chrono_types::make_shared<ChContactMaterialSMC>();
  material->SetFriction(static_cast<float>(FRICTION));
  material->SetRestitution(0.01f);
  material->SetYoungModulus(2e7f);
  terrain.AddPatch(material, ChCoordsys<>(ChVector3d(0, 0, 0), QUNIT), 4000,
                   4000);
  terrain.Initialize();

  if (write_sedan) {
    std::ofstream out(sedan_out);
    out.precision(17);
    const ChFrame<>& com = vehicle.GetCOMFrame();
    const ChMatrix33<>& inertia = vehicle.GetInertia();
    out << "name,value\n";
    out << "mass," << vehicle.GetMass() << '\n';
    out << "com_x," << com.GetPos().x() << '\n';
    out << "com_y," << com.GetPos().y() << '\n';
    out << "com_z," << com.GetPos().z() << '\n';
    out << "inertia_xx," << inertia(0, 0) << '\n';
    out << "inertia_yy," << inertia(1, 1) << '\n';
    out << "inertia_zz," << inertia(2, 2) << '\n';
    out << "inertia_xz," << inertia(0, 2) << '\n';
    for (int axle = 0; axle < 2; ++axle) {
      for (VehicleSide side : {LEFT, RIGHT}) {
        ChVector3d at = vehicle.GetTransform().TransformPointParentToLocal(
            vehicle.GetSpindlePos(axle, side));
        std::string name = std::string(axle == 0 ? "front" : "rear") +
                           (side == LEFT ? "_left" : "_right");
        out << name << "_x," << at.x() << '\n';
        out << name << "_y," << at.y() << '\n';
        out << name << "_z," << at.z() << '\n';
      }
    }
    out << "wheel_radius," << vehicle.GetTire(0, LEFT)->GetRadius() << '\n';
  }

  auto chassis = vehicle.GetChassisBody();
  unsigned int accumulator = chassis->AddAccumulator();
  double integral = 0.0;
  double next_sample = 0.0;
  double at_03g = 0.0;
  for (double time = 0.0; time < maneuver.end + 0.5 * STEP; time += STEP) {
    // Speed held by a PI controller on throttle and brakes until the hold
    // ends, then the throttle is released.
    double speed = vehicle.GetSpeed();
    DriverInputs inputs{maneuver.steering(time), 0.0, 0.0};
    if (maneuver.braked) {
      inputs.m_braking = 1.0;
    } else if (time < maneuver.hold_until) {
      double error = maneuver.speed - speed;
      integral += error * STEP;
      double command = 0.4 * error + 0.2 * integral;
      inputs.m_throttle = std::clamp(command, 0.0, 1.0);
      inputs.m_braking = std::clamp(-command, 0.0, 1.0);
    }

    if (time >= next_sample - 0.5 * STEP) {
      next_sample += SAMPLE;
      ChVector3d com = vehicle.GetCOMFrame().GetPos();
      ChVector3d position = vehicle.GetPointLocation(com);
      ChVector3d velocity =
          vehicle.GetTransform().TransformDirectionParentToLocal(
              vehicle.GetPointVelocity(com));
      ChVector3d rate = vehicle.GetChassisBody()->GetAngVelLocal();
      ChVector3d angles = vehicle.GetRot().GetCardanAnglesXYZ();
      double heading = std::atan2(vehicle.GetRot().GetAxisX().y(),
                                  vehicle.GetRot().GetAxisX().x());
      double left = SteerAngle(vehicle, LEFT);
      double right = SteerAngle(vehicle, RIGHT);
      csv << maneuver.name << ',' << time << ',' << inputs.m_steering << ','
          << left << ',' << right << ',' << position.x() << ','
          << position.y() << ',' << position.z() << ',' << heading << ','
          << velocity.x() << ',' << velocity.y() << ',' << rate.z() << ','
          << angles.x() << ',' << inputs.m_throttle << ','
          << inputs.m_braking;
      for (int axle = 0; axle < 2; ++axle) {
        for (VehicleSide side : {LEFT, RIGHT}) {
          double load = vehicle.GetTire(axle, side)
                            ->ReportTireForce(&terrain)
                            .force.z();
          double height = vehicle.GetTransform()
                              .TransformPointParentToLocal(
                                  vehicle.GetSpindlePos(axle, side))
                              .z();
          csv << ',' << load << ',' << height << ','
              << Camber(vehicle, axle, side) << ','
              << SteerAngle(vehicle, side, axle);
        }
      }
      csv << '\n';
      if (at_03g == 0.0 && velocity.x() * rate.z() >= 0.3 * 9.81) {
        at_03g = inputs.m_steering;
      }
    }

    ChVector3d load = maneuver.load(time);
    chassis->EmptyAccumulator(accumulator);
    chassis->AccumulateForce(accumulator, ChVector3d(0, 0, load.z()),
                             vehicle.GetPointLocation(
                                 vehicle.GetCOMFrame().GetPos()),
                             false);
    chassis->AccumulateTorque(accumulator, ChVector3d(load.x(), 0, 0), true);

    terrain.Synchronize(time);
    sedan.Synchronize(time, inputs, terrain);
    terrain.Advance(STEP);
    sedan.Advance(STEP);
  }
  return at_03g;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 4) {
    std::cerr << "chrono_reference <chrono data> <tire file> <output>\n";
    return 1;
  }
  std::string data = argv[1];
  std::string out = argv[3];
  SetChronoDataPath(data + "/");
  SetVehicleDataPath(data + "/vehicle/");
  WriteTires(argv[2], out + "/chrono_tires.csv");

  constexpr double TAU = 2.0 * std::numbers::pi;
  constexpr double SETTLE = 5.0;  // s, straight at speed before each.
  auto ramp = [](double t) {
    return t < SETTLE ? 0.0 : 0.0025 * (t - SETTLE);
  };
  auto step = [](double t) {
    return t < SETTLE ? 0.0 : std::min(1.0, (t - SETTLE) / 0.1) * 0.06;
  };
  // A sine with dwell of `amplitude`: 0.7 Hz, three quarters of a period,
  // half a second's dwell at the second peak, and the last quarter.
  auto dwell = [TAU](double amplitude) {
    return [TAU, amplitude](double t) {
      constexpr double FREQUENCY = 0.7;
      double quarter = 0.25 / FREQUENCY;
      double s = t - SETTLE;
      if (s < 0.0) {
        return 0.0;
      }
      if (s < 3.0 * quarter) {
        return amplitude * std::sin(TAU * FREQUENCY * s);
      }
      if (s < 3.0 * quarter + 0.5) {
        return -amplitude;
      }
      if (s < 4.0 * quarter + 0.5) {
        return amplitude * std::sin(TAU * FREQUENCY * (s - 0.5));
      }
      return 0.0;
    };
  };
  std::ofstream csv(out + "/chrono_maneuvers.csv");
  csv.precision(8);
  csv << "maneuver,time,steering_input,left_steer,right_steer,x,y,z,heading,"
         "vx,vy,yaw_rate,roll,throttle,braking";
  for (std::string wheel : {"lf", "rf", "lr", "rr"}) {
    csv << ",load_" << wheel << ",height_" << wheel << ",camber_" << wheel
        << ",steer_" << wheel;
  }
  csv << '\n';
  // At rest and braked: settled, pressed down by 3 kN at the center of mass,
  // released, and rolled by 2 kN m, 3 s each.
  Drive({.name = "rest",
         .speed = 0.0,
         .end = 12.0,
         .hold_until = 0.0,
         .steering = [](double) { return 0.0; },
         .braked = true,
         .load =
             [](double t) {
               if (t >= 3.0 && t < 6.0) {
                 return ChVector3d(0, 0, -3000.0);
               }
               if (t >= 9.0) {
                 return ChVector3d(2000.0, 0, 0);
               }
               return VNULL;
             }},
        csv, false, "");
  // Inputs on the steering's normalized range, which Chrono's rack maps to
  // the wheels; the wheels' angles are recorded. The ramps are ISO 4138's
  // and FMVSS 126's slowly increasing steer; the step, ISO 7401's, to about
  // 4 m/s^2; the sines with dwell, FMVSS 126's, at 2.5 and 5 times the
  // input that holds 0.3 g on the ramp at 80 km/h.
  Drive({"ramp100", 100.0 / 3.6, SETTLE + 60.0, SETTLE + 60.0, ramp}, csv,
        true, out + "/chrono_sedan.csv");
  double at_03g = Drive({"ramp80", 80.0 / 3.6, SETTLE + 60.0, SETTLE + 60.0,
                         ramp},
                        csv, false, "");
  Drive({"step", 80.0 / 3.6, SETTLE + 5.0, SETTLE + 5.0, step}, csv, false,
        "");
  for (double multiple : {2.5, 5.0}) {
    Drive({"dwell" + std::to_string(static_cast<int>(multiple * 10)),
           80.0 / 3.6, SETTLE + 7.0, SETTLE, dwell(multiple * at_03g)},
          csv, false, "");
  }
  std::cout << "0.3 g at steering input " << at_03g << std::endl;
}

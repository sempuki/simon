// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <cmath>
#include <functional>
#include <map>
#include <string>
#include <string_view>
#include <vector>

#include "application/automotive/simulation.hpp"
#include "application/testing.hpp"
#include "base/testing.hpp"
#include "core/units.hpp"
#include "model/collision.hpp"
#include "model/vehicle/vehicle.hpp"

// Shared by the automotive tests: the reference tables in
// application/automotive/reference, read as rows of text by column name.
namespace simon::automotive::testing {

using simon::testing::load_table;
using simon::testing::parse_number;
using simon::testing::split_cells;

inline constexpr std::string_view ROADS = "application/automotive/roads/";
inline constexpr std::string_view REFERENCE =
    "application/automotive/reference/";
inline constexpr std::string_view TIRES = "3rd_party/chrono/";
inline constexpr std::string_view CARLA = "3rd_party/carla/";
inline constexpr std::string_view ESMINI_ROADS = "3rd_party/esmini/xodr/";

// The path of a road the tests read: CARLA's Town01 and esmini's signed roads
// from 3rd_party, the rest from ROADS.
inline auto find_road_path(std::string_view file) -> std::string {
  std::string_view directory =
      file == "Town01.xodr" ? CARLA
      : file == "some_signs.xodr" || file == "lane_offset_intersection.xodr"
          ? ESMINI_ROADS
          : ROADS;
  return std::string{directory} + std::string{file};
}

// The network of the road file `file`, read for a test.
inline auto create_network(std::string_view file) -> Network {
  auto network = load_network(find_road_path(file));
  REQUIRE(network);
  return std::move(*network);
}

// A car's width, every test vehicle's.
inline constexpr double CAR_WIDTH = 1.8;  // m.

// The box a vehicle covers, from its front at `pose` back its length.
inline auto create_box(const RoadPose& pose, const Driver& driver)
    -> model::OrientedBox {
  Vector3 front = pose.position.numerical_value_in(meter).eigen();
  double heading = pose.heading.numerical_value_in(radian);
  double length = driver.length.numerical_value_in(meter);
  return {.x = front.x() - 0.5 * length * std::cos(heading),
          .y = front.y() - 0.5 * length * std::sin(heading),
          .heading = heading,
          .length = length,
          .width = CAR_WIDTH};
}

// One row of a table, as text by column name.
using Row = std::map<std::string, std::string, std::less<>>;

// Every line of the reference table `name` but its header, as cells, for
// tables whose rows differ in length.
inline auto load_cells(std::string_view name)
    -> std::vector<std::vector<std::string>> {
  return load_table(std::string{REFERENCE} + std::string{name}).lines;
}

// Every row of the reference table `name`, named by its header.
inline auto load_rows(std::string_view name) -> std::vector<Row> {
  simon::testing::Table table =
      load_table(std::string{REFERENCE} + std::string{name});
  std::vector<Row> rows;
  for (const std::vector<std::string>& cells : table.lines) {
    REQUIRE(cells.size() == table.header.size());
    Row row;
    for (std::size_t i = 0; i < cells.size(); ++i) {
      row[table.header[i]] = cells[i];
    }
    rows.push_back(std::move(row));
  }
  return rows;
}

// The number in `row`'s `column`.
inline auto number(const Row& row, std::string_view column) -> double {
  return parse_number(row.find(column)->second);
}

// CommonRoad's vehicles 1, 2 and 3 by number, every parameter from
// commonroad_vehicles.csv.
inline auto load_commonroad_vehicles() -> std::map<int, vehicle::Parameters> {
  using namespace vehicle;
  std::map<int, std::map<std::string, double, std::less<>>> values;
  for (const Row& row : load_rows("commonroad_vehicles.csv")) {
    values[static_cast<int>(number(row, "vehicle"))][row.find("name")->second] =
        number(row, "value");
  }
  std::map<int, vehicle::Parameters> vehicles;
  for (const auto& [id, named] : values) {
    auto at = [&](std::string_view name) {
      auto found = named.find(name);
      REQUIRE(found != named.end());
      return found->second;
    };
    CommonRoadTire tire{.p_cx1 = at("tire_p_cx1"),
                        .p_dx1 = at("tire_p_dx1"),
                        .p_dx3 = at("tire_p_dx3"),
                        .p_ex1 = at("tire_p_ex1"),
                        .p_kx1 = at("tire_p_kx1"),
                        .p_hx1 = at("tire_p_hx1"),
                        .p_vx1 = at("tire_p_vx1"),
                        .r_bx1 = at("tire_r_bx1"),
                        .r_bx2 = at("tire_r_bx2"),
                        .r_cx1 = at("tire_r_cx1"),
                        .r_ex1 = at("tire_r_ex1"),
                        .r_hx1 = at("tire_r_hx1"),
                        .p_cy1 = at("tire_p_cy1"),
                        .p_dy1 = at("tire_p_dy1"),
                        .p_dy3 = at("tire_p_dy3"),
                        .p_ey1 = at("tire_p_ey1"),
                        .p_ky1 = at("tire_p_ky1"),
                        .p_hy1 = at("tire_p_hy1"),
                        .p_hy3 = at("tire_p_hy3"),
                        .p_vy1 = at("tire_p_vy1"),
                        .p_vy3 = at("tire_p_vy3"),
                        .r_by1 = at("tire_r_by1"),
                        .r_by2 = at("tire_r_by2"),
                        .r_by3 = at("tire_r_by3"),
                        .r_cy1 = at("tire_r_cy1"),
                        .r_ey1 = at("tire_r_ey1"),
                        .r_hy1 = at("tire_r_hy1"),
                        .r_vy1 = at("tire_r_vy1"),
                        .r_vy3 = at("tire_r_vy3"),
                        .r_vy4 = at("tire_r_vy4"),
                        .r_vy5 = at("tire_r_vy5"),
                        .r_vy6 = at("tire_r_vy6")};
    vehicles[id] = vehicle::Parameters{
        .length = at("l") * meter,
        .width = at("w") * meter,
        .front = at("a") * meter,
        .rear = at("b") * meter,
        .front_track = at("T_f") * meter,
        .rear_track = at("T_r") * meter,
        .center_of_gravity_height = at("h_cg") * meter,
        .sprung_height = at("h_s") * meter,
        .front_roll_axis_height = at("h_raf") * meter,
        .rear_roll_axis_height = at("h_rar") * meter,
        .wheel_radius = at("R_w") * meter,
        .mass = at("m") * kilogram,
        .sprung_mass = at("m_s") * kilogram,
        .front_unsprung_mass = at("m_uf") * kilogram,
        .rear_unsprung_mass = at("m_ur") * kilogram,
        .roll_inertia = at("I_Phi_s") * kilogram_square_meter,
        .pitch_inertia = at("I_y_s") * kilogram_square_meter,
        .yaw_inertia = at("I_z") * kilogram_square_meter,
        .roll_yaw_product = at("I_xz_s") * kilogram_square_meter,
        .front_unsprung_roll_inertia = at("I_uf") * kilogram_square_meter,
        .rear_unsprung_roll_inertia = at("I_ur") * kilogram_square_meter,
        .wheel_inertia = at("I_y_w") * kilogram_square_meter,
        .front_brake_share = at("T_sb"),
        .front_drive_share = at("T_se"),
        .steering = {.min = at("steering_min") * radian,
                     .max = at("steering_max") * radian,
                     .min_rate = at("steering_v_min") * radian_per_second,
                     .max_rate = at("steering_v_max") * radian_per_second},
        .longitudinal =
            {.max_acceleration =
                 at("longitudinal_a_max") * meter_per_second_squared,
             .switch_speed = at("longitudinal_v_switch") * meter_per_second,
             .min_speed = at("longitudinal_v_min") * meter_per_second,
             .max_speed = at("longitudinal_v_max") * meter_per_second},
        .suspension =
            {.front_spring = at("K_sf") * newton_per_meter,
             .front_damping = at("K_sdf") * newton_second_per_meter,
             .rear_spring = at("K_sr") * newton_per_meter,
             .rear_damping = at("K_sdr") * newton_second_per_meter,
             .roll_axis_spring = at("K_ras") * newton_per_meter,
             .roll_axis_damping = at("K_rad") * newton_second_per_meter,
             .front_roll_stiffness = at("K_tsf") * newton_meter_per_radian,
             .rear_roll_stiffness = at("K_tsr") * newton_meter_per_radian,
             .tire_spring = at("K_zt") * newton_per_meter,
             .tire_compliance = at("K_lt") * meter_per_newton,
             .front_camber = at("D_f") * radian_per_meter,
             .rear_camber = at("D_r") * radian_per_meter,
             .front_camber_squared = at("E_f") * radian_per_square_meter,
             .rear_camber_squared = at("E_r") * radian_per_square_meter},
        .tire = tire,
    };
  }
  return vehicles;
}

}  // namespace simon::automotive::testing

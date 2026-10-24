#include "px4_state.hpp"
#include <algorithm>
#include <cmath>
#include <numbers>
namespace nv_bridge {
namespace a = neurovector::avoidance;
using Status = px4_msgs::msg::VehicleStatus;
bool managed_mode(uint8_t mode) {
  return mode == Status::NAVIGATION_STATE_AUTO_MISSION || mode == Status::NAVIGATION_STATE_OFFBOARD ||
    mode == Status::NAVIGATION_STATE_AUTO_LOITER || mode == Status::NAVIGATION_STATE_AUTO_LAND;
}
Px4State::Px4State(rclcpp::Node &node) {
  position_.subscribe(node, "/fmu/out/vehicle_local_position");
  attitude_.subscribe(node, "/fmu/out/vehicle_attitude");
  status.subscribe(node, "/fmu/out/vehicle_status");
  land_.subscribe(node, "/fmu/out/vehicle_land_detected");
  battery_.subscribe(node, "/fmu/out/battery_status");
}
void Px4State::apply(neurovector::navigation::DeliveryInput &in,
                     const nv_safety_bridge::msg::DeliveryObservation &o) {
  const double now = in.flight.now_s;
  const auto &p = position_.message;
  const auto &s = status.message;
  const auto &l = land_.message;
  const auto &q = attitude_.message.q;
  const auto resets = std::pair{p.xy_reset_counter, p.z_reset_counter};
  if (position_.received >= 0) {
    if (reset_counters_ && resets != *reset_counters_) frame_reset_ = true;
    reset_counters_ = resets;
  }
  auto &f = in.flight;
  f.position = {p.y, p.x, -p.z}; f.velocity = {p.vy, p.vx, -p.vz};
  auto &state = f.state;
  state.observed_at_s = std::min(position_.received, attitude_.received);
  state.position_valid = position_.fresh(now, 0.2) && p.xy_valid && p.z_valid && !frame_reset_;
  state.velocity_valid = position_.fresh(now, 0.2) && p.v_xy_valid && p.v_z_valid;
  const double norm = q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3];
  state.attitude_valid = attitude_.fresh(now, 0.2) && std::isfinite(norm) && std::abs(norm - 1) < 0.01;
  state.roll_deg = std::atan2(2*(q[0]*q[1]+q[2]*q[3]), 1-2*(q[1]*q[1]+q[2]*q[2])) * 180 / std::numbers::pi;
  state.pitch_deg = std::asin(std::clamp(2.0*(q[0]*q[2]-q[3]*q[1]), -1.0, 1.0)) * 180 / std::numbers::pi;
  state.motors_healthy = o.motors_healthy; state.thrust_available = o.thrust_available;
  state.ground_contact = land_.fresh(now, 1.5) && l.ground_contact;
  state.landed_confirmed = land_.fresh(now, 1.5) && l.landed;
  const bool managed = managed_mode(s.nav_state);
  state.mode = a::FlightMode::unknown;
  if (status.fresh(now, 1.5) && land_.fresh(now, 1.5)) {
    if (l.landed) state.mode = a::FlightMode::landed;
    else if (s.nav_state == Status::NAVIGATION_STATE_AUTO_LAND) state.mode = a::FlightMode::landing;
    else if (managed && o.phase == o.WINDOW_DELIVERY) state.mode = a::FlightMode::window_delivery;
    else if (managed && o.phase == o.LANDING) state.mode = a::FlightMode::landing;
  }
  in.system.operator_override = o.operator_override || !managed || s.failsafe_and_user_took_over;
  in.system.native_failsafe_active = s.failsafe || frame_reset_;
  in.system.command_link_healthy = o.command_link_healthy && status.fresh(now, 1.5) && !s.gcs_connection_lost;
  in.system.battery_healthy = o.battery_healthy && battery_.fresh(now, 1.5) &&
    battery_.message.connected && battery_.message.warning == px4_msgs::msg::BatteryStatus::WARNING_NONE;
  if (!o.mission_authorized) state.mode = a::FlightMode::unknown;
  if (!std::isfinite(o.vehicle_position_error_m) || o.vehicle_position_error_m <= 0)
    state.position_valid = false;
  if (!std::isfinite(p.eph) || !std::isfinite(p.epv) || p.eph < 0 || p.epv < 0)
    state.position_valid = false;
  else in.vehicle_position_error_m = std::max<double>(o.vehicle_position_error_m, std::hypot(p.eph, p.epv));
}
}

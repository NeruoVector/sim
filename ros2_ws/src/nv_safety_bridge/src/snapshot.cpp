#include "snapshot.hpp"
namespace nv_bridge {
namespace a = neurovector::avoidance;
namespace n = neurovector::navigation;
namespace {
a::Vec3 vector(const geometry_msgs::msg::Vector3 &v) { return {v.x, v.y, v.z}; }
a::Box box(const nv_safety_bridge::msg::Box &b) { return {vector(b.minimum), vector(b.maximum)}; }
}
n::DeliveryInput snapshot(const nv_safety_bridge::msg::DeliveryObservation &m, double now) {
  n::DeliveryInput in;
  auto &f = in.flight;
  f.now_s = now; f.sensors_ready = m.sensors_ready; f.observations_at_s = m.observations_at_s;
  f.window_handoff = m.window_handoff;
  for (const auto &p : m.remaining_route) f.remaining_route.push_back(vector(p));
  for (const auto &b : m.no_fly_boxes) f.no_fly_boxes.push_back(box(b));
  for (const auto &c : m.coverage) f.coverage.push_back({box(c.bounds), c.observed_at_s});
  for (const auto &o : m.obstacles) {
    a::Obstacle value{o.id, o.priority, o.confidence, o.observed_at_s, o.position_known,
                      vector(o.position), o.velocity_known, vector(o.velocity), {}, o.radius_m};
    if (o.distance_known) value.distance_m = o.distance_m;
    f.obstacles.push_back(value);
  }
  if (m.has_landing_surface) {
    const auto &s = m.landing_surface;
    f.landing_surface = a::LandingSurface{vector(s.ground_point), s.half_width_m,
      s.half_length_m, s.slope_deg, s.roughness_m, s.position_error_m,
      s.confidence, s.observed_at_s, s.sensors_ready, s.support_confirmed};
  }
  in.vehicle_position_error_m = m.vehicle_position_error_m;
  auto &d = in.destination;
  d.now_s = now; d.floor = m.floor;
  d.destination = {{m.destination_gps[0], m.destination_gps[1], m.destination_gps[2]},
                   m.gps_error_m, m.gps_observed_at_s, m.gps_valid};
  d.survey = {{m.survey_origin_gps[0], m.survey_origin_gps[1], m.survey_origin_gps[2]},
    m.origin_error_m, vector(m.facade_point), vector(m.outward_normal), m.first_row_height_m,
    m.floor_pitch_m, m.row_tolerance_m, m.first_floor, m.rows_complete,
    m.protected_standoff_m, m.center_standoff_m};
  for (const auto &w : m.windows)
    d.windows.push_back({w.id, vector(w.center_enu), w.error_m, w.confidence, w.observed_at_s});
  in.disturbance = n::DisturbanceSample{m.disturbance_observed_at_s, vector(m.wind_enu_mps),
    vector(m.acceleration_residual_enu_mps2), vector(m.angular_rate_deg_s), m.disturbance_calibrated};
  in.wifi = {m.wifi_observed_at_s, m.wifi_healthy, {}};
  for (const auto &p : m.wifi_paths) {
    n::WifiPath value{p.id, vector(p.transmitter_enu), vector(p.receiver_enu),
      p.azimuth_rad, p.elevation_rad, p.excess_delay_s, p.delay_error_s,
      p.angular_error_rad, p.anchor_error_m, p.observed_at_s, p.confidence,
      p.object_radius_m, p.calibrated, p.path_resolved, {}};
    if (p.velocity_known) value.tracked_velocity_enu_mps = vector(p.tracked_velocity_enu_mps);
    in.wifi.paths.push_back(value);
  }
  in.system = {m.system_observed_at_s, m.usable_energy_wh, m.return_energy_wh, m.reserve_energy_wh,
    m.battery_healthy, m.command_link_healthy, m.independent_watchdog_enabled, false, m.operator_override};
  in.handoff = {m.handoff_observed_at_s, m.recipient_confirmed, m.receiving_path_clear,
    m.rotor_space_clear, m.payload_secure, m.mechanism_healthy, m.target_window_id};
  return in;
}
}

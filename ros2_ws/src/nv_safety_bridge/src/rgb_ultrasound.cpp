#include "rgb_ultrasound.hpp"
#include <algorithm>
#include <cmath>
#include <numbers>
namespace nv_bridge {
namespace a = neurovector::avoidance;
namespace n = neurovector::navigation;
namespace {
bool range_metadata(const sensor_msgs::msg::Range &r, const std::string &frame) {
  return r.header.frame_id == frame && r.radiation_type == r.ULTRASOUND &&
    std::isfinite(r.min_range) && std::isfinite(r.max_range) && r.min_range >= 0 &&
    r.max_range > r.min_range && std::isfinite(r.field_of_view) &&
    r.field_of_view > 0 && r.field_of_view < std::numbers::pi / 2;
}
bool measured(const sensor_msgs::msg::Range &r) {
  return std::isfinite(r.range) && r.range > 0 && r.range >= r.min_range && r.range <= r.max_range;
}
}
RgbUltrasound::RgbUltrasound(rclcpp::Node &node) {
  rcl_interfaces::msg::ParameterDescriptor fixed;
  fixed.read_only = true;
  camera_frame_ = node.declare_parameter<std::string>("camera_frame", "camera_optical_frame", fixed);
  front_frame_ = node.declare_parameter<std::string>("front_ultrasound_frame", "ultrasound_front", fixed);
  down_frame_ = node.declare_parameter<std::string>("down_ultrasound_frame", "ultrasound_down", fixed);
  camera_calibrated_ = node.declare_parameter<bool>("camera_calibrated", false, fixed);
  ultrasound_calibrated_ = node.declare_parameter<bool>("ultrasound_calibrated", false, fixed);
  error_m_ = node.declare_parameter<double>("range_and_mount_error_m", 0, fixed);
  if (camera_frame_.empty() || front_frame_.empty() || down_frame_.empty() || front_frame_ == down_frame_ ||
      camera_frame_ == front_frame_ || camera_frame_ == down_frame_)
    throw std::invalid_argument("camera/front/down sensor frames must identify distinct mounts");
  image_.subscribe(node, "/camera/image_raw"); info_.subscribe(node, "/camera/camera_info");
  front_.subscribe(node, "/ultrasound/front"); down_.subscribe(node, "/ultrasound/down");
}
std::string RgbUltrasound::apply(n::DeliveryInput &in) const {
  auto &f = in.flight;
  if (f.state.mode == a::FlightMode::landed) return {};
  auto block = [&](const char *reason) { f.sensors_ready = false; return std::string(reason); };
  if (!camera_calibrated_ || !ultrasound_calibrated_ || !std::isfinite(error_m_) || error_m_ <= 0 || error_m_ > 1)
    return block("RGB/ultrasound mounting calibration or bounded range error missing");
  if (!image_.fresh(f.now_s) || !info_.fresh(f.now_s))
    return block("RGB image or CameraInfo missing, replayed or stale");
  const auto &image = *image_.message;
  const auto &info = *info_.message;
  if (image.header.frame_id != camera_frame_ || info.header.frame_id != camera_frame_ ||
      std::abs(image_.stamp - info_.stamp) > 0.05 || image.width == 0 || image.height == 0 ||
      image.width > 4096 || image.height > 4096 || image.width != info.width || image.height != info.height ||
      (image.encoding != "rgb8" && image.encoding != "bgr8") || image.step < image.width * 3 ||
      image.step > 65536 || image.data.size() != static_cast<std::size_t>(image.step) * image.height)
    return block("RGB image layout, frame or calibration synchronization invalid");
  for (double v : info.k) if (!std::isfinite(v)) return block("invalid camera intrinsics");
  if (info.d.size() > 14) return block("unsupported camera distortion calibration");
  for (double v : info.d) if (!std::isfinite(v)) return block("invalid camera distortion calibration");
  if (info.k[0] <= 0 || info.k[4] <= 0 || info.k[2] < 0 || info.k[2] >= image.width ||
      info.k[5] < 0 || info.k[5] >= image.height || std::abs(info.k[8] - 1) > 1e-6 ||
      std::abs(info.k[1]) > 1e-6 || std::abs(info.k[3]) > 1e-6 ||
      std::abs(info.k[6]) > 1e-6 || std::abs(info.k[7]) > 1e-6 ||
      info.binning_x > 1 || info.binning_y > 1 || info.roi.width != 0 || info.roi.height != 0)
    return block("unsupported or uncalibrated RGB camera geometry");
  if (!front_.fresh(f.now_s) || !down_.fresh(f.now_s))
    return block("front/down ultrasound missing, replayed or stale");
  const auto &front = *front_.message, &down = *down_.message;
  if (!range_metadata(front, front_frame_) || !range_metadata(down, down_frame_))
    return block("ultrasound type, field of view, range limits or mounting frame invalid");
  if (!measured(front)) return block("front ultrasound has no trustworthy metric return");
  const a::Config config;
  const double speed = std::hypot(f.velocity.x, f.velocity.y, f.velocity.z);
  const double stop = speed * config.reaction_s + speed * speed / (2 * config.deceleration_mps2) +
    config.drone_radius_m + config.clearance_m + in.vehicle_position_error_m + error_m_;
  const double front_lower = front.range -
    (speed + config.unknown_obstacle_speed_mps) * (f.now_s - front_.stamp);
  if (front_lower <= stop) return block("front ultrasound distance is inside the current braking envelope");
  if (!measured(down) && !(std::isinf(down.range) && down.range > 0 && f.state.mode != a::FlightMode::landing))
    return block("down ultrasound lacks a trustworthy landing-height return");
  if (f.state.mode == a::FlightMode::landing) {
    if (!f.landing_surface) return block("down ultrasound cannot establish landing support or a clear footprint");
    const double tilt = config.max_tilt_deg * std::numbers::pi / 180;
    const double half = down.field_of_view / 2;
    const double low = down.range * std::cos(tilt + half) - error_m_;
    const double high = down.range + error_m_;
    const double height = f.position.z + f.velocity.z * (down_.stamp - f.state.observed_at_s) -
      f.landing_surface->ground_point.z;
    const double error = in.vehicle_position_error_m + f.landing_surface->position_error_m;
    if (tilt + half >= std::numbers::pi / 2 || !std::isfinite(height) ||
        height + error < low || height - error > high)
      return block("down ultrasound disagrees with the measured landing-surface height");
  }
  f.observations_at_s = std::min(f.observations_at_s, image_.stamp);
  return {};
}
}

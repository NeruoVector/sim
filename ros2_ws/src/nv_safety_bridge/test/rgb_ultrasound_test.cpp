#include "rgb_ultrasound.hpp"
#include "sensor_stream.hpp"
#include <iostream>
#include <limits>
#include <stdexcept>
#include <thread>
namespace n = neurovector::navigation;
namespace a = neurovector::avoidance;
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("camera_calibrated", true),
    rclcpp::Parameter("ultrasound_calibrated", true), rclcpp::Parameter("range_and_mount_error_m", 0.1)});
  auto node = std::make_shared<rclcpp::Node>("rgb_ultrasound_test", options);
  nv_bridge::RgbUltrasound gate(*node);
  SensorStream stream(*node);
  const auto image = stream.image;
  const auto info = stream.info;
  const auto front = stream.front, down = stream.down;
  n::DeliveryInput input;
  auto reset = [&] {
    stream.image = image; stream.info = info; stream.front = front; stream.down = down;
    stream.camera_enabled = stream.front_enabled = stream.down_enabled = true;
    stream.freeze_camera_stamp = stream.freeze_front_stamp = false;
    input = {}; input.vehicle_position_error_m = 0.1;
    input.flight.state.mode = a::FlightMode::window_delivery;
    input.flight.position = {0, 0, 7.5}; input.flight.velocity = {1, 0, 0};
  };
  auto pump = [&](int ticks) {
    for (int i = 0; i < ticks; ++i) {
      stream.publish(node->now()); rclcpp::spin_some(node);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    input.flight.now_s = input.flight.observations_at_s = input.flight.state.observed_at_s = node->now().seconds();
    input.flight.sensors_ready = true;
  };
  int cases = 0;
  auto check = [&](const char *name, auto mutate, bool blocked = true) {
    reset(); mutate(); pump(35);
    const auto reason = gate.apply(input);
    if (reason.empty() == blocked || input.flight.sensors_ready == blocked)
      throw std::runtime_error(std::string(name) + ": " + reason);
    std::cout << "PASS " << name << '\n'; ++cases;
  };
  reset(); pump(100);
  check("calibrated-rgb-front-down", [] {}, false);
  check("missing-rgb", [&] { stream.camera_enabled = false; });
  check("replayed-rgb", [&] { stream.image.header.stamp = node->now(); stream.freeze_camera_stamp = true; });
  check("bad-image-buffer", [&] { stream.image.data.pop_back(); });
  check("unsupported-image-encoding", [&] { stream.image.encoding = "16UC1"; });
  check("mismatched-camera-info", [&] { stream.info.width = 16; });
  check("missing-camera-intrinsics", [&] { stream.info.k[0] = 0; });
  check("nonfinite-camera-intrinsics", [&] { stream.info.k[0] = std::numeric_limits<double>::quiet_NaN(); });
  check("wrong-camera-frame", [&] { stream.image.header.frame_id = "other_camera"; });
  check("front-disconnected", [&] { stream.front_enabled = false; });
  check("replayed-ultrasound", [&] { stream.front.header.stamp = node->now(); stream.freeze_front_stamp = true; });
  check("down-disconnected", [&] { stream.down_enabled = false; });
  check("front-obstacle-inside-braking-distance", [&] { stream.front.range = 0.3; });
  check("front-no-echo-is-not-free-space", [&] { stream.front.range = std::numeric_limits<float>::infinity(); });
  check("front-nan", [&] { stream.front.range = std::numeric_limits<float>::quiet_NaN(); });
  check("front-below-minimum", [&] { stream.front.range = 0.1; });
  check("front-beyond-maximum", [&] { stream.front.range = 20; });
  check("infrared-cannot-impersonate-ultrasound", [&] { stream.front.radiation_type = stream.front.INFRARED; });
  check("swapped-front-and-down-frames", [&] { stream.front.header.frame_id = stream.down.header.frame_id; });
  check("down-out-of-range-remains-unknown-in-cruise", [&] { stream.down.range = std::numeric_limits<float>::infinity(); }, false);
  auto landing = [&] {
    input.flight.state.mode = a::FlightMode::landing;
    input.flight.velocity = {0, 0, -0.3};
    input.flight.landing_surface = a::LandingSurface{{0, 0, 0}, 3, 3, 0, 0.01, 0.05, 0.99, 0, true, true};
  };
  check("down-echo-agrees-with-landing-height", landing, false);
  check("down-echo-disagrees-with-floor", [&] { landing(); stream.down.range = 2; });
  check("down-no-echo-blocks-descent", [&] { landing(); stream.down.range = std::numeric_limits<float>::infinity(); });
  check("single-ultrasound-cannot-authorize-landing-support", [&] { landing(); input.flight.landing_surface.reset(); });
  reset(); pump(35);
  auto default_node = std::make_shared<rclcpp::Node>("uncalibrated_rgb_ultrasound_test");
  nv_bridge::RgbUltrasound uncalibrated(*default_node);
  if (uncalibrated.apply(input).empty()) throw std::runtime_error("default calibration permitted flight");
  ++cases;
  const auto result = node->set_parameters_atomically({rclcpp::Parameter("range_and_mount_error_m", 0.0)});
  if (result.successful) throw std::runtime_error("calibration was mutable during a mission");
  ++cases;
  std::cout << "RGB/ultrasound cases passed: " << cases << '\n';
  rclcpp::shutdown();
}

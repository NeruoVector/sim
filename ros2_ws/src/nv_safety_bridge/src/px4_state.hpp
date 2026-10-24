#pragma once
#include "snapshot.hpp"
#include "px4_topics.hpp"
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>
#include <px4_msgs/msg/battery_status.hpp>
#include <chrono>
namespace nv_bridge {
template<class T> struct Sample {
  T message;
  uint64_t source_stamp{};
  double received{-1};
  std::chrono::steady_clock::time_point wall;
  typename rclcpp::Subscription<T>::SharedPtr subscription;
  void subscribe(rclcpp::Node &node, const std::string &name) {
    subscription = node.create_subscription<T>(topic<T>(name), rclcpp::SensorDataQoS(),
      [this, &node](const T &m) {
        uint64_t stamp = m.timestamp;
        if constexpr (requires { m.timestamp_sample; }) {
          stamp = m.timestamp_sample;
          if (stamp > m.timestamp || m.timestamp - stamp > 200000) return;
        }
        if (stamp <= source_stamp) return;
        source_stamp = stamp;
        message = m; received = node.now().seconds(); wall = std::chrono::steady_clock::now();
      });
  }
  bool fresh(double now, double age) const {
    return received >= 0 && received <= now && now - received <= age &&
      std::chrono::duration<double>(std::chrono::steady_clock::now() - wall).count() <= age;
  }
};
class Px4State {
public:
  explicit Px4State(rclcpp::Node &node);
  void apply(neurovector::navigation::DeliveryInput &in,
             const nv_safety_bridge::msg::DeliveryObservation &observation);
  Sample<px4_msgs::msg::VehicleStatus> status;
private:
  Sample<px4_msgs::msg::VehicleLocalPosition> position_;
  Sample<px4_msgs::msg::VehicleAttitude> attitude_;
  Sample<px4_msgs::msg::VehicleLandDetected> land_;
  Sample<px4_msgs::msg::BatteryStatus> battery_;
  bool frame_reset_{};
  std::optional<std::pair<uint8_t, uint8_t>> reset_counters_;
};
}

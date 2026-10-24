#pragma once
#include "neurovector/delivery_navigation.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <chrono>
namespace nv_bridge {
template<class T> struct CameraRangeSample {
  typename T::ConstSharedPtr message;
  double stamp{-1};
  std::chrono::steady_clock::time_point received;
  typename rclcpp::Subscription<T>::SharedPtr subscription;
  void subscribe(rclcpp::Node &node, const std::string &topic) {
    subscription = node.create_subscription<T>(topic, rclcpp::SensorDataQoS(),
      [this](typename T::ConstSharedPtr m) {
        if (m->header.stamp.sec < 0 || m->header.stamp.nanosec >= 1000000000) {
          message.reset(); return;
        }
        const double time = m->header.stamp.sec + m->header.stamp.nanosec * 1e-9;
        if (time <= stamp) return;
        message = std::move(m); stamp = time; received = std::chrono::steady_clock::now();
      });
  }
  bool fresh(double now) const {
    return message && stamp >= 0 && stamp <= now && now - stamp <= 0.2 &&
      std::chrono::duration<double>(std::chrono::steady_clock::now() - received).count() <= 0.2;
  }
};
class RgbUltrasound {
public:
  explicit RgbUltrasound(rclcpp::Node &node);
  std::string apply(neurovector::navigation::DeliveryInput &input) const;
private:
  std::string camera_frame_, front_frame_, down_frame_;
  bool camera_calibrated_{}, ultrasound_calibrated_{};
  double error_m_{};
  CameraRangeSample<sensor_msgs::msg::Image> image_;
  CameraRangeSample<sensor_msgs::msg::CameraInfo> info_;
  CameraRangeSample<sensor_msgs::msg::Range> front_, down_;
};
}

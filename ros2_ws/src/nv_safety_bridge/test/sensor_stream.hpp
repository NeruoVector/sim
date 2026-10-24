#pragma once
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/range.hpp>
struct SensorStream {
  explicit SensorStream(rclcpp::Node &node) {
    image_pub = node.create_publisher<sensor_msgs::msg::Image>("/camera/image_raw", 10);
    info_pub = node.create_publisher<sensor_msgs::msg::CameraInfo>("/camera/camera_info", 10);
    front_pub = node.create_publisher<sensor_msgs::msg::Range>("/ultrasound/front", 10);
    down_pub = node.create_publisher<sensor_msgs::msg::Range>("/ultrasound/down", 10);
    image.header.frame_id = info.header.frame_id = "camera_optical_frame";
    image.width = info.width = 8; image.height = info.height = 6;
    image.encoding = "rgb8"; image.step = 24; image.data.resize(144);
    info.k = {4, 0, 4, 0, 4, 3, 0, 0, 1};
    front.header.frame_id = "ultrasound_front"; down.header.frame_id = "ultrasound_down";
    front.radiation_type = down.radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
    front.min_range = down.min_range = 0.2;
    front.max_range = down.max_range = 12;
    front.field_of_view = down.field_of_view = 0.2;
    front.range = 8; down.range = 7.5;
  }
  void publish(rclcpp::Time now) {
    if (!freeze_camera_stamp) image.header.stamp = now;
    if (!freeze_front_stamp) front.header.stamp = now;
    info.header.stamp = down.header.stamp = now;
    if (camera_enabled) image_pub->publish(image);
    info_pub->publish(info);
    if (front_enabled) front_pub->publish(front);
    if (down_enabled) down_pub->publish(down);
  }
  bool camera_enabled{true}, front_enabled{true}, down_enabled{true};
  bool freeze_camera_stamp{}, freeze_front_stamp{};
  sensor_msgs::msg::Image image;
  sensor_msgs::msg::CameraInfo info;
  sensor_msgs::msg::Range front, down;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr front_pub, down_pub;
};

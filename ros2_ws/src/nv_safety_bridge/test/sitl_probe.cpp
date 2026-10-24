#include "px4_topics.hpp"
#include "nv_safety_bridge/msg/safety_decision.hpp"
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <iostream>
#include <stdexcept>
#include <thread>
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("nv_safety_sitl_probe");
  using Status = px4_msgs::msg::VehicleStatus;
  using Position = px4_msgs::msg::VehicleLocalPosition;
  using Command = px4_msgs::msg::VehicleCommand;
  using Decision = nv_safety_bridge::msg::SafetyDecision;
  int statuses = 0, positions = 0, decisions = 0, commands = 0;
  bool valid_position = false;
  auto status = node->create_subscription<Status>(nv_bridge::topic<Status>("/fmu/out/vehicle_status"),
    rclcpp::SensorDataQoS(), [&](const Status &s) {
      if (s.arming_state != Status::ARMING_STATE_DISARMED)
        throw std::runtime_error("SITL probe requires an already disarmed vehicle");
      ++statuses;
    });
  auto position = node->create_subscription<Position>(nv_bridge::topic<Position>("/fmu/out/vehicle_local_position"),
    rclcpp::SensorDataQoS(), [&](const Position &p) { ++positions; valid_position = p.xy_valid && p.z_valid; });
  auto decision = node->create_subscription<Decision>("safety/decision", 10, [&](const Decision &d) {
    if (d.directive != "delegate_to_autopilot" || d.disarm_allowed || d.handoff_ready || d.command_pending)
      throw std::runtime_error("missing observations did not keep the disarmed SITL blocked");
    ++decisions;
  });
  auto command = node->create_subscription<Command>("/fmu/in/vehicle_command", rclcpp::SensorDataQoS(),
    [&](const Command &) { ++commands; });
  const auto start = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - start < std::chrono::seconds(20)) {
    rclcpp::spin_some(node);
    if (statuses >= 3 && positions >= 20 && decisions >= 100 && valid_position) break;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  if (statuses < 3 || positions < 20 || decisions < 100 || commands != 0 || !valid_position)
    throw std::runtime_error("SITL telemetry missing/invalid, bridge not running, or unexpected actuation");
  std::cout << "PASS actual disarmed PX4 SITL: statuses=" << statuses << " positions=" << positions
            << " decisions=" << decisions << " commands=" << commands << '\n';
  rclcpp::shutdown();
}

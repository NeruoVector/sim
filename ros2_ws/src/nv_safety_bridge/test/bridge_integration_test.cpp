#include "px4_topics.hpp"
#include "sensor_stream.hpp"
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>
#include <px4_msgs/msg/battery_status.hpp>
#include "nv_safety_bridge/msg/delivery_observation.hpp"
#include "nv_safety_bridge/msg/safety_decision.hpp"
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_command_ack.hpp>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <thread>
using Observation = nv_safety_bridge::msg::DeliveryObservation;
using Decision = nv_safety_bridge::msg::SafetyDecision;
using Status = px4_msgs::msg::VehicleStatus;
using Command = px4_msgs::msg::VehicleCommand;
using Ack = px4_msgs::msg::VehicleCommandAck;
void require(bool condition, const char *reason) {
  if (!condition) throw std::runtime_error(reason);
}
Observation fixture() {
  Observation o;
  o.mission_authorized = true; o.phase = o.WINDOW_DELIVERY;
  o.sensors_ready = o.motors_healthy = o.thrust_available = true;
  o.vehicle_position_error_m = 0.1;
  geometry_msgs::msg::Vector3 end; end.y = -4; end.z = 7.5;
  o.remaining_route = {end};
  o.floor = 3; o.destination_gps = o.survey_origin_gps = {37, 127, 100};
  o.gps_error_m = 0.2; o.gps_valid = true;
  o.outward_normal.y = -1; o.first_row_height_m = 1.5; o.floor_pitch_m = 3;
  o.row_tolerance_m = 0.35; o.first_floor = 1; o.rows_complete = true;
  o.protected_standoff_m = 2; o.center_standoff_m = 4;
  for (int row = 0; row < 3; ++row) {
    nv_safety_bridge::msg::Window w;
    w.id = std::to_string(row); w.center_enu.z = 1.5 + row*3;
    w.error_m = 0.05; w.confidence = 0.99; o.windows.push_back(w);
  }
  o.disturbance_calibrated = o.wifi_healthy = true;
  o.usable_energy_wh = 100; o.return_energy_wh = 20; o.reserve_energy_wh = 10;
  o.battery_healthy = o.command_link_healthy = o.independent_watchdog_enabled = true;
  return o;
}
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("bridge_integration_test");
  const bool reject = node->declare_parameter<bool>("reject_commands", false);
  const bool freeze = node->declare_parameter<bool>("freeze_pose", false);
  auto observation_pub = node->create_publisher<Observation>("safety/observation", rclcpp::QoS(1).best_effort());
  auto pos_pub = node->create_publisher<px4_msgs::msg::VehicleLocalPosition>(nv_bridge::topic<px4_msgs::msg::VehicleLocalPosition>("/fmu/out/vehicle_local_position"), 10);
  auto attitude_pub = node->create_publisher<px4_msgs::msg::VehicleAttitude>("/fmu/out/vehicle_attitude", 10);
  auto status_pub = node->create_publisher<Status>(nv_bridge::topic<Status>("/fmu/out/vehicle_status"), 10);
  auto land_pub = node->create_publisher<px4_msgs::msg::VehicleLandDetected>("/fmu/out/vehicle_land_detected", 10);
  auto battery_pub = node->create_publisher<px4_msgs::msg::BatteryStatus>(nv_bridge::topic<px4_msgs::msg::BatteryStatus>("/fmu/out/battery_status"), 10);
  auto ack_pub = node->create_publisher<Ack>(nv_bridge::topic<Ack>("/fmu/out/vehicle_command_ack"), 10);
  Decision decision;
  int commands = 0, holds = 0, returns = 0;
  uint8_t nav_state = Status::NAVIGATION_STATE_OFFBOARD;
  auto decision_sub = node->create_subscription<Decision>("safety/decision", 10,
    [&](const Decision &d) { decision = d; });
  auto command_sub = node->create_subscription<Command>("/fmu/in/vehicle_command", 10,
    [&](const Command &c) {
      require(c.command == Command::VEHICLE_CMD_DO_SET_MODE && c.param2 == 4,
              "bridge emitted an unexpected command (including arm/disarm)");
      ++commands;
      if (c.param3 == 3) { ++holds; if (!reject) nav_state = Status::NAVIGATION_STATE_AUTO_LOITER; }
      else if (c.param3 == 5) { ++returns; if (!reject) nav_state = Status::NAVIGATION_STATE_AUTO_RTL; }
      else throw std::runtime_error("unexpected PX4 submode");
      Ack ack; ack.timestamp = static_cast<uint64_t>(node->now().seconds() * 1e6);
      ack.command = c.command; ack.target_system = c.source_system;
      ack.target_component = c.source_component;
      ack.result = reject ? Ack::VEHICLE_CMD_RESULT_DENIED : Ack::VEHICLE_CMD_RESULT_ACCEPTED;
      ack_pub->publish(ack);
    });
  auto observation = fixture();
  SensorStream sensors(*node);
  std::optional<uint64_t> frozen_stamp;
  auto tick = [&](bool publish_observation = true) {
    const double now = node->now().seconds();
    const auto stamp = static_cast<uint64_t>(now * 1e6);
    sensors.publish(node->now());
    px4_msgs::msg::VehicleLocalPosition p;
    p.timestamp = p.timestamp_sample = stamp;
    if (frozen_stamp) p.timestamp_sample = *frozen_stamp;
    p.xy_valid = p.z_valid = p.v_xy_valid = p.v_z_valid = true;
    p.x = -10; p.z = -7.5; p.vx = 1; p.eph = p.epv = 0.02;
    pos_pub->publish(p);
    px4_msgs::msg::VehicleAttitude attitude; attitude.timestamp = attitude.timestamp_sample = stamp; attitude.q[0] = 1;
    attitude_pub->publish(attitude);
    Status status; status.timestamp = stamp; status.nav_state = nav_state;
    status.arming_state = Status::ARMING_STATE_ARMED; status.system_id = status.component_id = 1;
    status_pub->publish(status);
    px4_msgs::msg::VehicleLandDetected land; land.timestamp = stamp; land_pub->publish(land);
    px4_msgs::msg::BatteryStatus battery; battery.timestamp = stamp; battery.connected = true;
    battery_pub->publish(battery);
    if (publish_observation) {
      ++observation.sequence;
      observation.observed_at_s = observation.observations_at_s = observation.gps_observed_at_s = now;
      observation.disturbance_observed_at_s = observation.wifi_observed_at_s = now;
      observation.system_observed_at_s = observation.handoff_observed_at_s = now;
      for (auto &w : observation.windows) w.observed_at_s = now;
      observation_pub->publish(observation);
    }
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  };
  for (int i = 0; i < 150; ++i) tick(false);
  require(commands == 0, "missing mission observations caused an unsolicited command");
  for (int i = 0; i < 200; ++i) tick();
  if (reject) {
    require(decision.directive == "delegate_to_autopilot" && !decision.handoff_ready &&
            decision.reason.find("failure latched") != std::string::npos,
            "rejected PX4 command did not latch mission delegation");
    std::cout << "PASS ROS command rejection latches delegation\n";
  } else {
    require(decision.directive == "continue_mission" && decision.target_window_id == "2",
            "healthy ROS input never reached the window navigation decision");
    if (freeze) {
      frozen_stamp = static_cast<uint64_t>(node->now().seconds() * 1e6);
      for (int i = 0; i < 120; ++i) tick();
      require(decision.directive == "delegate_to_autopilot" && returns > 0,
              "fresh camera/header timestamp concealed a replayed PX4 pose sample");
      std::cout << "PASS ROS replayed PX4 sample rejected despite fresh observations\n";
      rclcpp::shutdown();
      return 0;
    }
    sensors.camera_enabled = false;
    for (int i = 0; i < 40; ++i) tick();
    require(decision.directive == "hold", "fresh observation flag concealed a missing RGB camera");
    sensors.camera_enabled = true;
    for (int i = 0; i < 30; ++i) tick();
    require(decision.directive == "continue_mission", "fresh RGB camera did not restore the sensor gate");
    sensors.front.range = 0.3;
    for (int i = 0; i < 30; ++i) tick();
    require(decision.directive == "hold", "near front ultrasound return did not block movement");
    sensors.front.range = 8;
    for (int i = 0; i < 30; ++i) tick();
    const int initial_holds = holds;
    nav_state = Status::NAVIGATION_STATE_OFFBOARD;
    observation.wind_enu_mps.x = 10;
    for (int i = 0; i < 120; ++i) tick();
    require(decision.directive == "hold" && holds > initial_holds &&
            decision.last_command_result == Ack::VEHICLE_CMD_RESULT_ACCEPTED && !decision.command_pending,
            "gust did not reach an acknowledged PX4 hold command");
    for (int i = 0; i < 120; ++i) tick(false);
    require(decision.directive == "delegate_to_autopilot" && returns > 0,
            "observation timeout did not delegate and request native return");
    observation.wind_enu_mps.x = 0;
    for (int i = 0; i < 120; ++i) tick();
    require(decision.directive == "delegate_to_autopilot", "stale-input failure automatically resumed");
    nav_state = Status::NAVIGATION_STATE_POSCTL;
    observation.operator_override = true;
    const int previous = commands;
    for (int i = 0; i < 120; ++i) tick();
    require(commands == previous, "bridge overrode manual control");
    std::cout << "PASS ROS healthy window, gust hold/ACK, timeout return, failure latch, manual priority; commands=" << commands << '\n';
  }
  rclcpp::shutdown();
}

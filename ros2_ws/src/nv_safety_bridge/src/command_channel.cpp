#include "command_channel.hpp"
#include <cmath>
namespace nv_bridge {
namespace n = neurovector::navigation;
using Status = px4_msgs::msg::VehicleStatus;
using Command = px4_msgs::msg::VehicleCommand;
using Ack = px4_msgs::msg::VehicleCommandAck;
CommandChannel::CommandChannel(rclcpp::Node &node, const Px4State &state) : state_(state) {
  source_component_ = node.declare_parameter<int>("source_component", 191);
  if (source_component_ < 1 || source_component_ > 255)
    throw std::invalid_argument("source_component must be in 1..255");
  publisher_ = node.create_publisher<Command>("/fmu/in/vehicle_command", 10);
  ack_subscriber_ = node.create_subscription<Ack>(topic<Ack>("/fmu/out/vehicle_command_ack"), rclcpp::SensorDataQoS(),
    [this](const Ack &m) {
      if (pending_ && m.command == Command::VEHICLE_CMD_DO_SET_MODE &&
          m.target_component == source_component_ && m.target_system == 1) {
        last_result_ = m.result;
        if (m.result != Ack::VEHICLE_CMD_RESULT_ACCEPTED && m.result != Ack::VEHICLE_CMD_RESULT_IN_PROGRESS)
          failed_ = true;
      }
    });
}
void CommandChannel::apply(const n::DeliveryDecision &d,
    const nv_safety_bridge::msg::DeliveryObservation *o, double now) {
  if (o && o->mission_authorized && std::isfinite(o->observed_at_s) &&
      o->observed_at_s <= now && now - o->observed_at_s <= 0.2) active_ = true;
  if (!active_ || !o) return;
  const auto &s = state_.status.message;
  if (!state_.status.fresh(now, 1.5) || s.arming_state != Status::ARMING_STATE_ARMED ||
      !managed_mode(s.nav_state) || s.failsafe || s.failsafe_and_user_took_over || o->operator_override) {
    pending_ = false;
    return;
  }
  const bool stop = d.safety == n::SafetyDirective::hold ||
    (d.safety == n::SafetyDirective::continue_mission &&
     d.navigation.action == neurovector::avoidance::Action::slow);
  const bool delegate = d.safety == n::SafetyDirective::delegate_to_autopilot;
  if (!stop && !delegate) return;
  const uint8_t expected = stop ? Status::NAVIGATION_STATE_AUTO_LOITER : Status::NAVIGATION_STATE_AUTO_RTL;
  if (!requested_mode_ || *requested_mode_ != expected) {
    requested_mode_ = expected; attempts_ = 0; last_command_.reset();
  }
  if (s.nav_state == expected) { pending_ = false; attempts_ = 0; return; }
  const auto wall = std::chrono::steady_clock::now();
  if (last_command_ && std::chrono::duration<double>(wall - *last_command_).count() < 1) return;
  if (attempts_ >= 3) { failed_ = true; return; }
  Command command;
  command.timestamp = static_cast<uint64_t>(now * 1e6);
  command.command = Command::VEHICLE_CMD_DO_SET_MODE;
  command.param1 = 1; command.param2 = 4; command.param3 = stop ? 3 : 5;
  command.target_system = s.system_id; command.target_component = s.component_id;
  command.source_system = 1; command.source_component = static_cast<uint8_t>(source_component_);
  command.from_external = true;
  publisher_->publish(command);
  last_command_ = wall; pending_ = true; last_result_ = -1; ++attempts_;
}
}

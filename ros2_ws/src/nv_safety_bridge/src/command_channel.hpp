#pragma once
#include "px4_state.hpp"
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_command_ack.hpp>
namespace nv_bridge {
class CommandChannel {
public:
  CommandChannel(rclcpp::Node &node, const Px4State &state);
  void apply(const neurovector::navigation::DeliveryDecision &decision,
             const nv_safety_bridge::msg::DeliveryObservation *observation, double now);
  bool failed() const { return failed_; }
  bool pending() const { return pending_; }
  int last_result() const { return last_result_; }
private:
  const Px4State &state_;
  std::optional<std::chrono::steady_clock::time_point> last_command_;
  std::optional<uint8_t> requested_mode_;
  bool active_{}, failed_{}, pending_{};
  int source_component_{}, last_result_{-1}, attempts_{};
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr publisher_;
  rclcpp::Subscription<px4_msgs::msg::VehicleCommandAck>::SharedPtr ack_subscriber_;
};
}

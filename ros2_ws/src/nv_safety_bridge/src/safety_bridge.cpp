#include "command_channel.hpp"
#include "rgb_ultrasound.hpp"
#include "nv_safety_bridge/msg/safety_decision.hpp"
#include <cmath>
namespace nv_bridge {
namespace n = neurovector::navigation;
using Observation = nv_safety_bridge::msg::DeliveryObservation;
class SafetyBridge : public rclcpp::Node {
public:
  SafetyBridge() : Node("nv_safety_bridge"), px4_(*this), commands_(*this, px4_), sensors_(*this) {
    decision_publisher_ = create_publisher<nv_safety_bridge::msg::SafetyDecision>("safety/decision", 10);
    observation_subscriber_ = create_subscription<Observation>("safety/observation", rclcpp::QoS(1).best_effort(),
      [this](Observation::ConstSharedPtr m) {
        if (observation_ && m->sequence <= observation_->sequence) return;
        observation_ = std::move(m); observation_wall_ = std::chrono::steady_clock::now();
      });
    timer_ = create_wall_timer(std::chrono::milliseconds(10), [this] { tick(); });
  }
private:
  void tick() {
    const double time = now().seconds();
    n::DeliveryDecision d;
    auto valid = observation_ && std::isfinite(observation_->observed_at_s) &&
      observation_->observed_at_s <= time && time - observation_->observed_at_s <= 0.2 &&
      std::chrono::duration<double>(std::chrono::steady_clock::now() - observation_wall_).count() <= 0.2;
    if (valid) {
      if (!converted_sequence_ || *converted_sequence_ != observation_->sequence) {
        input_ = snapshot(*observation_, time);
        converted_sequence_ = observation_->sequence;
      }
      input_->flight.now_s = input_->destination.now_s = time;
      px4_.apply(*input_, *observation_);
      input_->flight.sensors_ready = observation_->sensors_ready;
      const auto sensor_reason = sensors_.apply(*input_);
      d = navigator_.update(*input_);
      if (!sensor_reason.empty() && d.safety == n::SafetyDirective::hold)
        d.navigation.reason = sensor_reason;
    } else {
      // Do not start a mission supervisor until its first coherent observation.
      d.safety = n::SafetyDirective::delegate_to_autopilot;
      d.navigation.reason = "delivery observations missing, replayed or stale";
      if (observation_) input_failed_ = true;
    }
    if (input_failed_ || commands_.failed()) {
      d = {};
      d.safety = n::SafetyDirective::delegate_to_autopilot;
      d.navigation.reason = "input/command failure latched; authorize a new mission supervisor";
    }
    commands_.apply(d, observation_.get(), time);
    nv_safety_bridge::msg::SafetyDecision out;
    out.observed_at_s = time; out.observation_sequence = observation_ ? observation_->sequence : 0;
    out.directive = n::safety_name(d.safety);
    out.action = neurovector::avoidance::action_name(d.navigation.action);
    out.desired_speed_mps = d.navigation.desired_speed_mps; out.reason = d.navigation.reason;
    out.handoff_ready = d.handoff_ready; out.disarm_allowed = d.disarm_allowed;
    if (d.target) out.target_window_id = d.target->id;
    out.command_pending = commands_.pending(); out.last_command_result = commands_.last_result();
    decision_publisher_->publish(out);
  }
  Px4State px4_;
  CommandChannel commands_;
  RgbUltrasound sensors_;
  n::DeliveryNavigator navigator_;
  Observation::ConstSharedPtr observation_;
  std::optional<n::DeliveryInput> input_;
  std::optional<uint64_t> converted_sequence_;
  std::chrono::steady_clock::time_point observation_wall_;
  bool input_failed_{};
  rclcpp::Publisher<nv_safety_bridge::msg::SafetyDecision>::SharedPtr decision_publisher_;
  rclcpp::Subscription<Observation>::SharedPtr observation_subscriber_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<nv_bridge::SafetyBridge>());
  rclcpp::shutdown();
}

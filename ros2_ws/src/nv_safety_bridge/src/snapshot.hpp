#pragma once
#include "neurovector/delivery_navigation.hpp"
#include "nv_safety_bridge/msg/delivery_observation.hpp"
namespace nv_bridge {
neurovector::navigation::DeliveryInput snapshot(const nv_safety_bridge::msg::DeliveryObservation &m, double now);
}

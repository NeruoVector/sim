#pragma once
#include <cstdint>
#include <string>
namespace nv_bridge {
template<class T> std::string topic(const std::string &base) {
  return T::MESSAGE_VERSION == 0 ? base : base + "_v" + std::to_string(T::MESSAGE_VERSION);
}
bool managed_mode(uint8_t mode);
}

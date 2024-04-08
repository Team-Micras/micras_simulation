/**
 * @file rotary_sensor.cpp
 *
 * @brief Proxy RotarySensor class source
 *
 * @date 04/2024
 */

#include <numbers>
#include <vector>
#include <cstdint>

std::vector<uint8_t> data;

#include "micras/proxy/rotary_sensor.hpp"

namespace micras::proxy {
RotarySensor::RotarySensor(const Config& config) {
    this->subscriber = config.node->create_subscription<sensor_msgs::msg::JointState>(
        config.topic, 1, [this](const sensor_msgs::msg::JointState& msg) { this->data = msg; }
    );
}

float RotarySensor::get_position() const {
    return this->data.position[0];
}
}  // namespace micras::proxy

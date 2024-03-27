/**
 * @file rotary_sensor.cpp
 *
 * @brief Proxy RotarySensor class source
 *
 * @date 03/2024
 */

#include <numbers>
#include <vector>

std::vector<uint8_t> data;

#include "proxy/rotary_sensor.hpp"

namespace proxy {
RotarySensor::RotarySensor(Config& config) {
    this->subscriber = config.node->create_subscription<sensor_msgs::msg::JointState>(
        config.topic, 1, [this](const sensor_msgs::msg::JointState& msg) {
            this->data = msg;
        });
}

float RotarySensor::get_position() {
    return this->data.position[0];
}
}  // namespace proxy

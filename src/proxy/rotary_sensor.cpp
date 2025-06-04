/**
 * @file rotary_sensor.cpp
 *
 * @brief Proxy RotarySensor class source
 *
 * @date 04/2024
 */

#include <cstdint>
#include <numbers>
#include <vector>

#include "micras/proxy/rotary_sensor.hpp"

namespace micras::proxy {
RotarySensor::RotarySensor(const Config& config) {
    this->subscriber = config.node->create_subscription<sensor_msgs::msg::JointState>(
        config.topic, 1,
        [this](const sensor_msgs::msg::JointState& msg) {
            if (msg.header.stamp.sec != 0 and msg.header.stamp.nanosec != 0) {
                this->data = msg;
            }
        }
    );

    this->data.position.resize(2);
}

float RotarySensor::get_position() const {
    return this->data.position[0];
}
}  // namespace micras::proxy

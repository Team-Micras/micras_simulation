/**
 * @file
 */

#include "micras/proxy/rotary_sensor.hpp"

namespace micras::proxy {
RotarySensor::RotarySensor(const Config& config) {
    this->subscriber = config.node->create_subscription<sensor_msgs::msg::JointState>(
        config.topic, 1,
        [this](const sensor_msgs::msg::JointState& msg) {
            if (!msg.position.empty()) {
                this->data = msg;
            }
        }
    );

    this->data.position.resize(1, 0.0);
}

float RotarySensor::get_position() const {
    return this->data.position[0];
}
}  // namespace micras::proxy

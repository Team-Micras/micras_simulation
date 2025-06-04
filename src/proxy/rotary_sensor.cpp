/**
 * @file
 */

#include "micras/proxy/rotary_sensor.hpp"

namespace micras::proxy {
RotarySensor::RotarySensor(const Config& config) {
    this->subscriber = config.node->create_subscription<example_interfaces::msg::Float64>(
        config.topic, 1, [this](const example_interfaces::msg::Float64& msg) { this->data = msg; }
    );
}

float RotarySensor::get_position() const {
    return static_cast<float>(this->data);
}
}  // namespace micras::proxy

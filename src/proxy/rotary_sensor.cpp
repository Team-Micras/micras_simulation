/**
 * @file
 */

#include "micras/proxy/rotary_sensor.hpp"

namespace micras::proxy {
RotarySensor::RotarySensor(const Config& config) {
    this->subscriber = config.node->create_subscription<example_interfaces::msg::Float64>(
        config.topic, 1, [](const example_interfaces::msg::Float64& /*msg*/) {}
    );
}

float RotarySensor::get_position() const {
    example_interfaces::msg::Float64 msg;
    rclcpp::MessageInfo              info;

    if (this->subscriber->take(msg, info)) {
        this->data = msg.data;
    }

    return static_cast<float>(this->data);
}
}  // namespace micras::proxy

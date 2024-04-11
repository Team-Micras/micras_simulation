/**
 * @file battery.cpp
 *
 * @brief Proxy Battery class implementation
 *
 * @date 04/2024
 */

#include "micras/proxy/battery.hpp"

namespace micras::proxy {
Battery::Battery(const Config& config) {
    this->subscriber = config.node->create_subscription<std_msgs::msg::Float32>(
        config.topic, 1, [this](const std_msgs::msg::Float32& msg) { this->reading = msg.data; }
    );
}

float Battery::get_voltage() const {
    return this->reading;
}

uint32_t Battery::get_voltage_raw() const {
    return ((this->reading) / 9.9) * 4095;
}
}  // namespace micras::proxy

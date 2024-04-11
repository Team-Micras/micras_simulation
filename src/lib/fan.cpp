/**
 * @file fan.cpp
 *
 * @brief Proxy Fan class source
 *
 * @date 03/2024
 */

#include "micras/proxy/fan.hpp"

namespace micras::proxy {
Fan::Fan(const Config& config) {
    this->publisher = config.node->create_publisher<std_msgs::msg::Float32>(config.topic, 1);
    this->stop();
    this->enable();
}

void Fan::enable() {
    this->enabled = true;
}

void Fan::disable() {
    this->enabled = false;
}

void Fan::set_speed(float speed) {
    if (!this->enabled) {
        this->stop();
        return;
    }

    this->fan_speed.data = speed;
    this->publisher->publish(this->fan_speed);
}

void Fan::stop() {
    this->fan_speed.data = 0;
    this->publisher->publish(this->fan_speed);
}
}  // namespace micras::proxy

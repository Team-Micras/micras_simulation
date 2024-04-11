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
    this->publisher = config.node->create_publisher<geometry_msgs::msg::Twist>(locomotion_config.topic, 1);
    this->stop();
    this->enable();
}

void Fan::enable() {
    this->enabled = true;
    this->publisher->publish(this->message);
}

void Fan::disable() {
    this->enabled = false;

    std_msgs::msg::Float32 stopped_message{};

    this->publisher->publish(stopped_message);
}

void Fan::set_speed(float speed) {
    this->message.data = speed;

    if (this->enabled) {
        this->publisher->publish(this->message);
    }
}

void Fan::stop() {
    this->set_speed(0);
}
}  // namespace micras::proxy

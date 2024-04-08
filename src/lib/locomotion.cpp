/**
 * @file locomotion.cpp
 *
 * @brief Proxy Locomotion class source
 *
 * @date 03/2024
 */

#include "micras/proxy/locomotion.hpp"

namespace micras::proxy {
Locomotion::Locomotion(const Config& locomotion_config) {
    this->publisher = locomotion_config.node->create_publisher<geometry_msgs::msg::Twist>(locomotion_config.topic, 1);
    this->stop();
    this->enable();
}

void Locomotion::enable() {
    this->enabled = true;
}

void Locomotion::disable() {
    this->enabled = false;
}

void Locomotion::set_wheel_speed(float left_speed, float right_speed) {
    if (!this->enabled) {
        this->stop();
        return;
    }

    this->twist.linear.x = (left_speed + right_speed) / 2;
    this->twist.angular.z = (right_speed - left_speed) / 2;
    this->publisher->publish(this->twist);
}

void Locomotion::set_speed(float linear, float angular) {
    if (!this->enabled) {
        this->stop();
        return;
    }

    this->twist.linear.x = linear;
    this->twist.angular.z = angular;
    this->publisher->publish(this->twist);
}

void Locomotion::stop() {
    this->twist.linear.x = 0;
    this->twist.angular.z = 0;
    this->publisher->publish(this->twist);
}

void Locomotion::stop_left() { }

void Locomotion::stop_right() { }
}  // namespace micras::proxy

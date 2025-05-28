/**
 * @file
 */

#include "micras/proxy/locomotion.hpp"

namespace micras::proxy {
Locomotion::Locomotion(const Config& config) {
    this->publisher = config.node->create_publisher<geometry_msgs::msg::Twist>(config.topic, 1);
    this->stop();
    this->disable();
}

void Locomotion::enable() {
    this->enabled = true;
}

void Locomotion::disable() {
    this->enabled = false;
}

void Locomotion::set_wheel_command(float left_command, float right_command) {
    this->twist.linear.x = (left_command + right_command) / 2;
    this->twist.angular.z = (right_command - left_command) / 2;

    if (this->enabled) {
        this->publisher->publish(this->twist);
    }
}

void Locomotion::set_command(float linear, float angular) {
    this->twist.linear.x = linear;
    this->twist.angular.z = angular;

    if (this->enabled) {
        this->publisher->publish(this->twist);
    }
}

void Locomotion::stop() {
    this->twist.linear.x = 0.0;
    this->twist.angular.z = 0.0;
    this->publisher->publish(this->twist);
}
}  // namespace micras::proxy

/**
 * @file locomotion.cpp
 *
 * @brief Proxy Locomotion class source
 *
 * @date 03/2024
 */

#include "proxy/locomotion.hpp"

namespace proxy {
Locomotion::Locomotion(const Config& locomotion_config) {
    this->publisher = locomotion_config.node->create_publisher<geometry_msgs::msg::Twist>(locomotion_config.topic, 1);
}

void Locomotion::set_velocity_command(double linear, double angular) {
    this->twist.linear.x = linear;
    this->twist.angular.z = angular;
    this->publisher->publish(this->twist);
}
}  // namespace proxy

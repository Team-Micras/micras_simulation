/**
 * @file odometry.cpp
 *
 * @brief Proxy Odometry class source
 *
 * @date 03/2024
 */

#include "proxy/odometry.hpp"

namespace proxy {
Odometry::Odometry(const Config& odometry_config) {
    this->subscriber = odometry_config.node->create_subscription<nav_msgs::msg::Odometry>(
        odometry_config.topic, 1, [this](const nav_msgs::msg::Odometry& msg) {
            this->data = msg;
        });
}

double Odometry::get_x_position() const {
    return this->data.pose.pose.position.x;
}

double Odometry::get_y_position() const {
    return this->data.pose.pose.position.y;
}

double Odometry::get_orientation() const {
    return 2 * std::atan2(this->data.pose.pose.orientation.z, this->data.pose.pose.orientation.w);
}

double Odometry::get_linear_velocity() const {
    return this->data.twist.twist.linear.x;
}

double Odometry::get_angular_velocity() const {
    return this->data.twist.twist.angular.z;
}
}  // namespace proxy

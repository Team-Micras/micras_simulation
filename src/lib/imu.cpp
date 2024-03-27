/**
 * @file imu.cpp
 *
 * @brief Proxy Imu class source
 *
 * @date 03/2024
 */

#include "proxy/imu.hpp"

namespace proxy {
Imu::Imu(const Config& imu_config) {
    this->subscriber = imu_config.node->create_subscription<sensor_msgs::msg::Imu>(
        imu_config.topic, 1, [this](const sensor_msgs::msg::Imu& msg) {
            this->data = msg;
        });
}

double Imu::get_orientation(Axis axis) const {
    switch (axis) {
        case Axis::W:
            return this->data.orientation.w;

        case Axis::X:
            return this->data.orientation.x;

        case Axis::Y:
            return this->data.orientation.y;

        case Axis::Z:
            return this->data.orientation.z;

        default:
            return 0.0;
    }
}

double Imu::get_angular_velocity(Axis axis) const {
    switch (axis) {
        case Axis::X:
            return this->data.angular_velocity.x;

        case Axis::Y:
            return this->data.angular_velocity.y;

        case Axis::Z:
            return this->data.angular_velocity.z;

        default:
            return 0.0;
    }
}

double Imu::get_linear_acceleration(Axis axis) const {
    switch (axis) {
        case Axis::X:
            return this->data.linear_acceleration.x;

        case Axis::Y:
            return this->data.linear_acceleration.y;

        case Axis::Z:
            return this->data.linear_acceleration.z;

        default:
            return 0.0;
    }
}
}  // namespace proxy

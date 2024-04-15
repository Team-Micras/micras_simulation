/**
 * @file imu.cpp
 *
 * @brief Proxy Imu class source
 *
 * @date 03/2024
 */

#include "micras/proxy/imu.hpp"

namespace micras::proxy {
Imu::Imu(const Config& config) {
    this->subscriber = config.node->create_subscription<sensor_msgs::msg::Imu>(
        config.topic, 1, [this](const sensor_msgs::msg::Imu& msg) { this->data = msg; }
    );
}

void Imu::update_data() {
    const auto& quat = this->data.orientation;

    // roll (x-axis rotation)
    float sinp = 2 * (quat.w * quat.x + quat.y * quat.z);
    float cosp = 1 - 2 * (quat.x * quat.x + quat.y * quat.y);
    this->orientation[0] = std::atan2(sinp, cosp);

    // pitch (y-axis rotation)
    sinp = std::sqrt(1 + 2 * (quat.w * quat.y - quat.x * quat.z));
    cosp = std::sqrt(1 - 2 * (quat.w * quat.y - quat.x * quat.z));
    this->orientation[1] = 2 * std::atan2(sinp, cosp) - M_PI / 2;

    // yaw (z-axis rotation)
    sinp = 2 * (quat.w * quat.z + quat.x * quat.y);
    cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z);
    this->orientation[2] = std::atan2(sinp, cosp);
}

float Imu::get_orientation(Axis axis) const {
    switch (axis) {
        case Axis::X:
            return this->orientation[0];

        case Axis::Y:
            return this->orientation[1];

        case Axis::Z:
            return this->orientation[2];

        default:
            return 0.0F;
    }
}

float Imu::get_angular_velocity(Axis axis) const {
    switch (axis) {
        case Axis::X:
            return this->data.angular_velocity.x;

        case Axis::Y:
            return this->data.angular_velocity.y;

        case Axis::Z:
            return this->data.angular_velocity.z;

        default:
            return 0.0F;
    }
}

float Imu::get_linear_acceleration(Axis axis) const {
    switch (axis) {
        case Axis::X:
            return this->data.linear_acceleration.x;

        case Axis::Y:
            return this->data.linear_acceleration.y;

        case Axis::Z:
            return this->data.linear_acceleration.z;

        default:
            return 0.0F;
    }
}
}  // namespace micras::proxy

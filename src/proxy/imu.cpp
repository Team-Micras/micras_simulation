/**
 * @file
 */

#include "micras/proxy/imu.hpp"

#include <cmath>

namespace micras::proxy {
Imu::Imu(const Config& config) {
    this->subscriber = config.node->create_subscription<sensor_msgs::msg::Imu>(
        config.topic, 1, [this](const sensor_msgs::msg::Imu& msg) { this->data = msg; }
    );
}

void Imu::update() {
    this->angular_velocity[0] = this->data.angular_velocity.x;
    this->angular_velocity[1] = this->data.angular_velocity.y;
    this->angular_velocity[2] = this->data.angular_velocity.z;
    this->linear_acceleration[0] = this->data.linear_acceleration.x;
    this->linear_acceleration[1] = this->data.linear_acceleration.y;
    this->linear_acceleration[2] = this->data.linear_acceleration.z;
}

float Imu::get_angular_velocity(Axis axis) const {
    switch (axis) {
        case Axis::X:
            return this->angular_velocity[0];

        case Axis::Y:
            return this->angular_velocity[1];

        case Axis::Z:
            return this->angular_velocity[2];

        default:
            return 0.0F;
    }
}

float Imu::get_linear_acceleration(Axis axis) const {
    switch (axis) {
        case Axis::X:
            return this->linear_acceleration[0];

        case Axis::Y:
            return this->linear_acceleration[1];

        case Axis::Z:
            return this->linear_acceleration[2];

        default:
            return 0.0F;
    }
}

void Imu::calibrate() {
    // Na implementação para ROS, a calibração é feita externamente
}

bool Imu::was_initialized() const {
    // Na implementação para ROS, o IMU é sempre inicializado
    return true;
}
}  // namespace micras::proxy

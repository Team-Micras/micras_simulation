/**
 * @file
 */

#include "micras/proxy/imu.hpp"

#include <cmath>

namespace micras::proxy {
Imu::Imu(const Config& config) {
    this->gyro_subscriber = config.node->create_subscription<example_interfaces::msg::Float64MultiArray>(
        config.gyro_topic, 1, [this](const example_interfaces::msg::Float64MultiArray& msg) { this->gyro_data = msg; }
    );

    this->accelerometer_subscriber = config.node->create_subscription<example_interfaces::msg::Float64MultiArray>(
        config.accelerometer_topic, 1,
        [this](const example_interfaces::msg::Float64MultiArray& msg) { this->accelerometer_data = msg; }
    );

    this->gyro_data.layout.dim.resize(1);
    this->gyro_data.layout.dim[0].label = "gyro";
    this->gyro_data.layout.dim[0].size = 3;
    this->gyro_data.layout.dim[0].stride = 3;
    this->gyro_data.data.resize(3, 0.0);
    this->accelerometer_data.layout.dim.resize(1);
    this->accelerometer_data.layout.dim[0].label = "accelerometer";
    this->accelerometer_data.layout.dim[0].size = 3;
    this->accelerometer_data.layout.dim[0].stride = 3;
    this->accelerometer_data.data.resize(3, 0.0);
}

void Imu::update() {
    this->angular_velocity[0] = this->gyro_data.data[0];
    this->angular_velocity[1] = this->gyro_data.data[1];
    this->angular_velocity[2] = this->gyro_data.data[2];
    this->linear_acceleration[0] = this->accelerometer_data.data[0];
    this->linear_acceleration[1] = this->accelerometer_data.data[1];
    this->linear_acceleration[2] = this->accelerometer_data.data[2];
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

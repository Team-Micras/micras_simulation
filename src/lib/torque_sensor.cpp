/**
 * @file torque_sensor.cpp
 *
 * @brief Proxy Torque Sensor class source
 *
 * @date 03/2024
 */

#include "proxy/torque_sensor.hpp"

namespace proxy {
TorqueSensor::TorqueSensor(const Config& torque_sensor_config) {
    this->front_subscriber = torque_sensor_config.node->create_subscription<geometry_msgs::msg::WrenchStamped>(
        torque_sensor_config.front_topic, 1, [this](const geometry_msgs::msg::WrenchStamped& msg) {
            this->front_torque = msg.wrench.torque.x;
        });

    this->rear_subscriber = torque_sensor_config.node->create_subscription<geometry_msgs::msg::WrenchStamped>(
        torque_sensor_config.rear_topic, 1, [this](const geometry_msgs::msg::WrenchStamped& msg) {
            this->rear_torque = msg.wrench.torque.x;
        });
}

double TorqueSensor::get_torque() const {
    return (this->front_torque + this->front_torque) / 2;
}
}  // namespace proxy

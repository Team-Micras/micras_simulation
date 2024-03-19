/**
 * @file distance_sensor.cpp
 *
 * @brief Proxy Distance Sensor class source
 *
 * @date 03/2024
 */

#include "proxy/distance_sensor.hpp"

namespace proxy {
DistanceSensor::DistanceSensor(const Config& distance_sensor_config) {
    this->subscriber = distance_sensor_config.node->create_subscription<sensor_msgs::msg::LaserScan>(
        distance_sensor_config.topic, 1, [this](const sensor_msgs::msg::LaserScan& msg) {
            this->distance = msg.ranges[0];
        });
}

double DistanceSensor::get_distance() const {
    return this->distance;
}
}  // namespace proxy

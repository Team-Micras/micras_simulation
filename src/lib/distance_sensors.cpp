/**
 * @file distance_sensor.cpp
 *
 * @brief Proxy Distance Sensor class source
 *
 * @date 03/2024
 */

#include "proxy/distance_sensor.hpp"

namespace proxy {
template <uint8_t num_of_sensors>
DistanceSensors<num_of_sensors>::DistanceSensors(Config& config) {
    for (uint8_t i = 0; i < num_of_sensors; i++) {
        this->subscribers[i] = config.node->create_subscription<sensor_msgs::msg::LaserScan>(
            config.topic_array[i], 1, [this](const sensor_msgs::msg::LaserScan& msg) {
                this->distances[i] = msg.ranges[0];
            });
    }
}

template <uint8_t num_of_sensors>
void DistanceSensors<num_of_sensors>::set_led_intensity(float  /* intensity */) {
}

template <uint8_t num_of_sensors>
float DistanceSensors<num_of_sensors>::get_distance(uint8_t sensor_index) const {
    return this->distances.at(sensor_index);
}

template <uint8_t num_of_sensors>
uint32_t DistanceSensors<num_of_sensors>::get_distance_raw(uint8_t sensor_index) const {
    return this->distances.at(sensor_index);
}
}  // namespace proxy

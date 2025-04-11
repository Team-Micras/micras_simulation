/**
 * @file
 */

#ifndef MICRAS_PROXY_WALL_SENSORS_CPP
#define MICRAS_PROXY_WALL_SENSORS_CPP

#include "micras/proxy/wall_sensors.hpp"

namespace micras::proxy {
template <uint8_t num_of_sensors>
TWallSensors<num_of_sensors>::TWallSensors(const Config& config) :
    max_distance{config.max_distance}, max_reading{config.max_reading} {
    for (uint8_t i = 0; i < num_of_sensors; i++) {
        this->subscribers[i] = config.node->template create_subscription<sensor_msgs::msg::LaserScan>(
            config.topic_array[i], 1,
            [this, i](const sensor_msgs::msg::LaserScan& msg) {
                if (!msg.ranges.empty()) {
                    this->distances[i] = msg.ranges[0];
                }
            }
        );
        this->distances[i] = 0.0f;
        this->wall_threshold[i] = 0.5f * max_distance;        // Valor padrão
        this->free_space_threshold[i] = 0.8f * max_distance;  // Valor padrão
    }
}

template <uint8_t num_of_sensors>
void TWallSensors<num_of_sensors>::turn_on() {
    this->leds_on = true;
}

template <uint8_t num_of_sensors>
void TWallSensors<num_of_sensors>::turn_off() {
    this->leds_on = false;
}

template <uint8_t num_of_sensors>
void TWallSensors<num_of_sensors>::update() {
    // As leituras são atualizadas automaticamente pelos callbacks
}

template <uint8_t num_of_sensors>
core::Observation TWallSensors<num_of_sensors>::get_observation(uint8_t sensor_index) const {
    if (sensor_index >= num_of_sensors) {
        return core::Observation::UNKNOWN;
    }

    const float reading = this->get_reading(sensor_index);

    if (reading < this->wall_threshold[sensor_index]) {
        return core::Observation::WALL;
    }

    if (reading > this->free_space_threshold[sensor_index]) {
        return core::Observation::FREE_SPACE;
    }

    return core::Observation::UNKNOWN;
}

template <uint8_t num_of_sensors>
float TWallSensors<num_of_sensors>::get_reading(uint8_t sensor_index) const {
    if (sensor_index >= num_of_sensors) {
        return 0.0f;
    }
    return this->distances[sensor_index];
}

template <uint8_t num_of_sensors>
float TWallSensors<num_of_sensors>::get_adc_reading(uint8_t sensor_index) const {
    if (sensor_index >= num_of_sensors) {
        return 0.0f;
    }
    return this->distances[sensor_index] / this->max_distance;
}

template <uint8_t num_of_sensors>
void TWallSensors<num_of_sensors>::calibrate_front_wall() {
    // Front sensor is typically at index 0
    this->wall_calibration_measure[0] = this->get_reading(0);
    this->update_thresholds();
}

template <uint8_t num_of_sensors>
void TWallSensors<num_of_sensors>::calibrate_left_wall() {
    // Left sensor is typically at index 1
    this->wall_calibration_measure[1] = this->get_reading(1);
    this->update_thresholds();
}

template <uint8_t num_of_sensors>
void TWallSensors<num_of_sensors>::calibrate_right_wall() {
    // Right sensor is typically at index 2
    this->wall_calibration_measure[2] = this->get_reading(2);
    this->update_thresholds();
}

template <uint8_t num_of_sensors>
void TWallSensors<num_of_sensors>::calibrate_front_free_space() {
    // Front sensor is typically at index 0
    this->free_space_calibration_measure[0] = this->get_reading(0);
    this->update_thresholds();
}

template <uint8_t num_of_sensors>
void TWallSensors<num_of_sensors>::calibrate_left_free_space() {
    // Left sensor is typically at index 1
    this->free_space_calibration_measure[1] = this->get_reading(1);
    this->update_thresholds();
}

template <uint8_t num_of_sensors>
void TWallSensors<num_of_sensors>::calibrate_right_free_space() {
    // Right sensor is typically at index 2
    this->free_space_calibration_measure[2] = this->get_reading(2);
    this->update_thresholds();
}

template <uint8_t num_of_sensors>
void TWallSensors<num_of_sensors>::update_thresholds() {
    for (uint8_t i = 0; i < num_of_sensors; i++) {
        if (this->wall_calibration_measure[i] > 0.0f && this->free_space_calibration_measure[i] > 0.0f) {
            // Se ambas as calibrações foram realizadas
            const float range = this->free_space_calibration_measure[i] - this->wall_calibration_measure[i];
            this->wall_threshold[i] = this->wall_calibration_measure[i] + this->uncertainty * range;
            this->free_space_threshold[i] = this->free_space_calibration_measure[i] - this->uncertainty * range;
        }
    }
}
}  // namespace micras::proxy

#endif  // MICRAS_PROXY_WALL_SENSORS_CPP

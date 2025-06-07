/**
 * @file
 */

#ifndef MICRAS_PROXY_WALL_SENSORS_CPP
#define MICRAS_PROXY_WALL_SENSORS_CPP

#include "micras/core/utils.hpp"
#include "micras/proxy/wall_sensors.hpp"

namespace micras::proxy {
template <uint8_t num_of_sensors>
TWallSensors<num_of_sensors>::TWallSensors(const Config& config) :
    max_distance{config.max_distance},
    max_reading{config.max_reading},
    leds_on{false},
    filters{core::make_array<core::ButterworthFilter, num_of_sensors>(config.filter_cutoff)},
    base_readings{config.base_readings},
    uncertainty{config.uncertainty} {
    this->turn_off();

    for (uint8_t i = 0; i < num_of_sensors; i++) {
        this->subscribers[i] = config.node->template create_subscription<sensor_msgs::msg::LaserScan>(
            config.topic_array[i], 1,
            [this, i](const sensor_msgs::msg::LaserScan& msg) {
                if (!msg.ranges.empty()) {
                    this->readings[i] = this->leds_on ? msg.ranges[0] : 0;
                }
            }
        );
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
    for (uint8_t i = 0; i < num_of_sensors; i++) {
        this->filters[i].update(this->get_adc_reading(i));
    }
}

template <uint8_t num_of_sensors>
bool TWallSensors<num_of_sensors>::get_wall(uint8_t sensor_index, bool disturbed) const {
    return this->filters.at(sensor_index).get_last() >
           this->base_readings.at(sensor_index) * this->uncertainty * (disturbed ? 1.2F : 1.0F);
}

template <uint8_t num_of_sensors>
float TWallSensors<num_of_sensors>::get_reading(uint8_t sensor_index) const {
    return this->filters.at(sensor_index).get_last();
}

template <uint8_t num_of_sensors>
float TWallSensors<num_of_sensors>::get_adc_reading(uint8_t sensor_index) const {
    return this->readings.at(sensor_index) / this->max_distance * this->max_reading;

    // return static_cast<float>(std::abs(this->buffer.at(sensor_index) - this->buffer.at(sensor_index +
    // num_of_sensors))
    //        ) /
    //        this->adc.get_max_reading();
}

template <uint8_t num_of_sensors>
float TWallSensors<num_of_sensors>::get_sensor_error(uint8_t sensor_index) const {
    return this->get_reading(sensor_index) - this->base_readings.at(sensor_index);
}

template <uint8_t num_of_sensors>
void TWallSensors<num_of_sensors>::calibrate_sensor(uint8_t sensor_index) {
    this->base_readings.at(sensor_index) = this->get_reading(sensor_index);
}
}  // namespace micras::proxy

#endif  // MICRAS_PROXY_WALL_SENSORS_CPP

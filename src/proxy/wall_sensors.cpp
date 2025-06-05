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
    max_sensor_reading{config.max_sensor_reading},
    leds_on{false},
    filters{core::make_array<core::ButterworthFilter, num_of_sensors>(config.filter_cutoff)},
    base_readings{config.base_readings},
    uncertainty{config.uncertainty},
    constant{static_cast<float>(
        -std::pow(config.max_sensor_distance, 2) *
        std::log(1.0f - config.min_sensor_reading / config.max_sensor_reading)
    )} {
    this->turn_off();

    for (uint8_t i = 0; i < num_of_sensors; i++) {
        this->subscribers[i] = config.node->template create_subscription<example_interfaces::msg::Float64>(
            config.topic_array[i], 1,
            [this, i](const example_interfaces::msg::Float64& msg) { this->readings[i] = this->leds_on ? msg.data : 0; }
        );
    }
}

template <uint8_t num_of_sensors>
void TWallSensors<num_of_sensors>::turn_on() {
    this->leds_on = true;
}

template <uint8_t num_of_sensors>
void TWallSensors<num_of_sensors>::turn_off() {
    this->leds_on = true;
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
    float raw_reading = this->readings.at(sensor_index);
    if (raw_reading < 0.0F) {
        return 0.0F;
    }

    raw_reading = this->readings.at(sensor_index);

    float intensity = 1 / std::pow(raw_reading, 2.0F);
    float reading = this->max_sensor_reading * (1 - std::exp(-this->constant * intensity));

    return reading;
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

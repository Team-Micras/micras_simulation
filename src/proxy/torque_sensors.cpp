/**
 * @file
 */

#ifndef MICRAS_PROXY_TORQUE_SENSORS_CPP
#define MICRAS_PROXY_TORQUE_SENSORS_CPP

#include "micras/proxy/torque_sensors.hpp"

namespace micras::proxy {
template <uint8_t num_of_wheel_pairs>
TTorqueSensors<num_of_wheel_pairs>::TTorqueSensors(const Config& config) :
    shunt_resistor{config.shunt_resistor}, max_torque{config.max_torque}, reference_voltage{config.reference_voltage} {
    for (uint8_t i = 0; i < num_of_wheel_pairs; i++) {
        this->wheel_pairs[i].front_subscriber =
            config.node->template create_subscription<geometry_msgs::msg::WrenchStamped>(
                config.wheel_pairs_topics[i][0], 1,
                [this, i](const geometry_msgs::msg::WrenchStamped& msg) {
                    this->wheel_pairs[i].front_torque = msg.wrench.torque.x;
                }
            );

        this->wheel_pairs[i].rear_subscriber =
            config.node->template create_subscription<geometry_msgs::msg::WrenchStamped>(
                config.wheel_pairs_topics[i][1], 1,
                [this, i](const geometry_msgs::msg::WrenchStamped& msg) {
                    this->wheel_pairs[i].rear_torque = msg.wrench.torque.x;
                }
            );
    }
}

template <uint8_t num_of_wheel_pairs>
void TTorqueSensors<num_of_wheel_pairs>::calibrate() {
    // Na simulação, não há necessidade de calibração
}

template <uint8_t num_of_wheel_pairs>
void TTorqueSensors<num_of_wheel_pairs>::update() {
    // As leituras são atualizadas automaticamente pelos callbacks
}

template <uint8_t num_of_wheel_pairs>
float TTorqueSensors<num_of_wheel_pairs>::get_torque(uint8_t sensor_index) const {
    if (sensor_index >= num_of_wheel_pairs) {
        return 0.0f;
    }
    return (this->wheel_pairs[sensor_index].front_torque + this->wheel_pairs[sensor_index].rear_torque) / 2;
}

template <uint8_t num_of_wheel_pairs>
float TTorqueSensors<num_of_wheel_pairs>::get_torque_raw(uint8_t sensor_index) const {
    return this->get_torque(sensor_index);  // Não há filtragem na simulação
}

template <uint8_t num_of_wheel_pairs>
float TTorqueSensors<num_of_wheel_pairs>::get_current(uint8_t sensor_index) const {
    return this->get_torque(sensor_index) * this->reference_voltage / (this->max_torque * this->shunt_resistor);
}

template <uint8_t num_of_wheel_pairs>
float TTorqueSensors<num_of_wheel_pairs>::get_current_raw(uint8_t sensor_index) const {
    return this->get_current(sensor_index);  // Não há filtragem na simulação
}

template <uint8_t num_of_wheel_pairs>
float TTorqueSensors<num_of_wheel_pairs>::get_adc_reading(uint8_t sensor_index) const {
    return this->get_current(sensor_index) * this->shunt_resistor / this->reference_voltage;
}
}  // namespace micras::proxy

#endif  // MICRAS_PROXY_TORQUE_SENSORS_CPP

/**
 * @file torque_sensors.cpp
 *
 * @brief Proxy TorqueSensors class implementation
 *
 * @date 04/2024
 */

#ifndef MICRAS_PROXY_TORQUE_SENSORS_CPP
#define MICRAS_PROXY_TORQUE_SENSORS_CPP

#include "micras/proxy/torque_sensors.hpp"

namespace micras::proxy {
template <uint8_t num_of_sensors>
TorqueSensors<num_of_sensors>::TorqueSensors(const Config& config) :
    shunt_resistor{config.shunt_resistor}, max_torque{config.max_torque}, reference_voltage{config.reference_voltage} {
    for (uint8_t i = 0; i < num_of_sensors; i++) {
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

template <uint8_t num_of_sensors>
float TorqueSensors<num_of_sensors>::get_torque(uint8_t sensor_index) const {
    return (this->wheel_pairs[sensor_index].front_torque + this->wheel_pairs[sensor_index].rear_torque) / 2;
}

template <uint8_t num_of_sensors>
float TorqueSensors<num_of_sensors>::get_current(uint8_t sensor_index) const {
    return this->get_torque(sensor_index) * this->reference_voltage / (this->max_torque * this->shunt_resistor);
}
}  // namespace micras::proxy

#endif  // MICRAS_PROXY_TORQUE_SENSORS_CPP

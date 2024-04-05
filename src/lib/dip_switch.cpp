/**
 * @file dip_switch.cpp
 *
 * @brief Proxy DIP Switch class source
 *
 * @date 04/2024
 */

#ifndef MICRAS_PROXY_DIP_SWITCH_CPP
#define MICRAS_PROXY_DIP_SWITCH_CPP

#include "proxy/dip_switch.hpp"

namespace proxy {
template <uint8_t num_of_sensors>
DipSwitch<num_of_sensors>::DipSwitch(const Config& config) {
    for (uint8_t i = 0; i < num_of_switches; i++) {
        this->subscribers[i] = config.node->create_subscription<std_msgs::msg::Bool>(
            config.topic_array[i], 1, [this](const std_msgs::msg::Bool& msg) {
                this->states[i] = msg.data;
            });
    }
}

template <uint8_t num_of_sensors>
bool DipSwitch<num_of_sensors>::get_switch_state(uint8_t switch_index) const {
    return this->states.at(switch_index);
}

template <uint8_t num_of_sensors>
uint8_t DipSwitch<num_of_sensors>::get_switches_value() const {
    uint8_t switches_value = 0;

    for (uint8_t i = 0; i < num_of_sensors; i++) {
        switches_value |= (this->states.at(switch_index) << i);
    }

    return switches_value;
}
}  // namespace proxy

#endif // MICRAS_PROXY_DIP_SWITCH_CPP

/**
 * @file
 */

#ifndef MICRAS_PROXY_DIP_SWITCH_CPP
#define MICRAS_PROXY_DIP_SWITCH_CPP

#include "micras/proxy/dip_switch.hpp"

namespace micras::proxy {
template <uint8_t num_of_switches>
TDipSwitch<num_of_switches>::TDipSwitch(const Config& config) {
    for (uint8_t i = 0; i < num_of_switches; i++) {
        this->subscribers[i] = config.node->template create_subscription<std_msgs::msg::Bool>(
            config.topic_array[i], 1, [this, i](const std_msgs::msg::Bool& msg) { this->states[i] = msg; }
        );
    }
}

template <uint8_t num_of_switches>
bool TDipSwitch<num_of_switches>::get_switch_state(uint8_t switch_index) const {
    if (switch_index >= num_of_switches) {
        return false;
    }
    return this->states.at(switch_index).data;
}

template <uint8_t num_of_switches>
uint8_t TDipSwitch<num_of_switches>::get_switches_value() const {
    uint8_t switches_value = 0;

    for (uint8_t i = 0; i < num_of_switches; i++) {
        switches_value |= (static_cast<uint8_t>(this->states.at(i).data) << i);
    }

    return switches_value;
}
}  // namespace micras::proxy

#endif  // MICRAS_PROXY_DIP_SWITCH_CPP

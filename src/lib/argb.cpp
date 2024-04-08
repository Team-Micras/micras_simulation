/**
 * @file argb.cpp
 *
 * @brief Proxy Argb class implementation
 *
 * @date 03/2024
 */

#ifndef MICRAS_PROXY_ARGB_CPP
#define MICRAS_PROXY_ARGB_CPP

#include "micras/proxy/argb.hpp"

namespace micras::proxy {
template <uint8_t num_of_leds>
Argb<num_of_leds>::Argb(const Config& config) {
    for (uint8_t i = 0; i < num_of_leds; i++) {
        this->publishers.at(i) =
            config.node->template create_publisher<std_msgs::msg::ColorRGBA>(config.topic_array.at(i), 1);
    }
}

template <uint8_t num_of_leds>
void Argb<num_of_leds>::set_color(const Color& color, uint8_t index) {
    if (index >= num_of_leds) {
        return;
    }

    this->publishers.at(index)->publish({color.red, color.green, color.blue, 1});
}

template <uint8_t num_of_leds>
void Argb<num_of_leds>::set_color(const Color& color) {
    for (uint8_t i = 0; i < num_of_leds; i++) {
        this->publishers.at(i)->publish({color.red, color.green, color.blue, 1});
    }
}

template <uint8_t num_of_leds>
void Argb<num_of_leds>::turn_off(uint8_t index) {
    this->set_color(index, {0, 0, 0});
}

template <uint8_t num_of_leds>
void Argb<num_of_leds>::turn_off() {
    this->set_color({0, 0, 0});
}
}  // namespace micras::proxy

#endif  // MICRAS_PROXY_ARGB_CPP

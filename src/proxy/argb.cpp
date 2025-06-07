/**
 * @file
 */

#ifndef MICRAS_PROXY_ARGB_CPP
#define MICRAS_PROXY_ARGB_CPP

#include "micras/proxy/argb.hpp"

namespace micras::proxy {
template <uint8_t num_of_leds>
TArgb<num_of_leds>::TArgb(const Config& config) {
    for (uint8_t i = 0; i < num_of_leds; i++) {
        this->publishers.at(i) =
            config.node->template create_publisher<std_msgs::msg::ColorRGBA>(config.topic_array.at(i), 1);
    }
}

template <uint8_t num_of_leds>
void TArgb<num_of_leds>::set_color(const Color& color, uint8_t index) {
    if (index >= num_of_leds) {
        return;
    }

    std_msgs::msg::ColorRGBA color_msg;
    color_msg.r = this->map_color(color.red);
    color_msg.g = this->map_color(color.green);
    color_msg.b = this->map_color(color.blue);
    color_msg.a = 1;

    this->publishers.at(index)->publish(color_msg);
}

template <uint8_t num_of_leds>
void TArgb<num_of_leds>::set_color(const Color& color) {
    std_msgs::msg::ColorRGBA color_msg;
    color_msg.r = this->map_color(color.red);
    color_msg.g = this->map_color(color.green);
    color_msg.b = this->map_color(color.blue);
    color_msg.a = 1;

    for (uint8_t i = 0; i < num_of_leds; i++) {
        this->publishers.at(i)->publish(color_msg);
    }
}

template <uint8_t num_of_leds>
void TArgb<num_of_leds>::set_colors(const std::array<Color, num_of_leds>& colors) {
    for (uint8_t i = 0; i < num_of_leds; i++) {
        this->set_color(colors[i], i);
    }
}

template <uint8_t num_of_leds>
void TArgb<num_of_leds>::turn_off(uint8_t index) {
    this->set_color({0, 0, 0}, index);
}

template <uint8_t num_of_leds>
void TArgb<num_of_leds>::turn_off() {
    this->set_color({0, 0, 0});
}

template <uint8_t num_of_leds>
void TArgb<num_of_leds>::update() {
    // No ROS 2, o publish já envia as alterações imediatamente
}

template <uint8_t num_of_leds>
float TArgb<num_of_leds>::map_color(uint8_t color) {
    return static_cast<float>(color) / 255;
}
}  // namespace micras::proxy

#endif  // MICRAS_PROXY_ARGB_CPP

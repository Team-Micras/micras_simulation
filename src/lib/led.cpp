/**
 * @file led.cpp
 *
 * @author Comp ThundeRatz <comp@thunderatz.org>
 *
 * @brief Proxy Led class source.
 *
 * @date 01/2024
 *
 * @copyright MIT License - Copyright (c) 2024 ThundeRatz
 *
 */

#include <stdio.h>

#include "proxy/led.hpp"

namespace proxy {
Led::Led(Config led_config) : rclcpp::Node(led_config.name) {
    this->publisher = this->create_publisher<std_msgs::msg::Bool>(led_config.topic, 1);
}

void Led::turn_on() {
    this->state.data = true;
    this->publisher->publish(this->state);
}

void Led::turn_off() {
    this->state.data = false;
    this->publisher->publish(this->state);
}

void Led::toggle() {
    this->state.data = not this->state.data;
    this->publisher->publish(this->state);
}
}  // namespace proxy

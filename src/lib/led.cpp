/**
 * @file led.cpp
 *
 * @brief Proxy Led class source.
 *
 * @date 03/2024
 */

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

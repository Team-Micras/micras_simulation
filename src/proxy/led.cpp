/**
 * @file led.cpp
 *
 * @brief Proxy Led class source
 *
 * @date 03/2024
 */

#include "micras/proxy/led.hpp"

namespace micras::proxy {
Led::Led(const Config& config) {
    this->publisher = config.node->create_publisher<std_msgs::msg::Bool>(config.topic, 1);
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
}  // namespace micras::proxy

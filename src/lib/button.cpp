/**
 * @file button.cpp
 *
 * @brief Proxy Button class source
 *
 * @date 03/2024
 */

#include "proxy/button.hpp"

namespace proxy {
Button::Button(const Config& button_config) {
    this->subscriber = button_config.node->create_subscription<std_msgs::msg::Bool>(
        button_config.topic, 1, [this](const std_msgs::msg::Bool& msg) {
            this->state.data = msg.data;
        });
}

bool Button::get_state() const {
    return this->state.data;
}
}  // namespace proxy

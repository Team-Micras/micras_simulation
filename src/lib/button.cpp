/**
 * @file button.cpp
 *
 * @author Comp ThundeRatz <comp@thunderatz.org>
 *
 * @brief Proxy Button class source.
 *
 * @date 01/2024
 *
 * @copyright MIT License - Copyright (c) 2024 ThundeRatz
 *
 */

#include <stdio.h>

#include "proxy/button.hpp"

namespace proxy {
Button::Button(Config button_config) : rclcpp::Node(button_config.name) {
    this->subscriber =
        this->create_subscription<std_msgs::msg::Bool>(button_config.topic, 1,
                                                       [this](const std_msgs::msg::Bool::SharedPtr msg) {
                                                           this->state = msg->data;
                                                       });
}

bool Button::get_state() {
    return this->state;
}
}  // namespace proxy

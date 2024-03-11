/**
 * @file target.cpp
 *
 * @brief Target configuration constants
 *
 * @date 03/2024
 */

#include "target.hpp"

std::shared_ptr<rclcpp::Node> micras_node;

proxy::Led::Config led_config{
    micras_node, // node
    "led",       // topic
};

proxy::Button::Config button_config{
    micras_node, // node
    "button",    // topic
};

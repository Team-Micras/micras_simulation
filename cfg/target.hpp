/**
 * @file target.hpp
 *
 * @brief Target configuration constants
 *
 * @date 03/2024
 */

#ifndef __TARGET_HPP__
#define __TARGET_HPP__

#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "proxy/button.hpp"
#include "proxy/led.hpp"

extern std::shared_ptr<rclcpp::Node> micras_node;

extern proxy::Led::Config led_config;
extern proxy::Button::Config button_config;

#endif // __TARGET_HPP__

/**
 * @file button.hpp
 *
 * @brief Proxy Button class header.
 *
 * @date 03/2024
 *
 * @copyright MIT License - Copyright (c) 2024 ThundeRatz
 *
 */

#ifndef __BUTTON_HPP__
#define __BUTTON_HPP__

#include <rclcpp/rclcpp.hpp>

namespace proxy {
/**
 * @brief Class for controlling Button.
 */
class Button :
    public rclcpp::Node {
    public:
        /**
         * @brief configuration structure for button.
         */
        struct Config {
        };

        /**
         * @brief Constructor for the Button class.
         *
         * @param button_config Configuration for the button.
         */
        Button(Config button_config);

        /**
         * @brief Get the button state.
         *
         * @return true if the button is pressed, false otherwise.
         */
        bool get_state();

    private:
        std_msgs::msg::Bool state;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscriber;
};
}  // namespace proxy

#endif // __BUTTON_HPP__

/**
 * @file led.hpp
 *
 * @brief Proxy Led class header.
 *
 * @date 03/2024
 *
 * @copyright MIT License - Copyright (c) 2024 ThundeRatz
 *
 */

#ifndef __LED_HPP__
#define __LED_HPP__

#include <rclcpp/rclcpp.hpp>

namespace proxy {
/**
 * @brief Class for controlling Led.
 */
class Led :
    public rclcpp::Node {
    public:
        /**
         * @brief configuration structure for led.
         */
        struct Config {
        };

        /**
         * @brief Constructor for the Led class.
         *
         * @param led_config Configuration for the led.
         */
        Led(Config led_config);

        /**
         * @brief Turn the led on.
         */
        void turn_on();

        /**
         * @brief Turn the led off.
         */
        void turn_off();

        /**
         * @brief Toggle led.
         */
        void toggle();

    private:
        std_msgs::msg::Bool state;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher;
};
}  // namespace proxy

#endif // __LED_HPP__

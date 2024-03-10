/**
 * @file led.hpp
 *
 * @brief Proxy Led class header
 *
 * @date 03/2024
 */

#ifndef __LED_HPP__
#define __LED_HPP__

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <string>

namespace proxy {
/**
 * @brief Class for controlling an LED
 */
class Led :
    public rclcpp::Node {
    public:
        /**
         * @brief Configuration structure for led
         */
        struct Config {
            std::string name;
            std::string topic;
        };

        /**
         * @brief Constructor for the Led class
         *
         * @param led_config Configuration for the led
         */
        Led(Config led_config);

        /**
         * @brief Turn the led on
         */
        void turn_on();

        /**
         * @brief Turn the led off
         */
        void turn_off();

        /**
         * @brief Toggle led
         */
        void toggle();

    private:
        /**
         * @brief Current state of the led
         */
        std_msgs::msg::Bool state;

        /**
         * @brief Publisher for the led state topic
         */
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher;
};
}  // namespace proxy

#endif // __LED_HPP__

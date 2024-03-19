/**
 * @file button.hpp
 *
 * @brief Proxy Button class header
 *
 * @date 03/2024
 */

#ifndef __BUTTON_HPP__
#define __BUTTON_HPP__

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <string>

namespace proxy {
/**
 * @brief Class for getting button data
 */
class Button {
    public:
        /**
         * @brief Configuration structure for button
         */
        struct Config {
            std::shared_ptr<rclcpp::Node>& node;
            std::string                    topic;
        };

        /**
         * @brief Constructor for the Button class
         *
         * @param button_config Configuration for the button
         */
        Button(const Config& button_config);

        /**
         * @brief Get the button state
         *
         * @return true if the button is pressed, false otherwise
         */
        bool get_state() const;

    private:
        /**
         * @brief Current state of the button
         */
        std_msgs::msg::Bool state;

        /**
         * @brief Subscriber for the button state topic
         */
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscriber;
};
}  // namespace proxy

#endif // __BUTTON_HPP__

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
         * @brief Enum for button status
         */
        enum Status {
            NO_PRESS,
            SHORT_PRESS,
            LONG_PRESS,
            EXTRA_LONG_PRESS
        };

        /**
         * @brief Configuration structure for button
         */
        struct Config {
            std::shared_ptr<rclcpp::Node>& node;
            std::string                    topic;
            uint16_t                       long_press_delay = 1000;
            uint16_t                       extra_long_press_delay = 5000;
        };

        /**
         * @brief Constructor for Button class
         *
         * @param config Button configuration
         */
        Button(Config& config);

        /**
         * @brief Check if button is pressed
         *
         * @return True if button is pressed, false otherwise
         */
        bool is_pressed();

        /**
         * @brief Get button status
         *
         * @return Button status
         */
        Status get_status();

    private:
        /**
         * @brief Update button state
         */
        void update_state();

        /**
         * @brief Check if button was just pressed
         *
         * @return True if button was just pressed, false otherwise
         */
        bool is_rising_edge() const;

        /**
         * @brief Check if button was just released
         *
         * @return True if button was just released, false otherwise
         */
        bool is_falling_edge() const;

        /**
         * @brief Current state of the button
         */
        std_msgs::msg::Bool state;

        /**
         * @brief Subscriber for the button state topic
         */
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscriber;

        /**
         * @brief Button pressing delays in ms
         */
        const uint16_t long_press_delay;
        const uint16_t extra_long_press_delay;

        /**
         * @brief Timer for button press time
         */
        std::chrono::time_point<std::chrono::steady_clock> press_time;

        /**
         * @brief Flag to know if button was being pressed
         */
        bool previous_state = false;

        /**
         * @brief Flag to know if button is being pressed
         */
        bool current_state = false;
};
}  // namespace proxy

#endif // __BUTTON_HPP__

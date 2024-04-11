/**
 * @file button.hpp
 *
 * @brief Proxy Button class header
 *
 * @date 03/2024
 */

#ifndef MICRAS_PROXY_BUTTON_HPP
#define MICRAS_PROXY_BUTTON_HPP

#include <cstdint>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <string>

namespace micras::proxy {
/**
 * @brief Class for receiving a button data
 */
class Button {
public:
    /**
     * @brief Enum for button status
     */
    enum Status : uint8_t {
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
        uint16_t                       long_press_delay{500};
        uint16_t                       extra_long_press_delay{2000};
    };

    /**
     * @brief Constructor for Button class
     *
     * @param config Button configuration
     */
    explicit Button(const Config& config);

    /**
     * @brief Check if button is pressed
     *
     * @return bool True if button is pressed, false otherwise
     */
    bool is_pressed();

    /**
     * @brief Get button status
     *
     * @return Status Button status
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
     * @brief Node for the button
     */
    std::shared_ptr<rclcpp::Node>& node;

    /**
     * @brief Timer for button press time
     */
    rclcpp::Time press_time;

    /**
     * @brief Flag to know if button was being pressed
     */
    bool previous_state{false};

    /**
     * @brief Flag to know if button is being pressed
     */
    bool current_state{false};
};
}  // namespace micras::proxy

#endif  // MICRAS_PROXY_BUTTON_HPP

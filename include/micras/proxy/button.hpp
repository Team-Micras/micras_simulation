/**
 * @file
 */

#ifndef MICRAS_PROXY_BUTTON_HPP
#define MICRAS_PROXY_BUTTON_HPP

#include <cstdint>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include "micras/proxy/stopwatch.hpp"

namespace micras::proxy {
/**
 * @brief Class for acquiring button data.
 */
class Button {
public:
    /**
     * @brief Enum for button status.
     */
    enum Status : uint8_t {
        NO_PRESS,
        SHORT_PRESS,
        LONG_PRESS,
        EXTRA_LONG_PRESS
    };

    /**
     * @brief Enum for button pull resistor.
     */
    enum PullResistor : uint8_t {
        PULL_UP,
        PULL_DOWN,
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
     * @brief Construct a new Button object.
     *
     * @param config Button configuration.
     */
    explicit Button(const Config& config);

    /**
     * @brief Check if button is pressed.
     *
     * @return True if button is pressed, false otherwise.
     */
    bool is_pressed() const;

    /**
     * @brief Get button status.
     *
     * @return Current button status.
     */
    Status get_status();

private:
    /**
     * @brief Update button state.
     */
    void update_state();

    /**
     * @brief Check if button was just pressed.
     *
     * @return True if button was just pressed, false otherwise.
     */
    bool is_rising_edge() const;

    /**
     * @brief Check if button was just released.
     *
     * @return True if button was just released, false otherwise.
     */
    bool is_falling_edge() const;

    /**
     * @brief Subscriber para receber o estado do botão.
     */
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscriber;

    /**
     * @brief Estado atual do botão.
     */
    std_msgs::msg::Bool state;

    /**
     * @brief Button pressing delays in ms.
     */
    ///@{
    uint16_t long_press_delay;
    uint16_t extra_long_press_delay;
    ///@}

    /**
     * @brief Stopwatch to determine type of button press.
     */
    proxy::Stopwatch status_timer;

    /**
     * @brief Flag to know if button was being pressed.
     */
    bool previous_state{false};

    /**
     * @brief Flag to know if button is being pressed.
     */
    bool current_state{false};
};
}  // namespace micras::proxy

#endif  // MICRAS_PROXY_BUTTON_HPP

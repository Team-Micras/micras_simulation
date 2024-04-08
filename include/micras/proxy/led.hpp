/**
 * @file led.hpp
 *
 * @brief Proxy Led class header
 *
 * @date 03/2024
 */

#ifndef MICRAS_PROXY_LED_HPP
#define MICRAS_PROXY_LED_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <string>

namespace micras::proxy {
/**
 * @brief Class for controlling an LED
 */
class Led {
public:
    /**
     * @brief Configuration structure for LED
     */
    struct Config {
        std::shared_ptr<rclcpp::Node>& node;
        std::string                    topic;
    };

    /**
     * @brief Constructor for the Led class
     *
     * @param config Configuration for the LED
     */
    explicit Led(const Config& config);

    /**
     * @brief Turn the LED on
     */
    void turn_on();

    /**
     * @brief Turn the LED off
     */
    void turn_off();

    /**
     * @brief Toggle the LED
     */
    void toggle();

private:
    /**
     * @brief Current state of the LED
     */
    std_msgs::msg::Bool state;

    /**
     * @brief Publisher for the LED state topic
     */
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher;
};
}  // namespace micras::proxy

#endif  // MICRAS_PROXY_LED_HPP

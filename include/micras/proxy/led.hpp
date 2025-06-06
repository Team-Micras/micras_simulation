/**
 * @file
 */

#ifndef MICRAS_PROXY_LED_HPP
#define MICRAS_PROXY_LED_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

namespace micras::proxy {
/**
 * @brief Class for controlling an LED.
 */
class Led {
public:
    /**
     * @brief Configuration struct for LED.
     */
    struct Config {
        // NOLINTNEXTLINE(cppcoreguidelines-avoid-const-or-ref-data-members)
        std::shared_ptr<rclcpp::Node>& node;
        std::string                    topic;
    };

    /**
     * @brief Construct a new Led object.
     *
     * @param config Configuration for the LED.
     */
    explicit Led(const Config& config);

    /**
     * @brief Turn the LED on.
     */
    void turn_on();

    /**
     * @brief Turn the LED off.
     */
    void turn_off();

    /**
     * @brief Toggle the LED.
     */
    void toggle();

private:
    /**
     * @brief Publisher para controlar o LED.
     */
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher;

    /**
     * @brief Estado atual do LED.
     */
    std_msgs::msg::Bool state;
};
}  // namespace micras::proxy

#endif  // MICRAS_PROXY_LED_HPP

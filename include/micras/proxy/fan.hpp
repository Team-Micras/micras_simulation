/**
 * @file fan.hpp
 *
 * @brief Proxy Fan class declaration
 *
 * @date 03/2024
 */

#ifndef MICRAS_PROXY_FAN_HPP
#define MICRAS_PROXY_FAN_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <string>
#include <cstdint>

namespace micras::proxy {
/**
 * @brief Class for controlling the fan driver
 */
class Fan {
public:
    /**
     * @brief Configuration structure for the fan
     */
    struct Config {
        std::shared_ptr<rclcpp::Node>& node;
        std::string                    topic;
    };

    /**
     * @brief Construct a new fan object
     *
     * @param config Configuration for the fan driver
     */
    explicit Fan(const Config& config);

    /**
     * @brief Enable the fan
     */
    void enable();

    /**
     * @brief Disable the fan
     */
    void disable();

    /**
     * @brief Set the speed of the fans
     *
     * @param speed Speed percentage of the fan
     */
    void set_speed(float speed);

    /**
     * @brief Stop the fan
     */
    void stop();

private:
    /**
     * @brief Flag to check if the fan is enabled
     */
    bool enabled = false;

    /**
     * @brief Current fan speed
     */
    std_msgs::msg::Float32 fan_speed;

    /**
     * @brief Publisher for the fan speed topic
     */
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher;
};
}  // namespace micras::proxy

#endif  // MICRAS_PROXY_FAN_HPP

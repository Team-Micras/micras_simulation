/**
 * @file battery.hpp
 *
 * @brief Proxy Battery class declaration
 *
 * @date 04/2024
 */

#ifndef MICRAS_PROXY_BATTERY_HPP
#define MICRAS_PROXY_BATTERY_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <string>
#include <cstdint>

namespace micras::proxy {
/**
 * @brief Class for getting the battery voltage
 */
class Battery {
public:
    /**
     * @brief Configuration structure for the battery
     */
    struct Config {
        std::shared_ptr<rclcpp::Node>& node;
        std::string                    topic;
    };

    /**
     * @brief Constructor for the Battery class
     *
     * @param config Configuration for the battery
     */
    explicit Battery(const Config& config);

    /**
     * @brief Get the battery voltage
     *
     * @return float Battery voltage in volts
     */
    float get_voltage() const;

    /**
     * @brief Get the raw reading from the battery
     *
     * @return uint32_t Raw reading from the battery
     */
    uint32_t get_voltage_raw() const;

private:
    /**
     * @brief Reading from the battery
     */
    uint32_t reading{};

    /**
     * @brief Subscriber for the battery topic
     */
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscriber;
};
}  // namespace micras::proxy

#endif  // MICRAS_PROXY_BATTERY_HPP

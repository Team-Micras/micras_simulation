/**
 * @file
 */

#ifndef MICRAS_PROXY_ROTARY_SENSOR_HPP
#define MICRAS_PROXY_ROTARY_SENSOR_HPP

#include <cstdint>
#include <example_interfaces/msg/float64.hpp>
#include <rclcpp/rclcpp.hpp>

namespace micras::proxy {
/**
 * @brief Class for acquiring rotary sensor data.
 */
class RotarySensor {
public:
    /**
     * @brief Rotary sensor configuration struct.
     */
    struct Config {
        // NOLINTNEXTLINE(cppcoreguidelines-avoid-const-or-ref-data-members)
        std::shared_ptr<rclcpp::Node>& node;
        std::string                    topic;
    };

    /**
     * @brief Construct a new RotarySensor object.
     *
     * @param config Configuration for the rotary sensor.
     */
    explicit RotarySensor(const Config& config);

    /**
     * @brief Get the rotary sensor position over an axis.
     *
     * @return Current angular position of the sensor in radians.
     */
    float get_position() const;

private:
    /**
     * @brief Subscription to the rotary sensor topic.
     */
    rclcpp::Subscription<example_interfaces::msg::Float64>::SharedPtr subscriber;

    /**
     * @brief Received data from the rotary sensor.
     */
    double data{0.0};
};
}  // namespace micras::proxy

#endif  // MICRAS_PROXY_ROTARY_SENSOR_HPP

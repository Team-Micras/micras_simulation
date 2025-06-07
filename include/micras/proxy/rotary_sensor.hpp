/**
 * @file
 */

#ifndef MICRAS_PROXY_ROTARY_SENSOR_HPP
#define MICRAS_PROXY_ROTARY_SENSOR_HPP

#include <cstdint>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

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
     * @brief Subscriber para os dados do sensor rotativo.
     */
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscriber;

    /**
     * @brief Dados do sensor rotativo recebidos.
     */
    sensor_msgs::msg::JointState data;
};
}  // namespace micras::proxy

#endif  // MICRAS_PROXY_ROTARY_SENSOR_HPP

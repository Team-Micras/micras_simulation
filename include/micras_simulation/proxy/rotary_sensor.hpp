/**
 * @file rotary_sensor.hpp
 *
 * @brief STM32 rotary sensor HAL wrapper
 *
 * @date 04/2024
 */

#ifndef MICRAS_PROXY_ROTARY_SENSOR_HPP
#define MICRAS_PROXY_ROTARY_SENSOR_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <string>

namespace proxy {
/**
 * @brief Class to handle rotary sensor
 */
class RotarySensor {
    public:
        /**
         * @brief Rotary sensor configuration struct
         */
        struct Config {
            std::shared_ptr<rclcpp::Node>& node;
            std::string                    topic;
        };

        /**
         * @brief Construct a new RotarySensor object
         *
         * @param config Configuration for the rotary sensor
         */
        explicit RotarySensor(const Config& config);

        /**
         * @brief Get the rotary sensor position over an axis
         *
         * @return float Current angular position of the sensor in radians
         */
        float get_position() const;

    private:
        /**
         * @brief Current data of the encoder
         */
        sensor_msgs::msg::JointState data;

        /**
         * @brief Subscriber for the encoder topic
         */
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscriber;
};
}  // namespace proxy

#endif // MICRAS_PROXY_ROTARY_SENSOR_HPP

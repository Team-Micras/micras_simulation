/**
 * @file rotary_sensor.hpp
 *
 * @brief STM32 rotary sensor HAL wrapper
 *
 * @date 03/2024
 */

#ifndef __ROTARY_SENSOR_HPP__
#define __ROTARY_SENSOR_HPP__

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <string>

namespace proxy {
/**
 * @brief Class to handle rotary sensor peripheral on STM32 microcontrollers
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
        RotarySensor(Config& config);

        /**
         * @brief Get the rotary sensor position over an axis
         *
         * @return Current angular position of the sensor in radians
         */
        float get_position();

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

#endif // __ROTARY_SENSOR_HPP__

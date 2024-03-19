/**
 * @file encoder.hpp
 *
 * @brief Proxy Encoder class header
 *
 * @date 03/2024
 */

#ifndef __ENCODER_HPP__
#define __ENCODER_HPP__

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <string>

namespace proxy {
/**
 * @brief Class for getting encoder data
 */
class Encoder {
    public:
        /**
         * @brief Configuration structure for the encoder
         */
        struct Config {
            std::shared_ptr<rclcpp::Node>& node;
            std::string                    topic;
        };

        /**
         * @brief Constructor for the Encoder class
         *
         * @param encoder_config Configuration for the encoder
         */
        Encoder(const Config& encoder_config);

        /**
         * @brief Get the encoder absolute position
         *
         * @return Encoder absolute position in radians
         */
        double get_position() const;

        /**
         * @brief Get the encoder velocity
         *
         * @return Encoder velocity in rad/s
         */
        double get_velocity() const;

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

#endif // __ENCODER_HPP__

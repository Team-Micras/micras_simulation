/**
 * @file locomotion.hpp
 *
 * @brief Proxy Locomotion class header
 *
 * @date 03/2024
 */

#ifndef __LOCOMOTION_HPP__
#define __LOCOMOTION_HPP__

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <string>

namespace proxy {
/**
 * @brief Class for controlling the robot locomotion
 */
class Locomotion {
    public:
        /**
         * @brief Configuration structure for the locomotion
         */
        struct Config {
            std::shared_ptr<rclcpp::Node>& node;
            std::string                    topic;
        };

        /**
         * @brief Constructor for the Locomotion class
         *
         * @param locomotion_config Configuration for the locomotion
         */
        Locomotion(const Config& locomotion_config);

        /**
         * @brief Set the velocity command of the robot
         *
         * @param linear Linear velocity command
         * @param angular Angular velocity command
         */
        void set_velocity_command(double linear, double angular);

    private:
        /**
         * @brief Current velocity command of the robot
         */
        geometry_msgs::msg::Twist twist;

        /**
         * @brief Publisher for the velocity command topic
         */
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher;
};
}  // namespace proxy

#endif // __LOCOMOTION_HPP__

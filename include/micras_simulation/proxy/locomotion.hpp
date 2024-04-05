/**
 * @file locomotion.hpp
 *
 * @brief Proxy Locomotion class declaration
 *
 * @date 04/2024
 */

#ifndef MICRAS_PROXY_LOCOMOTION_HPP
#define MICRAS_PROXY_LOCOMOTION_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <string>

namespace proxy {
/**
 * @brief Class for controlling the locomotion driver
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
         * @brief Construct a new locomotion object
         *
         * @param config Configuration for the locomotion driver
         */
        explicit Locomotion(const Config& config);

        /**
         * @brief Enable the locomotion driver
         */
        void enable();

        /**
         * @brief Disable the locomotion driver
         */
        void disable();

        /**
         * @brief Set the speed of the wheels
         *
         * @param left_speed Speed of the left wheels
         * @param right_speed Speed of the right wheels
         */
        void set_wheel_speed(float left_speed, float right_speed);

        /**
         * @brief Set the linear and angular speeds of the robot
         *
         * @param linear Linear speed of the robot
         * @param angular Angular speed of the robot
         */
        void set_speed(float linear, float angular);

        /**
         * @brief Stop the motors
         */
        void stop();

        /**
         * @brief Stop the left motor
         */
        void stop_left();

        /**
         * @brief Stop the right motor
         */
        void stop_right();

    private:
        /**
         * @brief Flag to enable/disable the locomotion driver
         */
        bool enabled;

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

#endif // MICRAS_PROXY_LOCOMOTION_HPP

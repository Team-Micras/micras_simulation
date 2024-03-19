/**
 * @file odometry.hpp
 *
 * @brief Proxy Odometry class header
 *
 * @date 03/2024
 */

#ifndef __ODOMETRY_HPP__
#define __ODOMETRY_HPP__

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <string>

namespace proxy {
/**
 * @brief Class for getting odometry data
 */
class Odometry {
    public:
        /**
         * @brief Configuration structure for the odometry
         */
        struct Config {
            std::shared_ptr<rclcpp::Node>& node;
            std::string                    topic;
        };

        /**
         * @brief Constructor for the Odometry class
         *
         * @param odometry_config Configuration for the odometry
         */
        Odometry(const Config& odometry_config);

        /**
         * @brief Get the odometry x position
         *
         * @return Current robot x position in meters
         */
        double get_x_position() const;

        /**
         * @brief Get the odometry y position
         *
         * @return Current robot y position in meters
         */
        double get_y_position() const;

        /**
         * @brief Get the odometry orientation
         *
         * @return Current robot orientation in radians
         */
        double get_orientation() const;

        /**
         * @brief Get the odometry linear velocity
         *
         * @return Current robot linear velocity in m/s
         */
        double get_linear_velocity() const;

        /**
         * @brief Get the odometry angular velocity
         *
         * @return Current robot angular velocity in rad/s
         */
        double get_angular_velocity() const;

    private:
        /**
         * @brief Current data of the odometry
         */
        nav_msgs::msg::Odometry data;

        /**
         * @brief Subscriber for the odometry topic
         */
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber;
};
}  // namespace proxy

#endif // __ODOMETRY_HPP__

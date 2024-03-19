/**
 * @file imu.hpp
 *
 * @brief Proxy Imu class header
 *
 * @date 03/2024
 */

#ifndef __IMU_HPP__
#define __IMU_HPP__

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <string>

namespace proxy {
/**
 * @brief Class for getting IMU data
 */
class Imu {
    public:
        /**
         * @brief Configuration structure for the IMU
         */
        struct Config {
            std::shared_ptr<rclcpp::Node>& node;
            std::string                    topic;
        };

        enum Axis {
            W,
            X,
            Y,
            Z
        };

        /**
         * @brief Constructor for the Imu class
         *
         * @param imu_config Configuration for the IMU
         */
        Imu(const Config& imu_config);

        /**
         * @brief Get the IMU orientation over an axis
         *
         * @param axis Axis to get the orientation from
         *
         * @return Orientation over the desired axis in radians
         */
        double get_orientation(Axis axis) const;

        /**
         * @brief Get the IMU angular_velocity over an axis
         *
         * @param axis Axis to get the angular_velocity from
         *
         * @return Angular velocity over the desired axis in rad/s
         */
        double get_angular_velocity(Axis axis) const;

        /**
         * @brief Get the IMU linear_acceleration over an axis
         *
         * @param axis Axis to get the linear_acceleration from
         *
         * @return Linear acceleration over the desired axis in m/s²
         */
        double get_linear_acceleration(Axis axis) const;

    private:
        /**
         * @brief Current data of the IMU
         */
        sensor_msgs::msg::Imu data;

        /**
         * @brief Subscriber for the IMU topic
         */
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscriber;
};
}  // namespace proxy

#endif // __IMU_HPP__

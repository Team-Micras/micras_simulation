/**
 * @file imu.hpp
 *
 * @brief STM32 IMU HAL wrapper
 *
 * @date 03/2024
 */

#ifndef MICRAS_PROXY_IMU_HPP
#define MICRAS_PROXY_IMU_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <string>

namespace micras::proxy {
/**
 * @brief Class to handle IMU peripheral on STM32 microcontrollers
 */
class Imu {
    public:
        /**
         * @brief IMU configuration struct
         */
        struct Config {
            std::shared_ptr<rclcpp::Node>& node;
            std::string                    topic;
        };

        /**
         * @brief Construct a new Imu object
         *
         * @param config Configuration for the IMU
         */
        explicit Imu(const Config& config);

        enum Axis {
            W,
            X,
            Y,
            Z
        };

        /**
         * @brief Update the IMU data
         */
        void update_data();

        /**
         * @brief Get the IMU orientation over an axis
         * @todo implement function using sensior fusion
         *
         * @param axis Axis to get the orientation from
         *
         * @return float Orientation over the desired axis using quaternions
         */
        float get_orientation(Axis axis) const;

        /**
         * @brief Get the IMU angular velocity over an axis
         *
         * @param axis Axis to get the angular velocity from
         *
         * @return float Angular velocity over the desired axis in rad/s
         */
        float get_angular_velocity(Axis axis) const;

        /**
         * @brief Get the IMU linear acceleration over an axis
         *
         * @param axis Axis to get the linear acceleration from
         *
         * @return float Linear acceleration over the desired axis in m/s²
         */
        float get_linear_acceleration(Axis axis) const;

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

#endif // MICRAS_PROXY_IMU_HPP

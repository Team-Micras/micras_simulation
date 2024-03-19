/**
 * @file torque_sensor.hpp
 *
 * @brief Proxy Torque Sensor class header
 *
 * @date 03/2024
 */

#ifndef __TORQUE_SENSOR_HPP__
#define __TORQUE_SENSOR_HPP__

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <string>

namespace proxy {
/**
 * @brief Class for getting torque sensor data
 */
class TorqueSensor {
    public:
        /**
         * @brief Configuration structure for the torque sensor
         */
        struct Config {
            std::shared_ptr<rclcpp::Node>& node;
            std::string                    front_topic;
            std::string                    rear_topic;
        };

        /**
         * @brief Constructor for the TorqueSensor class
         *
         * @param torque_sensor_config Configuration for the torque sensor
         */
        TorqueSensor(const Config& torque_sensor_config);

        /**
         * @brief Get the torque sensor current torque
         *
         * @return Torque measured by the torque sensor in N*m
         */
        double get_torque() const;

    private:
        /**
         * @brief Current torque measured by the torque sensor in the front wheel
         */
        double front_torque;

        /**
         * @brief Current torque measured by the torque sensor in the rear wheel
         */
        double rear_torque;

        /**
         * @brief Subscriber for the front torque sensor topic
         */
        rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr front_subscriber;

        /**
         * @brief Subscriber for the rear torque sensor topic
         */
        rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr rear_subscriber;
};
}  // namespace proxy

#endif // __TORQUE_SENSOR_HPP__

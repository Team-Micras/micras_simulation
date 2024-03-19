/**
 * @file distance_sensor.hpp
 *
 * @brief Proxy Distance Sensor class header
 *
 * @date 03/2024
 */

#ifndef __DISTANCE_SENSOR_HPP__
#define __DISTANCE_SENSOR_HPP__

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <string>

namespace proxy {
/**
 * @brief Class for getting distance sensor data
 */
class DistanceSensor {
    public:
        /**
         * @brief Configuration structure for the distance sensor
         */
        struct Config {
            std::shared_ptr<rclcpp::Node>& node;
            std::string                    topic;
        };

        /**
         * @brief Constructor for the DistanceSensor class
         *
         * @param distance_sensor_config Configuration for the distance sensor
         */
        DistanceSensor(const Config& distance_sensor_config);

        /**
         * @brief Get the distance sensor current distance
         *
         * @return Distance measured by the distance sensor in meters
         */
        double get_distance() const;

    private:
        /**
         * @brief Current distance measured by the distance sensor
         */
        double distance;

        /**
         * @brief Subscriber for the distance sensor topic
         */
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber;
};
}  // namespace proxy

#endif // __DISTANCE_SENSOR_HPP__

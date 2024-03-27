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
#include <array>

namespace proxy {
/**
 * @brief Class for controlling DistanceSensors
 */
template <uint8_t num_of_sensors>
class DistanceSensors {
    public:
        /**
         * @brief configuration structure for distance sensors
         */
        struct Config {
            std::shared_ptr<rclcpp::Node>&          node;
            std::array<std::string, num_of_sensors> topic_array;

            // std::string                    topic;
            float                                   max_distance;
        };

        /**
         * @brief Constructor for the DistanceSensors class
         *
         * @param config Configuration for the distance sensors
         */
        DistanceSensors(Config& config);

        /**
         * @brief Set the distance sensors led intensity
         *
         * @param intensity Intensity percentage of the infrared LED
         */
        void set_led_intensity(float intensity);

        /**
         * @brief Get the distance from a sensor
         *
         * @param sensor_index Index of the sensor
         * @return float Distance reading from the sensors
         */
        float get_distance(uint8_t sensor_index) const;

        /**
         * @brief Get the distance from a sensor
         *
         * @param sensor_index Index of the sensor
         * @return uint32_t Raw reading from the distance sensor
         */
        uint32_t get_distance_raw(uint8_t sensor_index) const;

    private:
        /**
         * @brief Current distance measured by the distance sensor
         */

        // double distance;

        std::array<float, num_of_sensors> distances;

        /**
         * @brief Subscriber for the distance sensor topic
         */

        // rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber;

        std::array<rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr> subscribers;
};
}  // namespace proxy

#include "../../../src/lib/distance_sensor.cpp"

#endif // __DISTANCE_SENSOR_HPP__

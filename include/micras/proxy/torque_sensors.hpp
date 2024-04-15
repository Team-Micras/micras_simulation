/**
 * @file torque_sensors.hpp
 *
 * @brief Proxy TorqueSensors class header
 *
 * @date 04/2024
 */

#ifndef MICRAS_PROXY_TORQUE_SENSORS_HPP
#define MICRAS_PROXY_TORQUE_SENSORS_HPP

#include <cstdint>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

namespace micras::proxy {
/**
 * @brief Class for receiving torque sensors data
 */
template <uint8_t num_of_sensors>
class TorqueSensors {
public:
    /**
     * @brief Configuration structure for torque sensors
     */
    struct Config {
        std::shared_ptr<rclcpp::Node>&                         node;
        std::array<std::array<std::string, 2>, num_of_sensors> wheel_pairs_topics;
        float                                                  shunt_resistor;
        float                                                  max_torque;
        float                                                  reference_voltage;
    };

    /**
     * @brief Constructor for the TorqueSensors class
     *
     * @param config Configuration for the torque sensors
     */
    explicit TorqueSensors(const Config& config);

    /**
     * @brief Get the torque from the sensor
     *
     * @param sensor_index Index of the sensor
     * @return float Torque reading from the sensor in N*m
     */
    float get_torque(uint8_t sensor_index) const;

    /**
     * @brief Get the current from the sensor
     *
     * @param sensor_index Index of the sensor
     * @return float Current reading from the sensor in amps
     */
    float get_current(uint8_t sensor_index) const;

private:
    /**
     * @brief Structure for a pair of wheels topics subscribers
     */
    struct WheelPair {
        float                                                              front_torque{};
        float                                                              rear_torque{};
        rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr front_subscriber;
        rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr rear_subscriber;
    };

    /**
     * @brief Array of wheel pairs
     */
    std::array<WheelPair, num_of_sensors> wheel_pairs;

    /**
     * @brief Value of the shunt resistor in ohms
     */
    const float shunt_resistor;

    /**
     * @brief Maximum torque that can be measured by the sensor
     */
    const float max_torque;

    /**
     * @brief Reference voltage for the ADC conversion
     */
    const float reference_voltage;
};
}  // namespace micras::proxy

#include "../src/proxy/torque_sensors.cpp"  // NOLINT(bugprone-suspicious-include)

#endif  // MICRAS_PROXY_TORQUE_SENSORS_HPP

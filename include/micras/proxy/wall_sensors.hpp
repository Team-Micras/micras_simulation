/**
 * @file
 */

#ifndef MICRAS_PROXY_WALL_SENSORS_HPP
#define MICRAS_PROXY_WALL_SENSORS_HPP

#include <array>
#include <cstdint>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "micras/core/utils.hpp"
#include "micras/core/types.hpp"

namespace micras::proxy {
/**
 * @brief Class for controlling Wall Sensors.
 */
template <uint8_t num_of_sensors>
class TWallSensors {
public:
    /**
     * @brief Configuration struct for wall sensors.
     */
    struct Config {
        std::shared_ptr<rclcpp::Node>&          node;
        std::array<std::string, num_of_sensors> topic_array;
        float                                   max_distance;
        uint16_t                                max_reading;
    };

    /**
     * @brief Construct a new WallSensors object.
     *
     * @param config Configuration for the wall sensors.
     */
    explicit TWallSensors(const Config& config);

    /**
     * @brief Turn on the wall sensors IR LED.
     */
    void turn_on();

    /**
     * @brief Turn off the wall sensors IR LED.
     */
    void turn_off();

    /**
     * @brief Update the wall sensors readings.
     */
    void update();

    /**
     * @brief Get the observation from a sensor.
     *
     * @param sensor_index Index of the sensor.
     * @return Observation from the sensor.
     */
    core::Observation get_observation(uint8_t sensor_index) const;

    /**
     * @brief Get the reading from a sensor.
     *
     * @param sensor_index Index of the sensor.
     * @return Reading from the sensor.
     */
    float get_reading(uint8_t sensor_index) const;

    /**
     * @brief Get the ADC reading from a sensor.
     *
     * @param sensor_index Index of the sensor.
     * @return ADC reading from the sensor from 0 to 1.
     */
    float get_adc_reading(uint8_t sensor_index) const;

    /**
     * @brief Calibrate the wall sensors for a wall at the front.
     */
    void calibrate_front_wall();

    /**
     * @brief Calibrate the wall sensors for a wall at the left.
     */
    void calibrate_left_wall();

    /**
     * @brief Calibrate the wall sensors for a wall at the right.
     */
    void calibrate_right_wall();

    /**
     * @brief Calibrate the wall sensors for free space at the front.
     */
    void calibrate_front_free_space();

    /**
     * @brief Calibrate the wall sensors for free space at the left.
     */
    void calibrate_left_free_space();

    /**
     * @brief Calibrate the wall sensors for free space at the right.
     */
    void calibrate_right_free_space();

    /**
     * @brief Update the wall sensors thresholds.
     */
    void update_thresholds();

private:
    /**
     * @brief Array de subscribers para os sensores de distância.
     */
    std::array<rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr, num_of_sensors> subscribers;

    /**
     * @brief Array de distâncias medidas pelos sensores.
     */
    std::array<float, num_of_sensors> distances;

    /**
     * @brief Distância máxima detectável pelo sensor.
     */
    const float max_distance;

    /**
     * @brief Leitura máxima do ADC.
     */
    const uint16_t max_reading;

    /**
     * @brief Estado dos LEDs.
     */
    bool leds_on{true};

    /**
     * @brief Uncertainty of the wall sensors.
     */
    float uncertainty{0.1f};

    /**
     * @brief Measured wall value during calibration.
     */
    std::array<float, num_of_sensors> wall_calibration_measure{};

    /**
     * @brief Measured free space value during calibration.
     */
    std::array<float, num_of_sensors> free_space_calibration_measure{};

    /**
     * @brief Minimum reading value to identify a wall.
     */
    std::array<float, num_of_sensors> wall_threshold{};

    /**
     * @brief Maximum reading value to identify a free space.
     */
    std::array<float, num_of_sensors> free_space_threshold{};
};
}  // namespace micras::proxy

#include "../src/proxy/wall_sensors.cpp"  // NOLINT(bugprone-suspicious-include, misc-header-include-cycle)

#endif  // MICRAS_PROXY_WALL_SENSORS_HPP

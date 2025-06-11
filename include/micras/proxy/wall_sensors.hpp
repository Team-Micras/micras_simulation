/**
 * @file
 */

#ifndef MICRAS_PROXY_WALL_SENSORS_HPP
#define MICRAS_PROXY_WALL_SENSORS_HPP

#include <array>
#include <cstdint>
#include <example_interfaces/msg/float64.hpp>
#include <rclcpp/rclcpp.hpp>

#include "micras/core/butterworth_filter.hpp"

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
        // NOLINTNEXTLINE(cppcoreguidelines-avoid-const-or-ref-data-members)
        std::shared_ptr<rclcpp::Node>&          node;
        std::array<std::string, num_of_sensors> topic_array;
        float                                   uncertainty;
        std::array<float, num_of_sensors>       base_readings;
        float                                   max_sensor_reading;
        float                                   min_sensor_reading;
        float                                   max_sensor_distance;
        float                                   filter_cutoff;
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
     * @param disturbed Whether or not there is another wall perpendicular to the one being measured.
     * @return True if the sensor detects a wall, false otherwise.
     */
    bool get_wall(uint8_t sensor_index, bool disturbed = false) const;

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
     * @brief Get the deviation of a wall sensor reading from its calibrated baseline.
     *
     * @param sensor_index Index of the sensor.
     * @return The reading error relative to the baseline; positive if above baseline.
     */
    float get_sensor_error(uint8_t sensor_index) const;

    /**
     * @brief Calibrate a wall sensor base reading.
     */
    void calibrate_sensor(uint8_t sensor_index);

private:
    /**
     * @brief Array of subscribers for distance sensors.
     */
    std::array<rclcpp::Subscription<example_interfaces::msg::Float64>::SharedPtr, num_of_sensors> subscribers;

    /**
     * @brief Array of distances measured by the sensors.
     */
    mutable std::array<float, num_of_sensors> readings;

    /**
     * @brief Maximum distance detectable by the sensor.
     */
    const float max_sensor_reading;

    /**
     * @brief LED state.
     */
    bool leds_on{true};

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

    /**
     * @brief Buffer to store the ADC values.
     */
    std::array<uint16_t, 2 * num_of_sensors> buffer;

    /**
     * @brief Butterworth filter for the ADC readings.
     */
    std::array<core::ButterworthFilter, num_of_sensors> filters;

    /**
     * @brief Measured wall values during calibration.
     */
    std::array<float, num_of_sensors> base_readings;

    /**
     * @brief Ratio of the base reading to still consider as seeing a wall.
     */
    float uncertainty;

    /**
     * @brief Constant value for the wall sensor.
     */
    float constant;
};
}  // namespace micras::proxy

#include "../src/proxy/wall_sensors.cpp"  // NOLINT(bugprone-suspicious-include, misc-header-include-cycle)

#endif  // MICRAS_PROXY_WALL_SENSORS_HPP

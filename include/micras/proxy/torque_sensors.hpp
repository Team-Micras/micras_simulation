/**
 * @file
 */

#ifndef MICRAS_PROXY_TORQUE_SENSORS_HPP
#define MICRAS_PROXY_TORQUE_SENSORS_HPP

#include <array>
#include <cstdint>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>

namespace micras::proxy {
/**
 * @brief Class for acquiring torque sensors data.
 */
template <uint8_t num_of_wheel_pairs>
class TTorqueSensors {
public:
    /**
     * @brief Estrutura para armazenar dados de um par de rodas.
     */
    struct WheelPair {
        rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr front_subscriber;
        rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr rear_subscriber;
        float                                                              front_torque{0.0f};
        float                                                              rear_torque{0.0f};
    };

    /**
     * @brief Configuration struct for torque sensors.
     */
    struct Config {
        std::shared_ptr<rclcpp::Node>&                             node;
        std::array<std::array<std::string, 2>, num_of_wheel_pairs> wheel_pairs_topics;
        float                                                      shunt_resistor;
        float                                                      max_torque;
        float                                                      reference_voltage;
    };

    /**
     * @brief Construct a new TorqueSensors object.
     *
     * @param config Configuration for the torque sensors.
     */
    explicit TTorqueSensors(const Config& config);

    /**
     * @brief Calibrate the torque sensors.
     */
    void calibrate();

    /**
     * @brief Update the torque sensors readings.
     */
    void update();

    /**
     * @brief Get the torque from the sensor.
     *
     * @param sensor_index Index of the sensor.
     * @return Torque reading from the sensor in N*m.
     */
    float get_torque(uint8_t sensor_index) const;

    /**
     * @brief Get the raw torque from the sensor without filtering.
     *
     * @param sensor_index Index of the sensor.
     * @return Raw torque reading from the sensor.
     */
    float get_torque_raw(uint8_t sensor_index) const;

    /**
     * @brief Get the electric current through the sensor.
     *
     * @param sensor_index Index of the sensor.
     * @return Current reading from the sensor in amps.
     */
    float get_current(uint8_t sensor_index) const;

    /**
     * @brief Get the raw electric current through the sensor without filtering.
     *
     * @param sensor_index Index of the sensor.
     * @return Raw current reading from the sensor in amps.
     */
    float get_current_raw(uint8_t sensor_index) const;

    /**
     * @brief Get the ADC reading from the sensor.
     *
     * @param sensor_index Index of the sensor.
     * @return Adc reading from the sensor from 0 to 1.
     */
    float get_adc_reading(uint8_t sensor_index) const;

private:
    /**
     * @brief Array de pares de rodas.
     */
    std::array<WheelPair, num_of_wheel_pairs> wheel_pairs;

    /**
     * @brief Resistor shunt usado para medir corrente.
     */
    const float shunt_resistor;

    /**
     * @brief Maximum torque that can be measured by the sensor.
     */
    const float max_torque;

    /**
     * @brief Tensão de referência para cálculo da corrente.
     */
    const float reference_voltage;
};
}  // namespace micras::proxy

#include "../src/proxy/torque_sensors.cpp"  // NOLINT(bugprone-suspicious-include, misc-header-include-cycle)

#endif  // MICRAS_PROXY_TORQUE_SENSORS_HPP

/**
 * @file
 */

#ifndef MICRAS_PROXY_BATTERY_HPP
#define MICRAS_PROXY_BATTERY_HPP

#include <cstdint>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>

namespace micras::proxy {
/**
 * @brief Class for getting the battery voltage.
 */
class Battery {
public:
    /**
     * @brief Configuration struct for the battery.
     */
    struct Config {
        std::shared_ptr<rclcpp::Node>& node;
        std::string                    topic;
        float                          max_voltage;
        uint16_t                       max_reading;
    };

    /**
     * @brief Construct a new Battery object.
     *
     * @param config Configuration for the battery.
     */
    explicit Battery(const Config& config);

    /**
     * @brief Update the battery reading.
     */
    void update();

    /**
     * @brief Get the battery voltage.
     *
     * @return Battery voltage in volts.
     */
    float get_voltage() const;

    /**
     * @brief Get the battery voltage in volts without the filter applied.
     *
     * @return Battery voltage in volts.
     */
    float get_voltage_raw() const;

    /**
     * @brief Get the battery reading from the ADC.
     *
     * @return Battery reading from 0 to 1.
     */
    float get_adc_reading() const;

private:
    /**
     * @brief Reading from the battery
     */
    float reading{};

    /**
     * @brief Subscriber for the battery topic
     */
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscriber;

    /**
     * @brief Max battery voltage for the adc conversion
     */
    const float max_voltage;

    /**
     * @brief Max adc reading for the battery
     */
    const uint16_t max_reading;
};
}  // namespace micras::proxy

#endif  // MICRAS_PROXY_BATTERY_HPP

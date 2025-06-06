/**
 * @file
 */

#ifndef MICRAS_PROXY_MOTOR_HPP
#define MICRAS_PROXY_MOTOR_HPP

#include <rclcpp/rclcpp.hpp>
#include <example_interfaces/msg/float64.hpp>

namespace micras::proxy {
/**
 * @brief Class for controlling a motor driver.
 */
class Motor {
public:
    /**
     * @brief Configuration struct for the motor.
     */
    struct Config {
        // NOLINTNEXTLINE(cppcoreguidelines-avoid-const-or-ref-data-members)
        std::shared_ptr<rclcpp::Node>& node;
        std::string                    topic;
    };

    /**
     * @brief Construct a new motor object.
     *
     * @param config Configuration for the motor driver.
     */
    explicit Motor(const Config& config);

    /**
     * @brief Set the command for the motor.
     *
     * @param command Command for the motor in percentage.
     */
    void set_command(float command);

private:
    /**
     * @brief Publisher for the motor command.
     */
    rclcpp::Publisher<example_interfaces::msg::Float64>::SharedPtr publisher;

    /**
     * @brief Command message for the motor.
     */
    example_interfaces::msg::Float64 command;
};
}  // namespace micras::proxy

#endif  // MICRAS_PROXY_MOTOR_HPP

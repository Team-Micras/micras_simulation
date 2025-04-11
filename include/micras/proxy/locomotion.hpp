/**
 * @file
 */

#ifndef MICRAS_PROXY_LOCOMOTION_HPP
#define MICRAS_PROXY_LOCOMOTION_HPP

#include <cstdint>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

namespace micras::proxy {
/**
 * @brief Class for controlling the locomotion driver.
 */
class Locomotion {
public:
    /**
     * @brief Configuration struct for the locomotion.
     */
    struct Config {
        std::shared_ptr<rclcpp::Node>& node;
        std::string                    topic;
    };

    /**
     * @brief Construct a new locomotion object.
     *
     * @param config Configuration for the locomotion driver.
     */
    explicit Locomotion(const Config& config);

    /**
     * @brief Enable the locomotion driver.
     */
    void enable();

    /**
     * @brief Disable the locomotion driver.
     */
    void disable();

    /**
     * @brief Set the command of the wheels.
     *
     * @param left_command Command of the left wheels.
     * @param right_command Command of the right wheels.
     */
    void set_wheel_command(float left_command, float right_command);

    /**
     * @brief Set the linear and angular commands of the robot.
     *
     * @param linear Linear command of the robot.
     * @param angular Angular command of the robot.
     */
    void set_command(float linear, float angular);

    /**
     * @brief Stop the motors.
     */
    void stop();

private:
    /**
     * @brief Publisher para enviar comandos de movimento.
     */
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher;

    /**
     * @brief Mensagem twist para controle de movimento.
     */
    geometry_msgs::msg::Twist twist;

    /**
     * @brief Estado de habilitação dos motores.
     */
    bool enabled{false};
};
}  // namespace micras::proxy

#endif  // MICRAS_PROXY_LOCOMOTION_HPP

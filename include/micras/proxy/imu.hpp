/**
 * @file
 */

#ifndef MICRAS_PROXY_IMU_HPP
#define MICRAS_PROXY_IMU_HPP

#include <array>
#include <cstdint>
#include <numbers>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace micras::proxy {
/**
 * @brief Class for acquiring IMU data.
 */
class Imu {
public:
    /**
     * @brief IMU configuration struct.
     */
    struct Config {
        std::shared_ptr<rclcpp::Node>& node;
        std::string                    topic;
    };

    /**
     * @brief Enum to select the axis of the IMU.
     */
    enum Axis : uint8_t {
        X = 0,
        Y = 1,
        Z = 2
    };

    /**
     * @brief Construct a new Imu object.
     *
     * @param config Configuration for the IMU.
     */
    explicit Imu(const Config& config);

    /**
     * @brief Update the IMU data.
     */
    void update();

    /**
     * @brief Get the IMU angular velocity over an axis.
     *
     * @param axis Axis to get the angular velocity from.
     * @return Angular velocity over the desired axis in rad/s.
     */
    float get_angular_velocity(Axis axis) const;

    /**
     * @brief Get the IMU linear acceleration over an axis.
     *
     * @param axis Axis to get the linear acceleration from.
     * @return Linear acceleration over the desired axis in m/s².
     */
    float get_linear_acceleration(Axis axis) const;

    /**
     * @brief Define the base reading to be removed from the IMU value.
     */
    void calibrate();

    /**
     * @brief Check if IMU was initialized.
     *
     * @return True if the device was successfully initialized, false otherwise.
     */
    bool was_initialized() const;

private:
    /**
     * @brief Subscriber para os dados do IMU.
     */
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscriber;

    /**
     * @brief Dados do IMU recebidos.
     */
    sensor_msgs::msg::Imu data;

    /**
     * @brief Current angular velocity on each axis.
     */
    std::array<float, 3> angular_velocity{};

    /**
     * @brief Current linear acceleration on each axis.
     */
    std::array<float, 3> linear_acceleration{};
};
}  // namespace micras::proxy

#endif  // MICRAS_PROXY_IMU_HPP

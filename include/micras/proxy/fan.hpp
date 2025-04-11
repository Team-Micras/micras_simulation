/**
 * @file
 */

#ifndef MICRAS_PROXY_FAN_HPP
#define MICRAS_PROXY_FAN_HPP

#include <cstdint>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>

#include "micras/proxy/stopwatch.hpp"

namespace micras::proxy {
/**
 * @brief Class for controlling the fan driver.
 */
class Fan {
public:
    /**
     * @brief Configuration struct for the fan.
     */
    struct Config {
        std::shared_ptr<rclcpp::Node>& node;
        std::string                    topic;
    };

    /**
     * @brief Construct a new fan object.
     *
     * @param config Configuration for the fan driver.
     */
    explicit Fan(const Config& config);

    /**
     * @brief Enable the fan.
     */
    void enable();

    /**
     * @brief Disable the fan.
     */
    void disable();

    /**
     * @brief Set the speed of the fans.
     *
     * @param speed Speed percentage of the fan.
     */
    void set_speed(float speed);

    /**
     * @brief Update the speed of the fan.
     *
     * @return Current speed value
     */
    float update();

    /**
     * @brief Stop the fan.
     */
    void stop();

private:
    /**
     * @brief Publisher para controlar a velocidade do ventilador.
     */
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher;

    /**
     * @brief Mensagem contendo a velocidade do ventilador.
     */
    std_msgs::msg::Float32 message;

    /**
     * @brief Flag para verificar se o ventilador está habilitado.
     */
    bool enabled{false};
};
}  // namespace micras::proxy

#endif  // MICRAS_PROXY_FAN_HPP

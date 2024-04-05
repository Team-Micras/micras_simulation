/**
 * @file buzzer.hpp
 *
 * @brief Proxy Buzzer class declaration
 *
 * @date 03/2024
 */

#ifndef MICRAS_PROXY_BUZZER_HPP
#define MICRAS_PROXY_BUZZER_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <string>
#include <cstdint>

namespace proxy {
/**
 * @brief Class for controlling a buzzer
 */
class Buzzer {
    public:
        /**
         * @brief Configuration structure for the buzzer
         */
        struct Config {
            std::shared_ptr<rclcpp::Node>& node;
            std::string                    topic;
        };

        /**
         * @brief Constructor for the Buzzer class
         *
         * @param config Configuration for the buzzer
         */
        explicit Buzzer(const Config& config);

        /**
         * @brief Play a tone for a duration
         *
         * @param frequency Buzzer sound frequency in Hz
         * @param duration Duration of the sound in ms
         */
        void play(uint32_t frequency, uint32_t duration = 0);

        /**
         * @brief Update the buzzer state
         */
        void update();

        /**
         * @brief Stop the buzzer sound
         */
        void stop();

    private:
        /**
         * @brief Publisher for the buzzer topic
         */
        rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr publisher;

        /**
         * @brief Message for the buzzer topic
         */
        std_msgs::msg::UInt32 message;

        /**
         * @brief Node for the buzzer
         */
        std::shared_ptr<rclcpp::Node>& node;

        /**
         * @brief Flag to check if the buzzer is playing
         */
        bool is_playing{false};

        /**
         * @brief Duration of the sound
         */
        uint32_t duration{0};

        /**
         * @brief Timer for the buzzer
         */
        rclcpp::Time playing_timer;
};
}  // namespace proxy

#endif // MICRAS_PROXY_BUZZER_HPP

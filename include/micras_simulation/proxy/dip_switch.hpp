/**
 * @file dip_switch.hpp
 *
 * @brief Proxy Dip Switch class header
 *
 * @date 04/2024
 */

#ifndef MICRAS_PROXY_DIP_SWITCH_HPP
#define MICRAS_PROXY_DIP_SWITCH_HPP

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <array>
#include <cstdint>

namespace proxy {
/**
 * @brief Class for controlling a dip switch
 */
template <uint8_t num_of_switches>
class DipSwitch {
    public:
        /**
         * @brief Configuration struct for DipSwitch
         */
        struct Config {
            std::shared_ptr<rclcpp::Node>&           node;
            std::array<std::string, num_of_switches> topic_array;
        };

        /**
         * @brief Construct a new Dip Switch object
         *
         * @param config Configuration struct for DipSwitch
         */
        explicit DipSwitch(const Config& config);

        /**
         * @brief Get the state of a switch
         *
         * @param switch_index Index of the switch
         * @return bool True if the switch is on, false otherwise
         */
        bool get_switch_state(uint8_t switch_index) const;

        /**
         * @brief Get the value of all switches
         *
         * @return uint8_t Value of all switches
         */
        uint8_t get_switches_value() const;

    private:
        /**
         * @brief Switches states array
         */
        std::array<std_msgs::msg::Bool, num_of_switches> states;

        /**
         * @brief Subscribers for the switches states
         */
        std::array<rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr, num_of_switches> subscribers;
};
}  // namespace proxy

#include "../src/proxy/dip_switch.cpp"  // NOLINT(bugprone-suspicious-include)

#endif // MICRAS_PROXY_DIP_SWITCH_HPP

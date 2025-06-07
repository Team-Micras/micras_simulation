/**
 * @file
 */

#ifndef MICRAS_PROXY_DIP_SWITCH_HPP
#define MICRAS_PROXY_DIP_SWITCH_HPP

#include <array>
#include <cstdint>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

namespace micras::proxy {
/**
 * @brief Class for acquiring dip switch data.
 */
template <uint8_t num_of_switches>
class TDipSwitch {
public:
    /**
     * @brief Configuration struct for the Dip Switch.
     */
    struct Config {
        std::shared_ptr<rclcpp::Node>&           node;
        std::array<std::string, num_of_switches> topic_array;
    };

    /**
     * @brief Construct a new Dip Switch object.
     *
     * @param config Configuration struct for the DipSwitch.
     */
    explicit TDipSwitch(const Config& config);

    /**
     * @brief Get the state of a switch.
     *
     * @param switch_index Index of the switch.
     * @return True if the switch is on, false otherwise.
     */
    bool get_switch_state(uint8_t switch_index) const;

    /**
     * @brief Get the value of all switches.
     *
     * @return Value of all switches.
     */
    uint8_t get_switches_value() const;

private:
    /**
     * @brief Array of subscribers para os switches.
     */
    std::array<rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr, num_of_switches> subscribers;

    /**
     * @brief Array de estados dos switches.
     */
    std::array<std_msgs::msg::Bool, num_of_switches> states;
};
}  // namespace micras::proxy

#include "../src/proxy/dip_switch.cpp"  // NOLINT(bugprone-suspicious-include, misc-header-include-cycle)

#endif  // MICRAS_PROXY_DIP_SWITCH_HPP

/**
 * @file argb.hpp
 *
 * @brief Proxy Argb class declaration
 *
 * @date 03/2024
 */

#ifndef MICRAS_PROXY_ARGB_HPP
#define MICRAS_PROXY_ARGB_HPP

#include <array>
#include <cstdint>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <string>

namespace micras::proxy {
/**
 * @brief Class for controlling an addressable RGB LED
 */
template <uint8_t num_of_leds>
class Argb {
public:
    /**
     * @brief Configuration structure for the addressable RGB LED
     */
    struct Config {
        std::shared_ptr<rclcpp::Node>&       node;
        std::array<std::string, num_of_leds> topic_array;
    };

    /**
     * @brief Structure for storing color information
     */
    struct Color {
        uint8_t red;
        uint8_t green;
        uint8_t blue;
    };

    /**
     * @brief Constructor for the Argb class
     *
     * @param config Configuration for the addressable RGB LED
     */
    explicit Argb(const Config& config);

    /**
     * @brief Set the color of the ARGB at the specified index
     *
     * @param index The index of the ARGB to set the color of
     * @param color The color to set the ARGB to
     */
    void set_color(const Color& color, uint8_t index);

    /**
     * @brief Set the color of all ARGBs
     *
     * @param color The color to set all ARGBs to
     */
    void set_color(const Color& color);

    /**
     * @brief Turn off the ARGB at the specified index
     *
     * @param index The index of the ARGB to turn off
     */
    void turn_off(uint8_t index);

    /**
     * @brief Turn off all ARGBs
     */
    void turn_off();

private:
    /**
     * @brief Map a color from 0-255 to 0-1
     *
     * @param color The color to map
     * @return The mapped color
     */
    float map_color(uint8_t color);

    /**
     * @brief Array of publishers for the ARGBs
     */
    std::array<std::shared_ptr<rclcpp::Publisher<std_msgs::msg::ColorRGBA>>, num_of_leds> publishers;
};
}  // namespace micras::proxy

#include "../src/proxy/argb.cpp"  // NOLINT(bugprone-suspicious-include)

#endif  // MICRAS_PROXY_ARGB_HPP

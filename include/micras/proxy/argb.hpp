/**
 * @file
 */

#ifndef MICRAS_PROXY_ARGB_HPP
#define MICRAS_PROXY_ARGB_HPP

#include <array>
#include <cstdint>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/color_rgba.hpp>

namespace micras::proxy {
/**
 * @brief Class for controlling an addressable RGB LED.
 */
template <uint8_t num_of_leds>
class TArgb {
public:
    /**
     * @brief Configuration struct for the addressable RGB LED.
     */
    struct Config {
        // NOLINTNEXTLINE(cppcoreguidelines-avoid-const-or-ref-data-members)
        std::shared_ptr<rclcpp::Node>&       node;
        std::array<std::string, num_of_leds> topic_array;
    };

    /**
     * @brief Struct for storing color information.
     */
    struct Color {
        uint8_t red;
        uint8_t green;
        uint8_t blue;

        Color operator*(float brightness) const {
            return {
                static_cast<uint8_t>(this->red * brightness),
                static_cast<uint8_t>(this->green * brightness),
                static_cast<uint8_t>(this->blue * brightness),
            };
        }
    };

    /**
     * @brief Predefined colors.
     */
    struct Colors {
        Colors() = delete;

        static constexpr Color red{255, 0, 0};
        static constexpr Color green{0, 255, 0};
        static constexpr Color blue{0, 0, 255};
        static constexpr Color yellow{255, 255, 0};
        static constexpr Color cyan{0, 255, 255};
        static constexpr Color magenta{255, 0, 255};
        static constexpr Color white{255, 255, 255};
    };

    /**
     * @brief Construct a new Argb object.
     *
     * @param config Configuration for the addressable RGB LED.
     */
    explicit TArgb(const Config& config);

    /**
     * @brief Set the color of the ARGB at the specified index.
     *
     * @param index The index of the ARGB to set the color of.
     * @param color The color to set the ARGB to.
     */
    void set_color(const Color& color, uint8_t index);

    /**
     * @brief Set the color of all ARGBs.
     *
     * @param color The color to set all ARGBs to.
     */
    void set_color(const Color& color);

    /**
     * @brief Set the colors of all ARGBs.
     *
     * @param colors The colors to set the ARGBs to.
     */
    void set_colors(const std::array<Color, num_of_leds>& colors);

    /**
     * @brief Turn off the ARGB at the specified index.
     *
     * @param index The index of the ARGB to turn off.
     */
    void turn_off(uint8_t index);

    /**
     * @brief Turn off all ARGBs.
     */
    void turn_off();

    /**
     * @brief Send the colors to the addressable RGB LED.
     *
     * This function is called automatically when the colors are set.
     */
    void update();

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
    std::array<rclcpp::Publisher<std_msgs::msg::ColorRGBA>::SharedPtr, num_of_leds> publishers;
};
}  // namespace micras::proxy

#include "../src/proxy/argb.cpp"  // NOLINT(bugprone-suspicious-include, misc-header-include-cycle)

#endif  // MICRAS_PROXY_ARGB_HPP

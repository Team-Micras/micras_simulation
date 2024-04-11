/**
 * @file timer.hpp
 *
 * @brief Timer HAL mock
 *
 * @date 04/2024
 */

#ifndef MICRAS_HAL_TIMER_HPP
#define MICRAS_HAL_TIMER_HPP

#include <cstdint>
#include <rclcpp/rclcpp.hpp>

namespace micras::hal {
/**
 * @brief Class to handle timer peripheral on STM32 microcontrollers
 */
class Timer {
public:
    /**
     * @brief Construct a new Timer object
     */
    Timer() = default;

    /**
     * @brief Sleep for a given amount of time
     *
     * @param time Time to sleep in milliseconds
     */
    static void sleep_ms(uint32_t time);
};
}  // namespace micras::hal

#endif  // MICRAS_HAL_TIMER_HPP

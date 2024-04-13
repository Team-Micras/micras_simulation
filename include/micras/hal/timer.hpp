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
 * @brief Class to handle a ros based timer
 */
class Timer {
public:
    /**
     * @brief Timer configuration struct
     */
    struct Config { };

    /**
     * @brief Construct a new Timer object
     */
    Timer();

    /**
     * @brief Construct a new Timer object
     *
     * @param config Configuration for the timer
     */
    explicit Timer(const Config& config);

    /**
     * @brief Reset the timer counter in milliseconds
     */
    void reset_ms();

    /**
     * @brief Reset the timer counter in microseconds
     */
    void reset_us();

    /**
     * @brief Get the time elapsed since the last reset
     *
     * @return uint32_t Time elapsed in miliseconds
     */
    uint32_t elapsed_time_ms() const;

    /**
     * @brief Get the time elapsed since the last reset
     *
     * @return uint32_t Time elapsed in microseconds
     */
    uint32_t elapsed_time_us() const;

    /**
     * @brief Sleep for a given amount of time
     *
     * @param time Time to sleep in milliseconds
     */
    static void sleep_ms(uint32_t time);

    /**
     * @brief Sleep for a given amount of time
     *
     * @param time Time to sleep in microseconds
     */
    void sleep_us(uint32_t time) const;

private:
    /**
     * @brief Get the current timer counter
     *
     * @return uint32_t Current timer counter in milliseconds
     */
    static uint32_t get_counter_ms();

    /**
     * @brief Get the current timer counter
     *
     * @return uint32_t Current timer counter in microseconds
     */
    uint32_t get_counter_us() const;

    /**
     * @brief Timer counter
     */
    uint32_t counter{};
};
}  // namespace micras::hal

#endif  // MICRAS_HAL_TIMER_HPP

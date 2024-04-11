/**
 * @file timer.cpp
 *
 * @brief Timer HAL mock
 *
 * @date 04/2024
 */

#include "micras/hal/timer.hpp"

namespace micras::hal {
Timer::Timer(const Config& /*config*/) { }

void Timer::reset_ms() {
    this->counter = get_counter_ms();
}

void Timer::reset_us() {
    this->counter = this->get_counter_us();
}

uint32_t Timer::elapsed_time_ms() const {
    return get_counter_ms() - this->counter;
}

uint32_t Timer::elapsed_time_us() const {
    return this->get_counter_us() - this->counter;
}

void Timer::sleep_ms(uint32_t time) {
    rclcpp::sleep_for(std::chrono::milliseconds(time));
}

void Timer::sleep_us(uint32_t time) const {
    rclcpp::sleep_for(std::chrono::microseconds(time));
}

uint32_t Timer::get_counter_ms() {
    return rclcpp::Clock().now().nanoseconds() * 1e-6;
}

uint32_t Timer::get_counter_us() const {
    return rclcpp::Clock().now().nanoseconds() * 1e-3;
}
}  // namespace micras::hal

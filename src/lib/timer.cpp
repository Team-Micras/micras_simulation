/**
 * @file timer.cpp
 *
 * @brief Timer HAL mock
 *
 * @date 04/2024
 */

#include "micras/hal/timer.hpp"

namespace micras::hal {

void Timer::sleep_ms(uint32_t time) {
    rclcpp::sleep_for(std::chrono::milliseconds(time));
}
}  // namespace micras::hal

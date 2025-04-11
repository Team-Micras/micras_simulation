/**
 * @file
 */

#include "micras/proxy/stopwatch.hpp"

#include <rclcpp/rclcpp.hpp>
#include <limits>

namespace micras::proxy {
Stopwatch::Stopwatch() {
    this->reset_ms();
}

Stopwatch::Stopwatch(const Config& /*config*/) {
    this->reset_us();
}

void Stopwatch::reset_ms() {
    this->counter = this->get_counter_ms();
}

void Stopwatch::reset_us() {
    this->counter = this->get_counter_us();
}

uint32_t Stopwatch::elapsed_time_ms() const {
    return this->get_counter_ms() - this->counter;
}

uint32_t Stopwatch::elapsed_time_us() const {
    const uint32_t current = this->get_counter_us();

    if (current < this->counter) {
        return std::numeric_limits<uint16_t>::max() - this->counter + current;
    }

    return current - this->counter;
}

void Stopwatch::sleep_ms(uint32_t time) {
    rclcpp::sleep_for(std::chrono::milliseconds(time));
}

void Stopwatch::sleep_us(uint32_t time) const {
    rclcpp::sleep_for(std::chrono::microseconds(time));
}

uint32_t Stopwatch::get_counter_ms() const {
    return static_cast<uint32_t>(rclcpp::Clock().now().nanoseconds() / 1000000);
}

uint32_t Stopwatch::get_counter_us() const {
    return static_cast<uint32_t>(rclcpp::Clock().now().nanoseconds() / 1000);
}
}  // namespace micras::proxy

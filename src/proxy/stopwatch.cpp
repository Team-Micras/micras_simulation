/**
 * @file
 */

#include "micras/proxy/stopwatch.hpp"
#include "target.hpp"
#include <rclcpp/rclcpp.hpp>
#include <limits>

namespace micras::proxy {

Stopwatch::Stopwatch() {
    this->clock_subscriber = micras_node->create_subscription<builtin_interfaces::msg::Time>(
        "clock", 1, [](const builtin_interfaces::msg::Time& /*msg*/) {}
    );
    clock_msg.sec = 0;
    clock_msg.nanosec = 0;
    this->reset_ms();
}

Stopwatch::Stopwatch(const Config& /*config*/) : Stopwatch() {
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
    // rclcpp::sleep_for(std::chrono::milliseconds(time));
}

void Stopwatch::sleep_us(uint32_t time) const {
    // rclcpp::sleep_for(std::chrono::microseconds(time));
}

uint32_t Stopwatch::get_counter_ms() const {
    builtin_interfaces::msg::Time msg;
    rclcpp::MessageInfo           info;
    if (this->clock_subscriber->take(msg, info)) {
        clock_msg = msg;
    }
    return static_cast<uint32_t>(clock_msg.sec * 1000 + clock_msg.nanosec / 1000000);
}

uint32_t Stopwatch::get_counter_us() const {
    builtin_interfaces::msg::Time msg;
    rclcpp::MessageInfo           info;
    if (this->clock_subscriber->take(msg, info)) {
        clock_msg = msg;
    }
    return static_cast<uint32_t>(clock_msg.sec * 1000000 + clock_msg.nanosec / 1000);
}
}  // namespace micras::proxy

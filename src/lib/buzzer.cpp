/**
 * @file buzzer.cpp
 *
 * @brief Proxy Buzzer class implementation
 *
 * @date 03/2024
 */

#include "micras/proxy/buzzer.hpp"

namespace micras::proxy {
Buzzer::Buzzer(const Config& config) {
    this->publisher = config.node->create_publisher<std_msgs::msg::UInt32>(config.topic, 1);
    this->message = std_msgs::msg::UInt32();
    this->stop();
}

void Buzzer::play(uint32_t frequency, uint32_t duration) {
    this->message.data = frequency;
    this->publisher->publish(message);
    this->is_playing = true;
    this->duration = duration;

    if (duration > 0) {
        this->playing_timer.reset_ms();
    }
}

void Buzzer::update() {
    if (this->is_playing and this->duration > 0 and this->playing_timer.elapsed_time_ms() > this->duration) {
        this->stop();
    }
}

void Buzzer::wait(uint32_t duration) {
    while (this->duration > 0 and this->is_playing) {
        this->update();
    }

    hal::Timer wait_timer;

    while (wait_timer.elapsed_time_ms() < duration) {
        this->update();
    }
}

void Buzzer::stop() {
    this->message.data = 0;
    this->publisher->publish(message);
    this->is_playing = false;
}
}  // namespace micras::proxy

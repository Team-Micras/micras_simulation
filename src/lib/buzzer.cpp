/**
 * @file buzzer.cpp
 *
 * @brief Proxy Buzzer class implementation
 *
 * @date 03/2024
 */

#include "micras/proxy/buzzer.hpp"

namespace micras::proxy {
Buzzer::Buzzer(const Config& config) : node{config.node} {
    this->publisher = config.node->create_publisher<std_msgs::msg::UInt32>(config.topic, 1);
    this->message = std_msgs::msg::UInt32();
    this->stop();
}

void Buzzer::play(uint32_t frequency, uint32_t duration) {
    this->message.data = frequency;
    this->publisher->publish(message);
    this->is_playing = true;

    if (duration > 0) {
        this->duration = duration;
        this->playing_timer = this->node->now();
    }
}

void Buzzer::update() {
    if (not this->is_playing) {
        return;
    }

    auto elapsed_time = ((this->node->now() - this->playing_timer).nanoseconds()) * 1e-6;

    if (this->duration > 0 and elapsed_time > this->duration) {
        this->stop();
    }
}

void Buzzer::stop() {
    this->message.data = 0;
    this->publisher->publish(message);
    this->is_playing = false;
}
}  // namespace proxy

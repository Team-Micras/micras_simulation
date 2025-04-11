/**
 * @file
 */

#include "micras/proxy/fan.hpp"

namespace micras::proxy {
Fan::Fan(const Config& config) {
    this->publisher = config.node->create_publisher<std_msgs::msg::Float32>(config.topic, 1);
    this->stop();
    this->enable();
}

void Fan::enable() {
    this->enabled = true;
    this->publisher->publish(this->message);
}

void Fan::disable() {
    this->enabled = false;

    std_msgs::msg::Float32 stopped_message{};
    stopped_message.data = 0.0f;

    this->publisher->publish(stopped_message);
}

void Fan::set_speed(float speed) {
    this->message.data = speed;

    if (this->enabled) {
        this->publisher->publish(this->message);
    }
}

float Fan::update() {
    // Em um ambiente de simulação, apenas retornamos o valor atual configurado
    return this->message.data;
}

void Fan::stop() {
    this->set_speed(0.0f);
}
}  // namespace micras::proxy

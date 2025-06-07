/**
 * @file
 */

#include "micras/proxy/battery.hpp"

namespace micras::proxy {
Battery::Battery(const Config& config) : max_voltage{config.max_voltage}, max_reading{config.max_reading} {
    this->subscriber = config.node->create_subscription<std_msgs::msg::Float32>(
        config.topic, 1, [this](const std_msgs::msg::Float32& msg) { this->reading = msg.data; }
    );
}

void Battery::update() {
    // A atualização é feita automaticamente pelo callback do subscriber
}

float Battery::get_voltage() const {
    return this->reading;
}

float Battery::get_voltage_raw() const {
    return (this->reading / this->max_voltage) * this->max_reading;
}

float Battery::get_adc_reading() const {
    return this->reading / this->max_voltage;
}
}  // namespace micras::proxy

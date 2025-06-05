/**
 * @file
 */

#include <cmath>

#include "micras/core/utils.hpp"
#include "micras/proxy/motor.hpp"

namespace micras::proxy {
Motor::Motor(const Config& config) {
    this->publisher = config.node->create_publisher<example_interfaces::msg::Float64>(config.topic, 1);
    this->set_command(0.0F);
}

void Motor::set_command(float command) {
    this->command.data = static_cast<double>(std::clamp(command, -100.0F, 100.0F));
    this->publisher->publish(this->command);
}
}  // namespace micras::proxy

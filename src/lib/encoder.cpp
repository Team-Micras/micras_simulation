/**
 * @file encoder.cpp
 *
 * @brief Proxy Encoder class source
 *
 * @date 03/2024
 */

#include "proxy/encoder.hpp"

namespace proxy {
Encoder::Encoder(const Config& encoder_config) {
    this->subscriber = encoder_config.node->create_subscription<sensor_msgs::msg::JointState>(
        encoder_config.topic, 1, [this](const sensor_msgs::msg::JointState& msg) {
            this->data = msg;
        });
}

double Encoder::get_position() const {
    return this->data.position[0];
}

double Encoder::get_velocity() const {
    return this->data.velocity[0];
}
}  // namespace proxy

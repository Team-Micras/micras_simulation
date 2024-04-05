/**
 * @file fan.cpp
 *
 * @brief Proxy Fan class source
 *
 * @date 03/2024
 */

#include "proxy/fan.hpp"

namespace proxy {
Fan::Fan(const Config& config) {
    (void) config;
    this->stop();
    this->enable();
}

void Fan::enable() {
    this->enabled = true;
}

void Fan::disable() {
    this->enabled = false;
}

void Fan::set_speed(float speed) {
    (void) speed;
}

void Fan::stop() {
}

void Fan::set_direction(RotationDirection direction) {
    (void) direction;
}
}  // namespace proxy

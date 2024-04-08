/**
 * @file button.cpp
 *
 * @brief Proxy Button class source
 *
 * @date 03/2024
 */

#include "micras/proxy/button.hpp"

namespace micras::proxy {
Button::Button(const Config& config) :
    long_press_delay{config.long_press_delay},
    extra_long_press_delay{config.extra_long_press_delay},
    node{config.node} {
    this->subscriber =
        config.node->create_subscription<std_msgs::msg::Bool>(config.topic, 1, [this](const std_msgs::msg::Bool& msg) {
            this->state.data = msg.data;
        });
}

bool Button::is_pressed() {
    this->update_state();
    return this->current_state;
}

Button::Status Button::get_status() {
    this->update_state();

    if (this->is_rising_edge()) {
        this->press_time = this->node->now();
    } else if (this->is_falling_edge()) {
        auto elapsed_time = ((this->node->now() - this->press_time).nanoseconds()) * 1e-6;

        if (elapsed_time > this->extra_long_press_delay) {
            return EXTRA_LONG_PRESS;
        } else if (elapsed_time > this->long_press_delay) {
            return LONG_PRESS;
        } else {
            return SHORT_PRESS;
        }
    }

    return NO_PRESS;
}

void Button::update_state() {
    this->previous_state = this->current_state;
    this->current_state = this->state.data;
}

bool Button::is_rising_edge() const {
    return this->current_state and not this->previous_state;
}

bool Button::is_falling_edge() const {
    return not this->current_state and this->previous_state;
}
}  // namespace micras::proxy

/**
 * @file
 */

#include "micras/proxy/button.hpp"

namespace micras::proxy {
Button::Button(const Config& config) :
    long_press_delay{config.long_press_delay}, extra_long_press_delay{config.extra_long_press_delay} {
    this->subscriber =
        config.node->create_subscription<std_msgs::msg::Bool>(config.topic, 1, [this](const std_msgs::msg::Bool& msg) {
            this->state.data = msg.data;
        });
}

bool Button::is_pressed() const {
    return this->state.data;
}

Button::Status Button::get_status() const {
    return this->current_status;
}

void Button::update() {
    this->previous_state = this->current_state;
    this->update_state();

    if (this->is_rising_edge()) {
        this->status_stopwatch.reset_ms();
    } else if (this->is_falling_edge()) {
        if (this->status_stopwatch.elapsed_time_ms() > extra_long_press_delay) {
            this->current_status = EXTRA_LONG_PRESS;
            return;
        }

        if (this->status_stopwatch.elapsed_time_ms() > long_press_delay) {
            this->current_status = LONG_PRESS;
            return;
        }

        this->current_status = SHORT_PRESS;
        return;
    }

    this->current_status = NO_PRESS;
}

void Button::update_state() {
    this->current_state = this->state.data;
}

bool Button::is_rising_edge() const {
    return this->current_state and not this->previous_state;
}

bool Button::is_falling_edge() const {
    return !this->current_state and this->previous_state;
}
}  // namespace micras::proxy

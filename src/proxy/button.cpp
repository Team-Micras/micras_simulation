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

Button::Status Button::get_status() {
    this->update_state();

    if (this->is_rising_edge()) {
        this->status_timer.reset_ms();
    } else if (this->is_falling_edge()) {
        if (this->status_timer.elapsed_time_ms() > this->extra_long_press_delay) {
            return EXTRA_LONG_PRESS;
        }

        if (this->status_timer.elapsed_time_ms() > this->long_press_delay) {
            return LONG_PRESS;
        }

        return SHORT_PRESS;
    }

    return NO_PRESS;
}

void Button::update_state() {
    this->previous_state = this->current_state;
    this->current_state = this->state.data;
}

bool Button::is_rising_edge() const {
    return this->current_state && !this->previous_state;
}

bool Button::is_falling_edge() const {
    return !this->current_state && this->previous_state;
}
}  // namespace micras::proxy

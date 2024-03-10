#ifndef __TARGET_HPP__
#define __TARGET_HPP__

#include "proxy/button.hpp"
#include "proxy/led.hpp"

proxy::Led::Config led_config{
    "led_node", // name
    "led",      // topic
};

proxy::Button::Config button_config{
    "button_node", // name
    "button",      // topic
};

#endif // __TARGET_HPP__

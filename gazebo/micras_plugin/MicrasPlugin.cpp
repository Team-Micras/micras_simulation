#include <gz/msgs.hh>
#include <ignition/plugin/Register.hh>
#include <iostream>

#include "MicrasPlugin.hpp"

namespace ignition::gui {
MicrasPlugin::MicrasPlugin() {
    this->title = "Micras Plugin";
    this->set_publishers();
    this->set_led_subscriber();
    this->set_rgb_0_subscriber();
    this->set_rgb_1_subscriber();
    this->set_buzzer_subscriber();
    this->set_fan_subscriber();
}

void MicrasPlugin::LoadConfig(const tinyxml2::XMLElement* /*_pluginElem*/) { }

void MicrasPlugin::on_button_click() {
    std::cout << "Button clicked" << std::endl;
    gz::msgs::Boolean msg;
    msg.set_data(true);
    this->button_pub.Publish(msg);
}

void MicrasPlugin::on_button_release() {
    std::cout << "Button released" << std::endl;
    gz::msgs::Boolean msg;
    msg.set_data(false);
    this->button_pub.Publish(msg);
}

void MicrasPlugin::on_slider_change(float _value) {
    std::cout << "Battery value: " << _value << std::endl;
    gz::msgs::Float msg;
    msg.set_data(_value);
    this->battery_pub.Publish(msg);
}

void MicrasPlugin::set_switch_state(int _index, bool _state) {
    std::cout << "Switch " << _index << " state: " << _state << std::endl;
    gz::msgs::Boolean msg;
    msg.set_data(_state);
    this->switch_pub.at(_index).Publish(msg);
}

void MicrasPlugin::set_publishers() {
    this->button_pub = this->node.Advertise<gz::msgs::Boolean>(this->button_topic);
    this->battery_pub = this->node.Advertise<gz::msgs::Float>(this->battery_topic);
    for (uint32_t i = 0; i < this->switch_pub.size(); i++) {
        this->switch_pub.at(i) = this->node.Advertise<gz::msgs::Boolean>(this->dip_switch_topic + std::to_string(i));
    }
}

void MicrasPlugin::set_led_subscriber() {
    std::function<void(const gz::msgs::Boolean&)> led_cb = [this](const gz::msgs::Boolean& msg) {
        emit this->led_state_changed(msg.data());
    };
    this->node.Subscribe(this->led_topic, led_cb);
}

void MicrasPlugin::set_rgb_0_subscriber() {
    std::function<void(const gz::msgs::Color&)> led_rgb_0_cb = [this](const gz::msgs::Color& msg) {
        emit this->led_rgb_0_changed(msg.r(), msg.g(), msg.b());
    };
    this->node.Subscribe(this->led_rgb_0_topic, led_rgb_0_cb);
}

void MicrasPlugin::set_rgb_1_subscriber() {
    std::function<void(const gz::msgs::Color&)> led_rgb_1_cb = [this](const gz::msgs::Color& msg) {
        emit this->led_rgb_1_changed(msg.r(), msg.g(), msg.b());
    };
    this->node.Subscribe(this->led_rgb_1_topic, led_rgb_1_cb);
}

void MicrasPlugin::set_buzzer_subscriber() {
    std::function<void(const gz::msgs::UInt32&)> buzzer_cb = [this](const gz::msgs::UInt32& msg) {
        emit this->buzzer_changed(msg.data());
    };
    this->node.Subscribe(this->buzzer_topic, buzzer_cb);
}

void MicrasPlugin::set_fan_subscriber() {
    std::function<void(const gz::msgs::Float&)> fan_cb = [this](const gz::msgs::Float& msg) {
        emit this->fan_speed_changed(msg.data());
    };
    this->node.Subscribe(this->fan_topic, fan_cb);
}
}  // namespace ignition::gui

/**
 * @brief Register the MicrasPlugin plugin
 */
IGNITION_ADD_PLUGIN(MicrasPlugin, Plugin);

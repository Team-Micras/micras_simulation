/*
 * Copyright (C) 2017 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include <iostream>
#include <ignition/plugin/Register.hh>
#include <gz/msgs.hh>

#include "MicrasPlugin.hpp"

using namespace ignition;
using namespace gui;

MicrasPlugin::MicrasPlugin() : Plugin() {
    this->button_pub = this->node.Advertise<gz::msgs::Boolean>("/button");
    this->battery_pub = this->node.Advertise<gz::msgs::Float>("/battery");
    for (int i = 0; i < this->switch_pub.size(); ++i) {
        this->switch_pub[i] = this->node.Advertise<gz::msgs::Boolean>("/dip_switch_" + std::to_string(i));
    }

    std::function<void(const gz::msgs::Boolean&)> led_cb = [this](const gz::msgs::Boolean& msg) {
        emit this->led_state_changed(msg.data());
    };
    this->node.Subscribe("/led", led_cb);

    std::function<void(const gz::msgs::Color&)> led_rgb_0_cb = [this](const gz::msgs::Color& msg) {
        emit this->led_rgb_0_changed(msg.r(), msg.g(), msg.b());
    };
    this->node.Subscribe("/rgb_0", led_rgb_0_cb);

    std::function<void(const gz::msgs::Color&)> led_rgb_1_cb = [this](const gz::msgs::Color& msg) {
        emit this->led_rgb_1_changed(msg.r(), msg.g(), msg.b());
    };
    this->node.Subscribe("/rgb_1", led_rgb_1_cb);

    std::function<void(const gz::msgs::UInt32&)> buzzer_cb = [this](const gz::msgs::UInt32& msg) {
        emit this->buzzer_changed(msg.data());
    };
    this->node.Subscribe("/buzzer", buzzer_cb);
}

MicrasPlugin::~MicrasPlugin() { }

void MicrasPlugin::LoadConfig(const tinyxml2::XMLElement* _pluginElem) {
    if (!_pluginElem)
        return;

    // Take parameters from XML at runtime
    auto messageElem = _pluginElem->FirstChildElement("message");
    if (nullptr != messageElem && nullptr != messageElem->GetText())
        this->message = messageElem->GetText();
}

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
    this->switch_pub[_index].Publish(msg);
}

// Register this plugin
IGNITION_ADD_PLUGIN(ignition::gui::MicrasPlugin, ignition::gui::Plugin);

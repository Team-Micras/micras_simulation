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

#ifndef IGNITION_GUI_HELLOPLUGIN_HH_
#define IGNITION_GUI_HELLOPLUGIN_HH_

#include <string>
#include <array>

#include <gz/transport.hh>

#include <ignition/gui/qt.h>
#include <ignition/gui/Plugin.hh>

namespace ignition {
namespace gui {
class MicrasPlugin : public Plugin {
    Q_OBJECT

public:
    MicrasPlugin();

public:
    virtual ~MicrasPlugin();

public:
    virtual void LoadConfig(const tinyxml2::XMLElement* _pluginElem) override;

signals:
    void led_state_changed(bool new_state);

    void led_rgb_0_changed(float r, float g, float b);

    void led_rgb_1_changed(float r, float g, float b);

protected slots:
    void on_button_click();

    void on_button_release();

    void on_slider_change(float _value);

    void set_switch_state(int _index, bool _state);

private:
    std::string message{"Micras, plugin!"};

    gz::transport::Node node;

    std::array<ignition::transport::v11::Node::Publisher, 4> switch_pub;

    ignition::transport::v11::Node::Publisher button_pub;

    ignition::transport::v11::Node::Publisher battery_pub;

    gz::msgs::Boolean led_state;
};
}  // namespace gui
}  // namespace ignition

#endif

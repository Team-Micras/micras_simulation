/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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
import QtQuick 2.0
import QtQuick.Controls 2.0
import QtQuick.Layouts 1.3

ColumnLayout {

    Layout.minimumWidth: 400
    Layout.minimumHeight: 400
    Layout.margins: 5
    anchors.fill: parent
    focus: true

    Column {
        anchors.centerIn: parent
        spacing: 20

        Button {
            text: "Button"
            width: 150
            height: 50

            onPressedChanged: {
                if (pressed) {
                    MicrasPlugin.on_button_click();
                } else {
                    MicrasPlugin.on_button_release();
                }
            }
        }

        Row {
            spacing: 20
            Switch {
                id: switch0
                onToggled: {
                    MicrasPlugin.set_switch_state(0, switch0.checked);
                }
            }
            Switch {
                id: switch1
                onToggled: {
                    MicrasPlugin.set_switch_state(1, switch1.checked);
                }
            }
            Switch {
                id: switch2
                onToggled: {
                    MicrasPlugin.set_switch_state(2, switch2.checked);
                }
            }
            Switch {
                id: switch3
                onToggled: {
                    MicrasPlugin.set_switch_state(3, switch3.checked);
                }
            }
        }

        Row {
            spacing: 20
            Rectangle {
                width: 50
                height: 50
                radius: 180
                border.color: "black"
                border.width: 1
                color: "lightgrey"

                Component.onCompleted: {
                    MicrasPlugin.led_state_changed.connect(updateColor);
                }

                function updateColor(newState) {
                    color = newState ? Qt.rgba(1, 0, 0) : "lightgrey";
                }
            }

            Rectangle {
                width: 50
                height: 50
                radius: 180
                border.color: "black"
                border.width: 1
                color: "lightgrey"

                Component.onCompleted: {
                    MicrasPlugin.led_rgb_0_changed.connect(updateColor);
                }

                function updateColor(r, g, b) {
                    color = Qt.rgba(r, g, b);
                }
            }
            Rectangle {
                width: 50
                height: 50
                radius: 180
                border.color: "black"
                border.width: 1
                color: "lightgrey"

                Component.onCompleted: {
                    MicrasPlugin.led_rgb_1_changed.connect(updateColor);
                }

                function updateColor(r, g, b) {
                    color = Qt.rgba(r, g, b);
                }
            }
        }

        Slider {
            id: slider
            width: parent.width
            from: 0.0
            to: 8.4
            stepSize: 0.1
            value: 8.4 // Initial value
            onValueChanged: {
                MicrasPlugin.on_slider_change(value);
            }
        }
    }
}

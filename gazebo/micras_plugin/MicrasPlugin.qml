import QtQuick 2.0
import QtQuick.Controls 2.0
import QtQuick.Layouts 1.3

ColumnLayout {

    Layout.minimumWidth: 400
    Layout.minimumHeight: 370
    Layout.margins: 5
    anchors.fill: parent
    focus: true

    Column {
        anchors.centerIn: parent
        spacing: 25

        Row {
            spacing: 20
            Button {
                text: "Button"
                width: 150
                height: 80

                onPressedChanged: {
                    if (pressed) {
                        MicrasPlugin.on_button_click();
                        MicrasPlugin.play_buzzer(4400, 2);
                    } else {
                        MicrasPlugin.on_button_release();
                    }
                }
            }

            Rectangle {
                id: dipSwitch
                width: 180
                height: 80
                radius: 10
                color: "lightblue"

                RowLayout {
                    spacing: -20

                    anchors.horizontalCenter: dipSwitch.horizontalCenter
                    anchors.verticalCenter: dipSwitch.verticalCenter

                    Column {
                        spacing: 2

                        Text {
                            text: "0"
                            anchors.horizontalCenter: parent.horizontalCenter
                        }
                        Switch {
                            transform: Rotation { origin.x: switch0.width / 2; origin.y: switch0.height / 2; angle: -90}
                            id: switch0
                            onToggled: {
                                MicrasPlugin.set_switch_state(0, switch0.checked);
                            }
                            anchors.horizontalCenter: parent.horizontalCenter
                        }
                    }
                    Column {
                        spacing: 2
                        Text {
                            text: "1"
                            anchors.horizontalCenter: parent.horizontalCenter
                        }
                        Switch {
                            transform: Rotation { origin.x: switch1.width / 2; origin.y: switch1.height / 2; angle: -90}
                            id: switch1
                            onToggled: {
                                MicrasPlugin.set_switch_state(1, switch1.checked);
                            }
                            anchors.horizontalCenter: parent.horizontalCenter
                        }
                    }
                    Column {
                        spacing: 2
                        Text {
                            text: "2"
                            anchors.horizontalCenter: parent.horizontalCenter
                        }
                        Switch {
                            transform: Rotation { origin.x: switch2.width / 2; origin.y: switch2.height / 2; angle: -90}
                            id: switch2
                            onToggled: {
                                MicrasPlugin.set_switch_state(2, switch2.checked);
                            }
                            anchors.horizontalCenter: parent.horizontalCenter
                        }
                    }
                    Column {
                        spacing: 2
                        Text {
                            text: "3"
                            anchors.horizontalCenter: parent.horizontalCenter
                        }
                        Switch {
                            transform: Rotation { origin.x: switch3.width / 2; origin.y: switch3.height / 2; angle: -90}
                            id: switch3
                            onToggled: {
                                MicrasPlugin.set_switch_state(3, switch3.checked);
                            }
                            anchors.horizontalCenter: parent.horizontalCenter
                        }
                    }
                }
            }
        }

        Row {
            spacing: 25

            Column {
                spacing: 5

                Rectangle {
                    width: 50
                    height: 50
                    radius: 180
                    color: "lightgrey"
                    anchors.horizontalCenter: parent.horizontalCenter

                    Component.onCompleted: {
                        MicrasPlugin.led_state_changed.connect(updateColor);
                    }

                    function updateColor(newState) {
                        color = newState ? Qt.rgba(1, 0, 0) : "lightgrey";
                    }
                }

                Text {
                    text: "LED"
                    anchors.horizontalCenter: parent.horizontalCenter
                }
            }

            Column {
                spacing: 5

                Rectangle {
                    width: 50
                    height: 50
                    radius: 180
                    border.color: "lightgrey"
                    border.width: 1
                    color: "lightgrey"
                    anchors.horizontalCenter: parent.horizontalCenter

                    Component.onCompleted: {
                        MicrasPlugin.led_rgb_0_changed.connect(updateColor);
                    }

                    function updateColor(r, g, b) {
                        color = Qt.rgba(r, g, b);
                    }

                }

                Text {
                    text: "ARGB 0"
                    anchors.horizontalCenter: parent.horizontalCenter
                }
            }

            Column {
                spacing: 5
                Rectangle {
                    width: 50
                    height: 50
                    radius: 180
                    border.color: "lightgrey"
                    border.width: 1
                    color: "lightgrey"
                    anchors.horizontalCenter: parent.horizontalCenter

                    Component.onCompleted: {
                        MicrasPlugin.led_rgb_1_changed.connect(updateColor);
                    }

                    function updateColor(r, g, b) {
                        color = Qt.rgba(r, g, b);
                    }
                }

                Text {
                    text: "ARGB 1"
                    anchors.horizontalCenter: parent.horizontalCenter
                }
            }

            Rectangle {
                id: buzzerRect
                width: 120
                height: 70
                radius: 10
                color: "lightgrey"

                Column{
                    spacing: 5
                    anchors.horizontalCenter: buzzerRect.horizontalCenter
                    anchors.verticalCenter: buzzerRect.verticalCenter

                    Text {
                        text: "Buzzer"
                        lineHeight: 1.5
                        anchors.top: parent.top
                        anchors.topMargin: -20
                        anchors.horizontalCenter: parent.horizontalCenter
                    }

                    Text {
                        id: buzzerText
                        text: ""
                        lineHeight: 1.5
                        anchors.horizontalCenter: parent.horizontalCenter
                    }
                }


                Component.onCompleted: {
                    MicrasPlugin.buzzer_changed.connect(updateBuzzerState);
                }

                function updateBuzzerState(freq) {
                    if (freq == 0) {
                        color = "lightgrey";
                        buzzerText.text = "\n";
                    } else {
                        color = Qt.rgba(0, 1, 0);
                        buzzerText.text = freq + "Hz";
                    }
                }
            }
        }

        RowLayout {
            spacing: 10

            Layout.alignment: Qt.AlignHCenter | Qt.AlignVCenter

            Slider {
                id: slider
                from: 0.0
                to: 8.4
                stepSize: 0.1
                value: 8.4 // Initial value
                onValueChanged: {
                    MicrasPlugin.on_slider_change(value);
                }
            }

            Label {
                text: "Battery Voltage: " + slider.value.toFixed(1) + "V"
            }
        }


    }
}

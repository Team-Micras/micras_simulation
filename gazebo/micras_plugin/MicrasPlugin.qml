import QtQuick 2.7
import QtQuick.Controls 2.0
import QtQuick.Layouts 1.3

ColumnLayout {

  Layout.minimumWidth: 400
  Layout.minimumHeight: 430
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
          if (pressed)
          {
            MicrasPlugin.on_button_click();
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

        function updateColor(newState)
        {
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

        function updateColor(r, g, b)
        {
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

        function updateColor(r, g, b)
        {
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

      Column {
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

      function updateBuzzerState(freq)
      {
        if (freq == 0)
        {
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
  spacing: 30

  RowLayout {
    id: fanRow
    spacing: -1

    Rectangle {
      id: rect1
      transform: Rotation { origin.x: rect1.width / 2; origin.y: rect1.height / 2; angle: 180}
      width: 110
      height: 30
      radius: 0
      border.width: 1
      border.color: "white"
      color: "transparent"

      ProgressBar {
        from: 0
        to: 100
        id: pBar
        value: 0
        anchors.fill: parent
        anchors.margins:1
        background: Rectangle {
          anchors.fill:parent
          color: "transparent"
          clip: true

          Rectangle {
            width: parent.width + radius
            height: parent.height
            anchors.right: parent.right
            radius: 6
            color: "lightgray"
          }
        }
        contentItem: Item {
          Rectangle {
            width: pBar.visualPosition * parent.width
            height: parent.height
            color: "transparent"
            clip: true
            Rectangle {
              width: parent.width + radius
              height: parent.height
              anchors.right: parent.right
              radius: 6
              color: "#03a9f4"
            }
          }
        }
      }
      Component.onCompleted: {
        MicrasPlugin.fan_speed_changed.connect(updateFanSpeed);
      }

      function updateFanSpeed(speed)
      {
        if (speed <= 0)
        {
          pBar.value = -speed;
        }
      }
    }

    Rectangle {
      z: 3
      implicitWidth: 2
      implicitHeight: 35
      radius: 6
      color: "grey"
    }


    Rectangle {
      id: rect2
      width: 110
      height: 30
      radius: 0
      border.width: 1
      border.color: "white"
      color: "transparent"

      ProgressBar {
        from: 0
        to: 100
        id: pBar2
        value: 0
        anchors.fill: parent
        anchors.margins:1
        background: Rectangle {
          anchors.fill:parent
          color: "transparent"
          clip: true

          Rectangle {
            width: parent.width + radius
            height: parent.height
            anchors.right: parent.right
            radius: 6
            color: "lightgray"
          }
        }
        contentItem: Item {
          Rectangle {
            width: pBar2.visualPosition * parent.width
            height: parent.height
            color: "transparent"
            clip: true
            Rectangle {
              width: parent.width + radius
              height: parent.height
              anchors.right: parent.right
              radius: 6
              color: "#03a9f4"
            }
          }
        }
      }
      Component.onCompleted: {
        MicrasPlugin.fan_speed_changed.connect(updateFanSpeed);
      }

      function updateFanSpeed(speed)
      {
        if (speed >= 0)
        {
          pBar2.value = speed;
        }
      }
    }
  }

  Rectangle {
    color: "transparent"
    width: 70
    height: 70
    radius: 180
    border.color: "lightgrey"
    border.width: 1
    //color: "lightgrey"
    //anchors.horizontalCenter: parent.horizontalCenter

    Rectangle {
      width: 70
      height: 70
      radius: 1000
      color: "transparent"
      clip: true

      Image {
        id: imagenCentro
        width: 70
        height: 70
        fillMode: Image.PreserveAspectCrop
        source: "/home/santi/Documentos/MicroMouse/micras_ws/src/micras_simulation/gazebo/micras_plugin/assets/fan.png"
        smooth: true
        z: parent.z + 1
        opacity: 1
        anchors.centerIn: parent

        property bool running: false
          property real duration: 1500
            property real angle: 360
              property int directionRot: RotationAnimation.Clockwise

                // Use RotationAnimator instead of RotationAnimation
                RotationAnimator {
                  id: rotationAnimator
                  target: imagenCentro
                  from: 0
                  to: 360
                  duration: imagenCentro.duration
                  running: true
                  loops: Animation.Infinite
                  direction: imagenCentro.directionRot
                }

                Component.onCompleted: {
                  MicrasPlugin.fan_speed_changed.connect(updateFanSpeed);
                }

                function updateFanSpeed(speed)
                {
                  if (speed == 0)
                  {
                    rotationAnimator.duration = 0;
                    return;
                  }
                  if (speed < 0)
                  {
                    imagenCentro.directionRot = RotationAnimation.Clockwise;
                  } else {
                  imagenCentro.directionRot = RotationAnimation.Counterclockwise;
                }

                imagenCentro.duration = 1500 - Math.abs(speed) * 15;

                // Directly update the RotationAnimator properties
                rotationAnimator.duration = imagenCentro.duration;
                rotationAnimator.direction = imagenCentro.directionRot;
              }
            }


          }
        }
        Text {
          text: "Fan Speed " + imagenCentro.duration.toFixed(1)
          anchors.verticalCenter: parent.verticalCenter
        }
      }

      RowLayout {
        spacing: 10

        Layout.alignment: Qt.AlignHCenter | Qt.AlignVCenter

        Slider {
          id: control
          from: 0.0
          to: 8.4
          stepSize: 0.1
          value: 8.4 // Initial value
          onValueChanged: {
            MicrasPlugin.on_slider_change(value);
          }

          background: Rectangle {
            x: control.leftPadding
            y: control.topPadding + control.availableHeight / 2 - height / 2
            implicitWidth: 170
            implicitHeight: 30
            width: control.availableWidth
            height: implicitHeight
            radius: 6
            color: "#bdbebf"

            Rectangle {
              width: control.visualPosition * parent.width
              height: parent.height
              color: {
                var r = control.visualPosition < 0.5 ? 1 : 1 - (control.visualPosition - 0.5) * 2
                var g = control.visualPosition < 0.5 ? control.visualPosition * 2 : 1
                var b = 0
                Qt.rgba(r, g, b)
              }
              radius: 6
            }
          }

          handle: Rectangle {
            x: control.leftPadding + control.visualPosition * (control.availableWidth - width)
            y: control.topPadding + control.availableHeight / 2 - height / 2
            implicitWidth: 15
            implicitHeight: 40
            radius: 6
            color: control.pressed ? "#f0f0f0" : "#f6f6f6"
            border.color: "#bdbebf"
          }
        }

        Label {
          text: "Battery Voltage: " + control.value.toFixed(1) + "V"
        }
      }


    }
  }

import QtQuick 2.7
import QtQuick.Controls 2.0
import QtQuick.Layouts 1.3

ColumnLayout {
  spacing: 10
  RowLayout {
    id: fanRow
    spacing: -1

    Rectangle {
      id: rect1
      transform: Rotation { origin.x: rect1.width / 2; origin.y: rect1.height / 2; angle: 180}
      width: 100
      height: 40
      radius: 0
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
      implicitHeight: 45
      radius: 6
      color: "grey"
    }

    Rectangle {
      id: rect2
      width: 100
      height: 40
      radius: 0
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

  Text {
    text: " Fan Speed: 0%"
    font.pixelSize: 15
    Component.onCompleted: {
      MicrasPlugin.fan_speed_changed.connect(updateFanSpeed);
    }

    function updateFanSpeed(speed)
    {
      text = " Fan Speed: " + speed.toFixed(1) + "%";
    }
  }
}

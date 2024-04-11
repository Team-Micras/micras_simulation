import QtQuick 2.7
import QtQuick.Controls 2.0
import QtQuick.Layouts 1.3

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

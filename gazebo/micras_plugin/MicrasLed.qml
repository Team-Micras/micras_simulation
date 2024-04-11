import QtQuick 2.7
import QtQuick.Controls 2.0
import QtQuick.Layouts 1.3

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

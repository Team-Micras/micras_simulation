import QtQuick 2.7
import QtQuick.Controls 2.0
import QtQuick.Layouts 1.3

Row {
  spacing: 25

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

    Label {
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

    Label {
      text: "ARGB 1"
      anchors.horizontalCenter: parent.horizontalCenter
    }
  }
}

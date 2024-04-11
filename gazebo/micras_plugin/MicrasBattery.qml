import QtQuick 2.7
import QtQuick.Controls 2.0
import QtQuick.Layouts 1.3

RowLayout {
  width: parent.width
  spacing: -5

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
      y: control.availableHeight / 2 - height / 2
      implicitWidth: 200
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
      x: control.visualPosition * (control.availableWidth - width)
      y: control.availableHeight / 2 - height / 2
      implicitWidth: 15
      implicitHeight: 40
      radius: 6
      color: control.pressed ? "#f0f0f0" : "#f6f6f6"
      border.color: "#bdbebf"
    }
  }

  Label {
    text: "Battery Voltage: " + control.value.toFixed(1) + "V"
    anchors.top: parent.top
    padding: 10
  }
}

import QtQuick 2.7
import QtQuick.Controls 2.0
import QtQuick.Layouts 1.3

import "qrc:/MicrasPlugin_qml"

ColumnLayout {

  Layout.minimumWidth: 400
  Layout.minimumHeight: 440
  Layout.margins: 5
  anchors.fill: parent
  focus: true

  Column {
    anchors.centerIn: parent
    spacing: 25

    RowLayout {
      width: parent.width
      MicrasButton {

      }
      Rectangle { Layout.fillWidth: true }
      MicrasDipSwitch {}
    }

    RowLayout {
      spacing: 25
      MicrasLed {}
      MicrasArgb {}
      MicrasBuzzer {}
    }

    RowLayout {
      width: parent.width
      MicrasFan {}
      Rectangle { Layout.fillWidth: true }
      MicrasFanBar {}
    }

    RowLayout {
      width: parent.width
      MicrasBattery {}
    }
  }
}

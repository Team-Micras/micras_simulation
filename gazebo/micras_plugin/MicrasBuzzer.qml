import QtQuick 2.7
import QtQuick.Controls 2.0
import QtQuick.Layouts 1.3

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
    }
    else
    {
      color = Qt.rgba(0, 1, 0);
      buzzerText.text = freq + "Hz";
    }
  }
}

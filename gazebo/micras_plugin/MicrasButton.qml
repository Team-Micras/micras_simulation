import QtQuick 2.7
import QtQuick.Controls 2.0
import QtQuick.Layouts 1.3

Button {
  text: "Button"
  width: 140
  height: 85
  Layout.preferredWidth: 140
  Layout.preferredHeight: 85

  onPressedChanged: {
    if (pressed)
    {
      MicrasPlugin.on_button_click();
    }
    else
    {
      MicrasPlugin.on_button_release();
    }
  }
}

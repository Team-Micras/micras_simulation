import QtQuick 2.7
import QtQuick.Controls 2.0
import QtQuick.Layouts 1.3

Rectangle {
  width: 120
  height: 70
  radius: 10
  color: "lightgrey"

  Rectangle {
    width: 120
    height: 70
    radius: 1000
    color: "transparent"
    clip: true

    Timer {
      id: timer
      interval: 10
      running: true
      repeat: true
      onTriggered: {
        rotationAnimator.start();
      }
    }

    Image {
      id: imagenCentro
      width: 55
      height: 55
      fillMode: Image.PreserveAspectCrop
      source: "qrc:/assets/fan.png"
      smooth: true
      z: parent.z + 1
      opacity: 1
      anchors.centerIn: parent

      RotationAnimator {
        id: rotationAnimator
        target: imagenCentro
        from: 0
        to: 51.43
        duration: 0
        running: true
        loops: 1
        direction: RotationAnimator.Clockwise
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
        if (speed > 0)
        {
          rotationAnimator.direction = RotationAnimator.Clockwise;
          rotationAnimator.to = 51.43;
        }
        else
        {
          rotationAnimator.direction = RotationAnimator.Counterclockwise;
          rotationAnimator.to = -51.43;
        }

        rotationAnimator.duration = 250 - 2 * Math.abs(speed);
        rotationAnimator.start();
      }
    }
  }
}

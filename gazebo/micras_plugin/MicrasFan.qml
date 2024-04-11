import QtQuick 2.7
import QtQuick.Controls 2.0
import QtQuick.Layouts 1.3

Rectangle {
  // color: "transparent"
  width: 120
  height: 70
  radius: 10
  // border.color: "lightgrey"
  // border.width: 1
  color: "lightgrey"
  //anchors.horizontalCenter: parent.horizontalCenter

  Rectangle {
    width: 120
    height: 70
    radius: 1000
    color: "transparent"
    clip: true

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
                }
                else
                {
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

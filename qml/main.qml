import QtQuick 2.15
import QtQuick.Controls 2.15
import Robot 1.0

ApplicationWindow {
    width: 400
    height: 500
    visible: true
    title: "Robot Kinematics"

    RobotKinematics {
        id: robot
    }

    Column {
        id: mainColumn
        spacing: 10
        anchors.centerIn: parent

        // Явная инициализация списка
        property var thetaInputs: []

        Repeater {
            model: 6
            delegate: Row {
                id: inputRow
                spacing: 5

                Label {
                    text: "Theta " + index + " (°):"
                    width: 100
                }
                TextField {
                    id: inputField
                    width: 100
                    validator: DoubleValidator {
                        bottom: -360
                        top: 360
                    }
                    text: getDefaultValue(index)
                    // Сохраняем ссылку при создании
                    Component.onCompleted: {
                        mainColumn.thetaInputs[index] = this
                    }
                }
            }
        }

        Button {
            text: "Calculate"
            anchors.horizontalCenter: parent.horizontalCenter
            onClicked: {
                var angles = []
                for (var i = 0; i < 6; ++i) {
                    if (mainColumn.thetaInputs[i]) {
                        angles.push(Number(mainColumn.thetaInputs[i].text))
                    }
                }
                if (angles.length === 6) {
                    var pos = robot.calculatePosition(angles)
                    resultText.text = "Position:\nX: " + pos.x +
                                    "\nY: " + pos.y +
                                    "\nZ: " + pos.z
                }
            }
        }

        Text {
            id: resultText
            text: "Position will be here"
            font.pixelSize: 16
            anchors.horizontalCenter: parent.horizontalCenter
        }
    }

    function getDefaultValue(index) {
        const defaults = ["15", "-50", "-60", "95", "50", "0"]
        return defaults[index]
    }
}

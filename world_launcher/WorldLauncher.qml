import QtQuick 2.9
import QtQuick.Window 2.2
import QtQuick.Controls 2.1
import QtQuick.Controls.Material 2.2
import QtQuick.Controls.Material.impl 2.2
import QtQuick.Layouts 1.3
import QtQuick.Dialogs 1.0


import QtQml.Models 2.2

import "qrc:/qml"

GridLayout {
  columns: 1
  columnSpacing: 10
  rows: 3
  rowSpacing: 10
  Layout.minimumWidth: 1000
  Layout.minimumHeight: 350
  anchors.fill: parent
  anchors.leftMargin: 10
  anchors.rightMargin: 10
  anchors.bottomMargin: 10
  anchors.topMargin: 10
  
  Rectangle {
    Layout.column: 1
    Layout.row: 1
    Layout.fillWidth: true
    height: 40
    color: "transparent"
    Text {
          id: appName
          color: "dimgrey"
          font.pointSize: 24
          anchors.horizontalCenter: parent.horizontalCenter
          text: "MovAi Simulator Launcher"
        }
  }
  

  TabBar {
    Layout.column: 1
    Layout.row: 2
    id: bar
    TabButton {
      text: "Fuel Worlds"
      width: implicitWidth
    }
    TabButton {
      text: "Local Worlds"
      width: implicitWidth
    }
  }

  StackLayout {
    Layout.column: 1
    Layout.row: 3
    currentIndex: bar.currentIndex

    // FUEL REPOSITORY:
    Item {
      id: fuelTab

      Rectangle {
        anchors.fill: parent
        GridLayout {
          columns: 3
          columnSpacing: 10
          rows: 2
          rowSpacing: 10
          Layout.minimumWidth: 1000
          Layout.minimumHeight: 300
          anchors.fill: parent
          anchors.leftMargin: 10
          anchors.rightMargin: 10
          anchors.bottomMargin: 10
          anchors.topMargin: 10

          Text {
            Layout.column: 1
            Layout.row: 1
            id: topicLabel
            font.pointSize: 12
            color: "dimgrey"
            text: "World Owner: "
          }

          TextField {
            Layout.column: 2
            Layout.row: 1
            id: topicField
            text: "movai"
            placeholderText: qsTr("Owner of the Worlds")
            onEditingFinished: {
              WorldLauncher.OnOwnerSelection(text)
            }
          }

          BusyIndicator {
            Layout.column: 3
            Layout.row: 1
            id: loadingIcon
            running: WorldLauncher.loadingStatus
          }

          Text {
            Layout.column: 1
            Layout.row: 2
            id: fuelText
            font.pointSize: 12
            color: "dimgrey"
            text: "Fuel World Scene: "
          }

          ComboBox {
            Layout.column: 2
            Layout.row: 2
            Layout.minimumWidth: 600
            id: fuelCombo
            model: WorldLauncher.fuelWorldsList
            currentIndex: 0
            onCurrentIndexChanged: {
              if (currentIndex < 0)
                return;
                WorldLauncher.SetFuelWorld(textAt(currentIndex));
              }
              ToolTip.visible: hovered
              ToolTip.delay: tooltipDelay
              ToolTip.timeout: tooltipTimeout
              ToolTip.text: qsTr("Fuel World Scene names available")
            }

            Button {
              Layout.alignment: Qt.AlignCenter
              Layout.minimumWidth: 150
              Layout.preferredHeight: 50
              Layout.column: 3
              Layout.row: 2
              id: fuelButton
              text: qsTr("Start Simulator")
              highlighted: true
              enabled: WorldLauncher.simulationStatus && !WorldLauncher.loadingStatus && WorldLauncher.validFuelWorld
              onClicked: {
                WorldLauncher.OnFuelButton();
              }
              ToolTip.visible: hovered
              ToolTip.delay: tooltipDelay
              ToolTip.timeout: tooltipTimeout
              ToolTip.text: qsTr("Start the world scene simulation")
            }

          }
        }
      }

      Item {
        id: localTab

        Rectangle {
          anchors.fill: parent
          GridLayout {
            columns: 3
            columnSpacing: 10
            rows: 1
            rowSpacing: 10
            Layout.minimumWidth: 1000
            Layout.minimumHeight: 300
            anchors.fill: parent
            anchors.leftMargin: 10
            anchors.rightMargin: 10
            anchors.bottomMargin: 10
            anchors.topMargin: 10
            Text {
              Layout.column: 1
              Layout.row: 1
              id: localText
              font.pointSize: 12
              color: "dimgrey"
              text: "Local World Scene: "
            }

            ComboBox {
              Layout.column: 2
              Layout.row: 1
              Layout.minimumWidth: 600
              id: localCombo
              model: WorldLauncher.worldsList
              currentIndex: 0
              onCurrentIndexChanged: {
                if (currentIndex < 0)
                  return;
                  WorldLauncher.SetWorld(textAt(currentIndex));
                }
                ToolTip.visible: hovered
                ToolTip.delay: tooltipDelay
                ToolTip.timeout: tooltipTimeout
                ToolTip.text: qsTr("Local World Scene names available")
              }

              Button {
                Layout.alignment: Qt.AlignCenter
                Layout.minimumWidth: 150
                Layout.preferredHeight: 50
                Layout.column: 3
                Layout.row: 1
                id: localButton
                text: qsTr("Start Simulator")
                highlighted: true
                enabled: WorldLauncher.simulationStatus && WorldLauncher.validLocalWorld
                onClicked: {
                  WorldLauncher.OnButton();
                }
                ToolTip.visible: hovered
                ToolTip.delay: tooltipDelay
                ToolTip.timeout: tooltipTimeout
                ToolTip.text: qsTr("Start the world scene simulation")
              }

            }
          }
        }
      }

    }


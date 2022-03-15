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
  columnSpacing: 5
  rows: 3
  rowSpacing: 5
  Layout.minimumWidth: 1050
  Layout.minimumHeight: 500
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

    GridLayout {
          columns: 3
          columnSpacing: 5
          rows: 1
          rowSpacing: 5
          Layout.minimumWidth: 1050
          Layout.minimumHeight: 450
          anchors.fill: parent
          anchors.leftMargin: 10
          anchors.rightMargin: 10
          anchors.bottomMargin: 10
          anchors.topMargin: 10
          Text {
            Layout.column: 2
            id: appName
            color: "dimgrey"
            font.pointSize: 24
            anchors.horizontalCenter: parent.horizontalCenter
            text: "MOV.AI Simulator Launcher"
          }
          Image {
            Layout.column: 3
            Layout.alignment: Qt.AlignRight
            fillMode: Image.Pad
            horizontalAlignment: Image.AlignHCenter
            verticalAlignment: Image.AlignVCenter
            source: "images/movai-logo.png"
            sourceSize.width: 150;
            sourceSize.height: 70;
          }
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
        ColumnLayout {
          spacing: 5
          GridLayout {
            columns: 3
            columnSpacing: 10
            rows: 3
            rowSpacing: 5
            Layout.minimumWidth: 1000
            Layout.minimumHeight: 250
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
            Text {
              Layout.column: 1
              Layout.row: 2
              font.pointSize: 12
              color: "dimgrey"
              text: "Render Engine: "
            }
            ComboBox {
              Layout.column: 2
              Layout.row: 2
              Layout.minimumWidth: 50
              id: fuelRenderCombo
              model: ["ogre2", "ogre"]
              currentIndex: 0
              onCurrentIndexChanged: {
                if (currentIndex < 0)
                  return;
                WorldLauncher.SetRenderWorld(textAt(currentIndex));
              }
              ToolTip.visible: hovered
              ToolTip.delay: tooltipDelay
              ToolTip.timeout: tooltipTimeout
              ToolTip.text: qsTr("Render Engine that is used by the scene and sensors")
            }
            Text {
              Layout.column: 1
              Layout.row: 3
              id: fuelText
              font.pointSize: 12
              color: "dimgrey"
              text: "Fuel World Scene: "
            }
            ComboBox {
              Layout.column: 2
              Layout.row: 3
              Layout.minimumWidth: 700
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
            RowLayout {
              Layout.column: 3
              Layout.row: 3
              Layout.alignment: Qt.AlignLeft
              RoundButton {
                text: "\u21bb"
                Material.background: Material.primary
                onClicked: {
                  WorldLauncher.OnOwnerSelection(topicField.text);
                }
                ToolTip.visible: hovered
                ToolTip.delay: tooltipDelay
                ToolTip.timeout: tooltipTimeout
                ToolTip.text: qsTr("Refresh World list")
              }
              BusyIndicator {
                id: loadingIcon
                running: WorldLauncher.loadingStatus
              }
            }
          }
          Button {
            Layout.alignment: Qt.AlignCenter
            Layout.fillWidth: true
            Layout.preferredHeight: 50
            Layout.leftMargin: 40
            Layout.rightMargin: 20
            id: fuelButton
            text: qsTr("Start Simulator")
            highlighted: true
            enabled: WorldLauncher.simulationStatus && !WorldLauncher.loadingStatus && WorldLauncher.validFuelWorld
            onClicked: {
              WorldLauncher.SetRenderWorld(fuelRenderCombo.model[fuelRenderCombo.currentIndex]);
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
        ColumnLayout {
          spacing: 5
          GridLayout {
            columns: 3
            columnSpacing: 10
            rows: 2
            rowSpacing: 5
            Layout.minimumWidth: 1000
            Layout.minimumHeight: 250
            anchors.fill: parent
            anchors.leftMargin: 10
            anchors.rightMargin: 10
            anchors.bottomMargin: 10
            anchors.topMargin: 10
            Text {
              Layout.column: 1
              Layout.row: 1
              font.pointSize: 12
              color: "dimgrey"
              text: "Render Engine: "
            }
            ComboBox {
              Layout.column: 2
              Layout.row: 1
              Layout.minimumWidth: 50
              id: localRenderCombo
              model: ["ogre2", "ogre"]
              currentIndex: 0
              onCurrentIndexChanged: {
                if (currentIndex < 0)
                  return;
                WorldLauncher.SetRenderWorld(textAt(currentIndex));
              }
              ToolTip.visible: hovered
              ToolTip.delay: tooltipDelay
              ToolTip.timeout: tooltipTimeout
              ToolTip.text: qsTr("Render Engine that is used by the scene and sensors")
            }

            Text {
              Layout.column: 1
              Layout.row: 2
              id: localText
              font.pointSize: 12
              color: "dimgrey"
              text: "Local World Scene: "
            }
            ComboBox {
              Layout.column: 2
              Layout.row: 2
              Layout.minimumWidth: 700
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
            RowLayout {
              Layout.column: 3
              Layout.row: 2
              Layout.alignment: Qt.AlignLeft
              RoundButton {
                text: "\u002b"
                Material.background: Material.LightBlue
                enabled: WorldLauncher.simulationStatus
                onClicked: {
                  WorldLauncher.SetRenderWorld(localRenderCombo.model[localRenderCombo.currentIndex]);
                  WorldLauncher.OnCreateButton();
                }
                ToolTip.visible: hovered
                ToolTip.delay: tooltipDelay
                ToolTip.timeout: tooltipTimeout
                ToolTip.text: qsTr("Start empty World scene")
              }
              
              RoundButton {
                text: "\u21bb"
                Material.background: Material.primary
                onClicked: {
                  WorldLauncher.LoadLocalList();
                }
                ToolTip.visible: hovered
                ToolTip.delay: tooltipDelay
                ToolTip.timeout: tooltipTimeout
                ToolTip.text: qsTr("Refresh World list")
              }
            }
          }
          Button {
            Layout.alignment: Qt.AlignCenter
            Layout.fillWidth: true
            Layout.preferredHeight: 50
            Layout.leftMargin: 40
            Layout.rightMargin: 20
            id: localButton
            text: qsTr("Start Simulator")
            highlighted: true
            enabled: WorldLauncher.simulationStatus && WorldLauncher.validLocalWorld
            onClicked: {
              WorldLauncher.SetRenderWorld(localRenderCombo.model[localRenderCombo.currentIndex]);
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


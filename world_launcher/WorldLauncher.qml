/*
 * Copyright (C) 2018 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
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
  columns: 4
  columnSpacing: 10
  rows: 3
  rowSpacing: 10
  Layout.minimumWidth: 1000
  Layout.minimumHeight: 300
  anchors.fill: parent
  anchors.leftMargin: 10
  anchors.rightMargin: 10

  Text {
    Layout.column: 1
    Layout.row: 1
    id: localText
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
    text: qsTr("Local Play!")
    highlighted: true
    onClicked: {
      WorldLauncher.OnButton();
    }
    ToolTip.visible: hovered
    ToolTip.delay: tooltipDelay
    ToolTip.timeout: tooltipTimeout
    ToolTip.text: qsTr("Start the world scene simulation")
  }
  
  // FUEL REPOSITORY:
  // Topic input
  Label {
    Layout.column: 1
    Layout.row: 2
    id: topicLabel
    color: "dimgrey"
    text: "Owner:"
  }

  TextField {
    Layout.column: 2
    Layout.row: 2
    id: topicField
    text:"openrobotics"
    placeholderText: qsTr("Owner of the Worlds")
    onEditingFinished: {
      WorldLauncher.OnOwnerSelection(text)
    }
  }

  RoundButton {
    Layout.column: 3
    Layout.row: 2
    text: "\u21bb"
    Material.background: Material.primary
    onClicked: {
      WorldLauncher.LoadFuelList();
    }
    ToolTip.visible: hovered
    ToolTip.delay: tooltipDelay
    ToolTip.timeout: tooltipTimeout
    ToolTip.text: qsTr("Refresh list of FUEL worlds")
  }
  
  Text {
    Layout.column: 1
    Layout.row: 3
    id: fuelText
    color: "dimgrey"
    text: "Fuel World Scene: "
  }

  ComboBox {
    Layout.column: 2
    Layout.row: 3
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
    Layout.row: 3
    id: fuelButton
    text: qsTr("Fuel Play!")
    highlighted: true
    onClicked: {
      WorldLauncher.OnFuelButton();
    }
    ToolTip.visible: hovered
    ToolTip.delay: tooltipDelay
    ToolTip.timeout: tooltipTimeout
    ToolTip.text: qsTr("Start the world scene simulation")
  }
}
import QtQuick 2.9
import QtQuick.Controls 2.1
import QtQuick.Dialogs 1.0
import QtQuick.Layouts 1.3
import "qrc:/qml"



Rectangle {
  Layout.minimumWidth: 320
  Layout.minimumHeight: 350
  anchors.leftMargin: 10
  anchors.rightMargin: 10
  color: "transparent"

  GridLayout {
    columns: 2
    columnSpacing: 10
    rows: 3
    rowSpacing: 10
    Layout.minimumWidth: 300
    Layout.minimumHeight: 300
    anchors.fill: parent
    anchors.leftMargin: 10
    anchors.rightMargin: 10
  
    Image {
      Layout.alignment: Qt.AlignHCenter
      Layout.topMargin: 15
      Layout.column: 1
      Layout.row: 1
      fillMode: Image.Pad
      horizontalAlignment: Image.AlignHCenter
      verticalAlignment: Image.AlignVCenter
      source: "images/emergency_button.png"
      sourceSize.width: 70;
      sourceSize.height: 70;
    }

    Switch {
      id: booleanSwitch
      anchors.right: content.right
      Layout.column: 2
      Layout.row: 1
      checked: EmergencyButton.emergencyStatus
      enabled: true
      height: 10
      width: 50
      onClicked: EmergencyButton.SetEmergency();
      ToolTip.visible: hovered
      ToolTip.delay: tooltipDelay
      ToolTip.timeout: tooltipTimeout
      ToolTip.text: qsTr("Starts Emergency mode")
    }
  
    Text {
      Layout.column: 1
      Layout.row: 2
      id: selectText
      font.pointSize: 12
      color: "dimgrey"
      text: "Select the model: "
    }
  
    ComboBox {
      Layout.column: 2
      Layout.row: 2
      Layout.minimumWidth: 150
      id: modelCombo
      model: EmergencyButton.modelsList
      currentIndex: 0
      onCurrentIndexChanged: {
        if (currentIndex < 0)
          return;
        EmergencyButton.SetModel(textAt(currentIndex));
      }
      ToolTip.visible: hovered
      ToolTip.delay: tooltipDelay
      ToolTip.timeout: tooltipTimeout
      ToolTip.text: qsTr("Models names available")
    }

  }

}

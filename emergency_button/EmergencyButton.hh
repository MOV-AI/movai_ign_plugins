#ifndef IGNITION_GAZEBO_GUI_EMERGENCYBUTTON_HH_
#define IGNITION_GAZEBO_GUI_EMERGENCYBUTTON_HH_

#include <ignition/msgs/boolean.pb.h>
#include <ignition/transport/Node.hh>
#include "ignition/gazebo/EntityComponentManager.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/Model.hh"
#include <ignition/gui/Plugin.hh>
#include <ignition/gazebo/gui/GuiSystem.hh>

namespace ignition
{
  namespace gazebo
  {
    class EmergencyButton : public ignition::gazebo::GuiSystem
    {
      Q_OBJECT

      /// \brief Fuel Worlds list GUI QT object.
      Q_PROPERTY(
          QStringList modelsList
              READ ModelsList
                  WRITE SetModelsList
                      NOTIFY ModelsListChanged)

      /// \brief Emergency status GUI QT object.
      Q_PROPERTY(
          bool emergencyStatus
              READ EmergencyStatus
                  WRITE SetEmergencyStatus
                      NOTIFY EmergencyStatusChanged)

      /// \brief Constructor
    public:
      EmergencyButton();

      /// \brief Destructor.
      ~EmergencyButton() override;

      /////////////////////////////////////////////////
      /// \brief Called by Ignition GUI when plugin is instantiated.
      /// \param[in] _pluginElem XML configuration for this plugin.
      void LoadConfig(const tinyxml2::XMLElement *) override;

      /////////////////////////////////////////////////
      /// \brief Called by Ignition Gazebo when update the simulation.
      /// \param[in] _info Simulation info status.
      /// \param[in] _ecm Entity Component Manager.
      void Update(const UpdateInfo &_info, EntityComponentManager &_ecm) override;

      /////////////////////////////////////////////////
      /// \brief Called by Ignition GUI when QStringList is instantiated.
      Q_INVOKABLE QStringList ModelsList() const;

      /////////////////////////////////////////////////
      /// \brief Called by Ignition GUI when QStringList is instantiated.
      /// \param[in] _modelsList QStringList to update
      Q_INVOKABLE void SetModelsList(const QStringList &_modelsList);

      /////////////////////////////////////////////////
      /// \brief Called by Ignition GUI when QBool is instantiated.
      Q_INVOKABLE bool EmergencyStatus() const;

      /////////////////////////////////////////////////
      /// \brief Called by Ignition GUI when QBool is instantiated.
      /// \param[in] _status QBool to update
      Q_INVOKABLE void SetEmergencyStatus(const bool _status);

      /// \brief List of models.
      QStringList modelsList;

      /// \brief Status of Simulation.
      bool emergencyStatus{false};

    signals:
      /// \brief Notify that the models list has changed
      void ModelsListChanged();

      /// \brief Notify that the Simulation status has changed
      void EmergencyStatusChanged();

    protected slots:
      /////////////////////////////////////////////////
      /// \brief Called by Ignition GUI when there is a change in the combobox of model list.
      /// \param[in] _sortType the selected string in the combobox.
      void SelectedModel(const QString &_sortType);

      /////////////////////////////////////////////////
      /// \brief Called by Ignition GUI when there is a change in the combobox of the model.
      /// \param[in] _sortType the selected string in the combobox.
      void SetModel(const QString &_sortType);

      /////////////////////////////////////////////////
      /// \brief Called by Ignition GUI when there is a change the emergency state.
      void SetEmergency();

    private:
      transport::Node node;
      std::string modelName;
      transport::Node::Publisher emergencyPub;
      bool initialized{false};

      /// \brief Vars to control the publish rate
      std::chrono::steady_clock::duration msgPubPeriod{0};
      std::chrono::steady_clock::duration lastMsgPubTime{0};
    };
  }
}

#endif

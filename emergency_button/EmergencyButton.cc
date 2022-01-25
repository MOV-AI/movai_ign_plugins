#include "EmergencyButton.hh"
#include <ignition/plugin/Register.hh>

using namespace ignition;
using namespace gazebo;

/////////////////////////////////////////////////
EmergencyButton::EmergencyButton()
    : GuiSystem()
{
}

/////////////////////////////////////////////////
EmergencyButton::~EmergencyButton()
{
}

/////////////////////////////////////////////////
/// \brief Called by Ignition GUI when plugin is instantiated.
/// \param[in] _pluginElem XML configuration for this plugin.
void EmergencyButton::LoadConfig(const tinyxml2::XMLElement *_pluginElem)
{
  if (!_pluginElem)
    return;

  if (this->title.empty())
    this->title = "Emergency Button";

  // Clear the list at begin
  this->modelsList.clear();
  // define the pub rate
  std::chrono::duration<double> msgPer{1 / 10};
  this->msgPubPeriod = std::chrono::duration_cast<std::chrono::steady_clock::duration>(msgPer);
}

/////////////////////////////////////////////////
/// \brief Called by Ignition GUI when QStringList is instantiated.
QStringList EmergencyButton::ModelsList() const
{
  return this->modelsList;
}

/////////////////////////////////////////////////
/// \brief Called by Ignition GUI when QStringList is instantiated.
/// \param[in] _modelsList QStringList to update
void EmergencyButton::SetModelsList(const QStringList &_modelsList)
{
  this->modelsList = _modelsList;
  this->modelsList.sort(Qt::CaseInsensitive);
  this->ModelsListChanged();
}

/////////////////////////////////////////////////
/// \brief Called by Ignition GUI when QBool is instantiated.
bool EmergencyButton::EmergencyStatus() const
{
  return this->emergencyStatus;
}

/////////////////////////////////////////////////
/// \brief Called by Ignition GUI when QBool is instantiated.
/// \param[in] _status QBool to update
void EmergencyButton::SetEmergencyStatus(const bool _status)
{
  this->emergencyStatus = _status;
  this->EmergencyStatusChanged();
}

/////////////////////////////////////////////////
/// \brief Called by Ignition GUI when there is a change in the combobox of model list.
/// \param[in] _sortType the selected string in the combobox.
void EmergencyButton::SetModel(const QString &_sortType)
{
  // Set the status to the default position "false"
  this->SetEmergencyStatus(false);
  // Assin to propery
  this->modelName = _sortType.toStdString();
}

/////////////////////////////////////////////////
/// \brief Called by Ignition GUI when there is a change the emergency state.
void EmergencyButton::SetEmergency()
{
  this->SetEmergencyStatus(!this->emergencyStatus);
}

/////////////////////////////////////////////////
/// \brief Called by Ignition Gazebo when update the simulation.
/// \param[in] _info Simulation info status.
/// \param[in] _ecm Entity Component Manager.
void EmergencyButton::Update(const UpdateInfo &_info,
                             EntityComponentManager &_ecm)
{
  // Load all models name at the begin
  if (!this->initialized)
  {
    _ecm.Each<components::Name>(
        [&](const Entity &_entity, const components::Name *_name) -> bool
        {
          auto modelComp = _ecm.Component<components::Model>(_entity);
          if (!modelComp)
          {
            return true;
          }
          // Create the List to show to the User in the GUI
          this->modelsList.push_back(QString::fromStdString(_name->Data()));
          return true;
        });
    this->modelsList.sort(Qt::CaseInsensitive);
    this->ModelsListChanged();
    this->SetModel(this->modelsList[0]);
    if (!this->modelsList.isEmpty())
      this->initialized = true;
  }
  else
  {
    // Only update it when there is a new model
    _ecm.EachNew<components::Name>(
        [&](const Entity &_entity, const components::Name *_name) -> bool
        {
          auto modelComp = _ecm.Component<components::Model>(_entity);
          if (!modelComp)
          {
            return true;
          }
          // Create the List to show to the User in the GUI
          this->modelsList.push_back(QString::fromStdString(_name->Data()));
          this->modelsList.sort(Qt::CaseInsensitive);
          this->ModelsListChanged();
          this->SetModel(this->modelsList[0]);
          return true;
        });
  }

  // Nothing to do if the simulation is paused
  if (_info.paused)
    return;

  // Only publish with the update rate
  auto diff = _info.simTime - this->lastMsgPubTime;
  if (diff > std::chrono::steady_clock::duration::zero() &&
      diff < this->msgPubPeriod)
  {
    return;
  }
  else
  {
    this->lastMsgPubTime = _info.simTime;
    // Check if there is a topic to publish
    std::string emergengyTopic = "/" + this->modelName + "/emergency_sw_detect";
    std::vector<std::string> topicList = this->node.AdvertisedTopics();
    if (topicList.size() > 0)
    {
      bool hasTopic = false;
      for (size_t i = 0; i < topicList.size(); i++)
      {
        if (topicList[i].compare(emergengyTopic) == 0)
        {
          hasTopic = true;
        }
      }
      if (!hasTopic)
      {
        this->emergencyPub = this->node.Advertise<msgs::Boolean>(emergengyTopic);
      }
    }
    else
    {
      this->emergencyPub = this->node.Advertise<msgs::Boolean>(emergengyTopic);
    }

    // Publish the emergency status
    msgs::Boolean msgState;
    msgState.set_data(this->emergencyStatus);
    this->emergencyPub.Publish(msgState);
  }
}

// Register this plugin
IGNITION_ADD_PLUGIN(ignition::gazebo::EmergencyButton,
                    ignition::gui::Plugin)

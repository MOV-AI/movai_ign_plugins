#include <ignition/plugin/Register.hh>

#include "BatteryStation.hh"

using namespace ignition;
using namespace gazebo;
using namespace systems;

/////////////////////////////////////////////////
/// \brief Constructor
BatteryStation::BatteryStation() = default;

/////////////////////////////////////////////////
/// \brief Destructor
BatteryStation::~BatteryStation() = default;

//////////////////////////////////////////////////
/// \brief This function is called when the model attached is loaded in the simulation
/// \param[in] _entity Object model that this plugin is attached (AKA charge stations models)
/// \param[in] _sdf SDF element of the plugin in the model attached 
/// \param[in] _ecm Entity Component Manager
void BatteryStation::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &/*_eventMgr*/)
{
  // Get the model and check if it is valid
  this->model = Model(_entity);
  if (!this->model.Valid(_ecm))
  {
    ignerr << "BatteryStation plugin should be attached to a model entity. "
           << "Failed to initialize." << std::endl;
    return;
  }

  // Get the battery name used in the robot. This name is a Ignition Gazebo linear battery plugin name
  this->batteryName = _sdf->Get<std::string>("battery_name");
  if (this->batteryName.empty())
  {
    ignerr << "BatteryStation found an empty battery_name parameter. This name should be in the robot sdf model, in the linear battery plugin."
           << "Failed to initialize.";
    return;
  }
  
  // Get the world name in the simulation
  auto worldEntity = _ecm.EntityByComponents(components::World());
  if (worldEntity == kNullEntity)
    return;

  // Get the components name to create the contact topic subscriber
  std::string chargeName = _ecm.Component<components::Name>(_entity)->Data();
  std::string worldName = _ecm.Component<components::Name>(worldEntity)->Data();
  std::string topic = "/world/" + worldName + "/model/" + chargeName + "/link/charging_station_base_link/sensor/charge/contact";

  // Subscribe to the charge station contact topic
  this->node.Subscribe(topic, &BatteryStation::OnContact,
                                this);

  ignmsg << "BatteryStation Started" << std::endl;
}

//////////////////////////////////////////////////
/// \brief This function is called in each simulation step update.
/// \param[in] _info Simulation state information
/// \param[in] _ecm Entity Component Manager
void BatteryStation::Update(const ignition::gazebo::UpdateInfo &_info,
    ignition::gazebo::EntityComponentManager &_ecm)
{
  IGN_PROFILE("BatteryStation::Update");

  // Nothing left to do if paused.
  if (_info.paused)
    return;
  
  // Create the ignition boolean message true due to different services to charge and discharge.
  msgs::Boolean msgState;
  msgState.set_data(true);

  // Check if the state of charge changed
  if (this->chargeState != this->oldState)
  {
    if (this->chargeState)
    {
      // Create the charge service topic using the contact informations and call the service to charge the battery
      const std::string srvCharge = "/model/" + this->contactRobotName + "/battery/" + this->batteryName + "/recharge/start";
      this->node.Request(srvCharge, msgState);
      ignmsg << "BatteryStation Charging" << std::endl;
    }
    else
    {
      // Create the charge service topic using the contact informations and call the service to discharge the battery
      const std::string srvStopCharge = "/model/" + this->contactRobotName + "/battery/" + this->batteryName + "/recharge/stop";
      this->node.Request(srvStopCharge, msgState);
      ignmsg << "BatteryStation Discharging" << std::endl;
    }
    // Update the old state
    this->oldState = this->chargeState;
  }

  // Set the state to false to verify if there still contact or not
  this->chargeState = false;
    
}

//////////////////////////////////////////////////
/// \brief Callback for contact subscription
/// \param[in] _msg Message
void BatteryStation::OnContact(const ignition::msgs::Contacts &_msg)
{
  // For each contact, get the object in the contact and use the base object as a "robotname"
  for (const auto &contact : _msg.contact())
  {
    const std::vector<std::string> seglist = SplitMsg(contact.collision2().name(), ':');
    this->contactRobotName = seglist[0];
    this->chargeState = true;
  }
}

/////////////////////////////////////////////////
/// \brief Function used to split a string by a delimiter character.
/// \param[in] str std::string to be splited.
/// \param[in] delim char to delimiter the strings separation.
/// \return Returns a Vector of strings. std::vector<std::string>
std::vector<std::string> BatteryStation::SplitMsg(std::string const &str, const char delim)
{
  size_t start;
  size_t end = 0;
  std::vector<std::string> out;

  while ((start = str.find_first_not_of(delim, end)) != std::string::npos)
  {
    end = str.find(delim, start);
    out.push_back(str.substr(start, end - start));
  }
  return out;
}


// Register this plugin
IGNITION_ADD_PLUGIN(BatteryStation,
                    ignition::gazebo::System,
                    BatteryStation::ISystemConfigure,
                    BatteryStation::ISystemUpdate)

IGNITION_ADD_PLUGIN_ALIAS(BatteryStation,
                          "ignition::gazebo::systems::BatteryStation")


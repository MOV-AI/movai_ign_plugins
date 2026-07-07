#include <ignition/plugin/Register.hh>
#include "SpawnMotionModel.hh"
#include <ignition/gazebo/components/ExternalWorldWrenchCmd.hh>

using namespace ignition;
using namespace gazebo;
using namespace systems;

/////////////////////////////////////////////////
/// \brief Constructor
SpawnMotionModel::SpawnMotionModel() = default;

/////////////////////////////////////////////////
/// \brief Destructor
SpawnMotionModel::~SpawnMotionModel() = default;

//////////////////////////////////////////////////
/// \brief This function is called when the model attached is loaded in the simulation
/// \param[in] _entity Object model that this plugin is attached
/// \param[in] _sdf SDF element of the plugin in the model attached
/// \param[in] _ecm Entity Component Manager
void SpawnMotionModel::Configure(const Entity &_entity,
                                 const std::shared_ptr<const sdf::Element> &_sdf,
                                 EntityComponentManager &_ecm,
                                 EventManager & /*_eventMgr*/)
{
  ignmsg << "SpawnMotionModel::Configure" << std::endl;

  // Get the model and check if it is valid
  this->model = Model(_entity);
  if (!this->model.Valid(_ecm))
  {
    ignerr << "SpawnMotionModel plugin should be attached to a model entity. "
           << "Failed to initialize." << std::endl;
    return;
  }

  // Resolve world name for service calls
  const auto worldEntity = _ecm.EntityByComponents(components::World());
  if (worldEntity == kNullEntity)
  {
    ignerr << "SpawnMotionModel failed to resolve world entity." << std::endl;
    return;
  }

  const auto *worldNameComp = _ecm.Component<components::Name>(worldEntity);
  if (worldNameComp == nullptr)
  {
    ignerr << "SpawnMotionModel failed to resolve world name component." << std::endl;
    return;
  }
  this->worldName = worldNameComp->Data();
  const auto *modelNameComp = _ecm.Component<components::Name>(_entity);
  if (modelNameComp != nullptr)
    this->objectName = modelNameComp->Data();

  if (_sdf && _sdf->HasElement("wrench_hold_steps"))
  {
    this->wrenchHoldSteps = _sdf->Get<uint32_t>("wrench_hold_steps");
  }

  const std::string topic{"/pedro_motion_topic"};
  const bool ok = this->node.Subscribe(topic, &SpawnMotionModel::OnMyTopic, this);
  if (!ok)
  {
    ignerr << "SpawnMotionModel failed to subscribe to topic [" << topic << "]" << std::endl;
  }
  else
  {
    ignmsg << "SpawnMotionModel subscribed to topic [" << topic << "]" << std::endl;
  }
}

//////////////////////////////////////////////////
/// \brief This function is called in each simulation step update.
/// \param[in] _info Simulation state information
/// \param[in] _ecm Entity Component Manager
void SpawnMotionModel::Update(const ignition::gazebo::UpdateInfo &_info,
                              ignition::gazebo::EntityComponentManager &_ecm)
{
  IGN_PROFILE("SpawnMotionModel::Update");

  // Nothing left to do if paused.
  if (_info.paused)
    return;

  ignition::msgs::Wrench wrenchCmd;
  std::string targetName;
  {
    std::lock_guard<std::mutex> lock(this->msgMutex);
    if (!this->hasPendingWrench)
      return;

    wrenchCmd = this->pendingWrench;
    targetName = this->pendingTargetName;

    if (this->wrenchStepsRemaining > 0)
      --this->wrenchStepsRemaining;

    if (this->wrenchStepsRemaining == 0)
      this->hasPendingWrench = false;
  }

  Entity targetModelEntity = kNullEntity;
  if (!targetName.empty())
  {
    targetModelEntity = _ecm.EntityByComponents(components::Model(),
                                                components::Name(targetName));
    if (targetModelEntity == kNullEntity)
    {
      ignerr << "SpawnMotionModel could not find target model [" << targetName
             << "] in world [" << this->worldName << "]" << std::endl;
      return;
    }
  }
  else
  {
    targetModelEntity = this->model.Entity();
    targetName = this->objectName;
  }

  Model targetModel(targetModelEntity);
  Entity linkEntity = targetModel.CanonicalLink(_ecm);
  if (linkEntity == kNullEntity)
  {
    auto links = targetModel.Links(_ecm);
    if (!links.empty())
      linkEntity = links.front();
  }

  if (linkEntity == kNullEntity)
  {
    ignerr << "SpawnMotionModel failed to find a link entity for model ["
           << targetName << "]" << std::endl;
    return;
  }

  _ecm.SetComponentData<components::ExternalWorldWrenchCmd>(linkEntity, wrenchCmd);
  ignmsg << "SpawnMotionModel applied wrench to model [" << targetName << "]" << std::endl;
}
// Necessary for retrieving the pose of a specified model
void SpawnMotionModel::PostUpdate(const UpdateInfo &_info,
                                  const EntityComponentManager &_ecm)
{
  IGN_PROFILE("SpawnMotionModel::PostUpdate");

  // Nothing left to do if paused.
  if (_info.paused)
    return;
  // ignmsg << "SpawnMotionModel::PostUpdate" << std::endl;
}

// //////////////////////////////////////////////////
/// \brief Callback for moving model command
/// \param[in] _msg Message

void SpawnMotionModel::OnMyTopic(const ignition::msgs::EntityWrench &_msg)
{
  // log message received
  ignmsg << "SpawnMotionModel::OnMyTopic: Received message with wrench data:" << std::endl;
  ignmsg << "Force: (" << _msg.wrench().force().x() << ", " << _msg.wrench().force().y() << ", " << _msg.wrench().force().z() << ")" << std::endl;
  ignmsg << "Torque: (" << _msg.wrench().torque().x() << ", " << _msg.wrench().torque().y() << ", " << _msg.wrench().torque().z() << ")" << std::endl;
  if (_msg.has_entity())
  {
    ignmsg << "Entity Name: " << _msg.entity().name() << std::endl;
    ignmsg << "Entity ID: " << _msg.entity().id() << std::endl;
    ignmsg << "Entity Type: " << _msg.entity().type() << std::endl;
  }

  {
    std::lock_guard<std::mutex> lock(this->msgMutex);
    this->pendingWrench = _msg.wrench();
    this->pendingTargetName = _msg.has_entity() ? _msg.entity().name() : "";
    this->hasPendingWrench = true;
    this->wrenchStepsRemaining = this->wrenchHoldSteps;
  }
}

// Register this plugin
IGNITION_ADD_PLUGIN(SpawnMotionModel,
                    ignition::gazebo::System,
                    SpawnMotionModel::ISystemConfigure,
                    SpawnMotionModel::ISystemUpdate,
                    SpawnMotionModel::ISystemPostUpdate)

IGNITION_ADD_PLUGIN_ALIAS(SpawnMotionModel,
                          "ignition::gazebo::systems::SpawnMotionModel")

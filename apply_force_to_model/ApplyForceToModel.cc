#include <ignition/plugin/Register.hh>
#include "ApplyForceToModel.hh"
#include <ignition/gazebo/components/ExternalWorldWrenchCmd.hh>
#include <iostream>

using namespace ignition;
using namespace gazebo;
using namespace systems;

void ApplyForceToModel::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager & /*_eventMgr*/)
{
  std::cerr << "[ApplyForceToModel] Configure" << std::endl;

  // This plugin is attached to a model and can fall back to that model as target.
  this->model = Model(_entity);
  if (!this->model.Valid(_ecm))
  {
    ignerr << "ApplyForceToModel plugin should be attached to a model entity. "
           << "Failed to initialize." << std::endl;
    return;
  }

  const auto worldEntity = _ecm.EntityByComponents(components::World());
  if (worldEntity == kNullEntity)
  {
    ignerr << "ApplyForceToModel failed to resolve world entity." << std::endl;
    return;
  }

  const auto *worldNameComp = _ecm.Component<components::Name>(worldEntity);
  if (worldNameComp == nullptr)
  {
    ignerr << "ApplyForceToModel failed to resolve world name component." << std::endl;
    return;
  }
  this->worldName = worldNameComp->Data();

  const auto *modelNameComp = _ecm.Component<components::Name>(_entity);
  if (modelNameComp != nullptr)
    this->defaultModelName = modelNameComp->Data();

  if (_sdf && _sdf->HasElement("wrench_hold_steps"))
    this->wrenchHoldSteps = _sdf->Get<uint32_t>("wrench_hold_steps");

  const std::string topic{"/apply_force_to_model_topic"};
  const bool ok = this->node.Subscribe(topic, &ApplyForceToModel::OnWrenchMessage, this);
  if (!ok)
  {
    ignerr << "ApplyForceToModel failed to subscribe to topic [" << topic << "]" << std::endl;
  }
  else
  {
    std::cerr << "[ApplyForceToModel] subscribed to topic [" << topic << "]" << std::endl;
  }
}

void ApplyForceToModel::Update(const ignition::gazebo::UpdateInfo &_info,
    ignition::gazebo::EntityComponentManager &_ecm)
{
  IGN_PROFILE("ApplyForceToModel::Update");

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
      ignerr << "ApplyForceToModel could not find target model [" << targetName
             << "] in world [" << this->worldName << "]" << std::endl;
      return;
    }
  }
  else
  {
    targetModelEntity = this->model.Entity();
    targetName = this->defaultModelName;
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
    ignerr << "ApplyForceToModel failed to find a link entity for model ["
           << targetName << "]" << std::endl;
    return;
  }

  _ecm.SetComponentData<components::ExternalWorldWrenchCmd>(linkEntity, wrenchCmd);
  std::cerr << "[ApplyForceToModel] applied wrench to model [" << targetName << "]" << std::endl;
}

void ApplyForceToModel::OnWrenchMessage(const ignition::msgs::EntityWrench &_msg)
{
  std::cerr << "[ApplyForceToModel] received wrench message" << std::endl;
  std::cerr << "[ApplyForceToModel] Force: (" << _msg.wrench().force().x() << ", "
            << _msg.wrench().force().y() << ", " << _msg.wrench().force().z() << ")" << std::endl;
  std::cerr << "[ApplyForceToModel] Torque: (" << _msg.wrench().torque().x() << ", "
            << _msg.wrench().torque().y() << ", " << _msg.wrench().torque().z() << ")" << std::endl;
  if (_msg.has_entity())
  {
    std::cerr << "[ApplyForceToModel] Entity Name: " << _msg.entity().name() << std::endl;
    std::cerr << "[ApplyForceToModel] Entity ID: " << _msg.entity().id() << std::endl;
    std::cerr << "[ApplyForceToModel] Entity Type: " << _msg.entity().type() << std::endl;
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
IGNITION_ADD_PLUGIN(ApplyForceToModel,
                    ignition::gazebo::System,
                    ApplyForceToModel::ISystemConfigure,
                    ApplyForceToModel::ISystemUpdate)

IGNITION_ADD_PLUGIN_ALIAS(ApplyForceToModel,
                          "ignition::gazebo::systems::ApplyForceToModel")

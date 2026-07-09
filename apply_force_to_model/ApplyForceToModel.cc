#include <ignition/plugin/Register.hh>
#include "ApplyForceToModel.hh"
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/components/ExternalWorldWrenchCmd.hh>
#include <ignition/gazebo/components/Model.hh>
#include <ignition/gazebo/components/Name.hh>
#include <iostream>

using namespace ignition;
using namespace gazebo;
using namespace systems;

void ApplyForceToModel::Configure(const Entity &_entity,
                                  const std::shared_ptr<const sdf::Element> &_sdf,
                                  EntityComponentManager &_ecm,
                                  EventManager &_eventMgr)
{
  if (_sdf && _sdf->HasElement("wrench_hold_steps"))
    this->wrenchHoldSteps = _sdf->Get<uint32_t>("wrench_hold_steps");

  const std::string topic{"/apply_force_to_model_topic"};
  const bool ok = this->node.Subscribe(topic, &ApplyForceToModel::OnWrenchMessage, this);
  if (!ok)
  {
    ignerr << "ApplyForceToModel failed to subscribe to topic [" << topic << "]" << std::endl;
    return;
  }

  std::cerr << "[ApplyForceToModel] subscribed to topic [" << topic << "]" << std::endl;
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

  if (targetName.empty())
  {
    ignerr << "ApplyForceToModel has a pending wrench without a target model name." << std::endl;
    return;
  }

  const auto targetModelEntity = _ecm.EntityByComponents(components::Model(),
                                                         components::Name(targetName));
  if (targetModelEntity == kNullEntity)
  {
    ignerr << "ApplyForceToModel could not find target model [" << targetName << "]" << std::endl;
    return;
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
    ignerr << "ApplyForceToModel failed to find a link entity for model [" << targetName << "]" << std::endl;
    return;
  }

  _ecm.SetComponentData<components::ExternalWorldWrenchCmd>(linkEntity, wrenchCmd);
  std::cerr << "[ApplyForceToModel] applied wrench to model [" << targetName << "]" << std::endl;
}

void ApplyForceToModel::OnWrenchMessage(const ignition::msgs::EntityWrench &_msg)
{
  if (!_msg.has_entity() || _msg.entity().name().empty())
  {
    ignerr << "ApplyForceToModel received a wrench message without a target model name." << std::endl;
    return;
  }

  {
    std::lock_guard<std::mutex> lock(this->msgMutex);
    this->pendingWrench = _msg.wrench();
    this->pendingTargetName = _msg.entity().name();
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

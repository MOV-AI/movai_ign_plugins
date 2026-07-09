#ifndef APPLY_FORCE_TO_MODEL_HH_
#define APPLY_FORCE_TO_MODEL_HH_

#include <ignition/common/Profiler.hh>
#include <ignition/gazebo/System.hh>
#include <ignition/msgs/entity_wrench.pb.h>
#include <ignition/msgs/wrench.pb.h>
#include <ignition/transport/Node.hh>
#include <mutex>
#include <string>

using namespace ignition;
using namespace gazebo;
using namespace systems;

/// \brief System plugin that applies a received wrench to a named model for a fixed number of updates.
class ApplyForceToModel : public ignition::gazebo::System,
                          public ignition::gazebo::ISystemConfigure,
                          public ignition::gazebo::ISystemUpdate
{
public:
    void Configure(const Entity &_entity,
                   const std::shared_ptr<const sdf::Element> &_sdf,
                   EntityComponentManager &_ecm,
                   EventManager &_eventMgr) override;

public:
    void Update(const ignition::gazebo::UpdateInfo &_info,
                ignition::gazebo::EntityComponentManager &_ecm) override;

private:
    void OnWrenchMessage(const ignition::msgs::EntityWrench &_msg);

private:
    ignition::transport::Node node;

private:
    ignition::msgs::Wrench pendingWrench;

private:
    std::string pendingTargetName;

private:
    bool hasPendingWrench{false};

private:
    uint32_t wrenchHoldSteps{50};

private:
    uint32_t wrenchStepsRemaining{0};

private:
    std::mutex msgMutex;
};

#endif

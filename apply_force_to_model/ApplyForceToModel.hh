#ifndef APPLY_FORCE_TO_MODEL_HH_
#define APPLY_FORCE_TO_MODEL_HH_

#include <ignition/gazebo/System.hh>
#include <ignition/common/Profiler.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/components/Model.hh>
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/World.hh"
#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/transport/Node.hh>
#include <ignition/msgs/entity_wrench.pb.h>
#include <ignition/msgs/wrench.pb.h>
#include <mutex>
#include <string>


using namespace ignition;
using namespace gazebo;
using namespace systems;

/// \brief System plugin that applies the latest received wrench to a model.
class ApplyForceToModel : public ignition::gazebo::System,
    public ignition::gazebo::ISystemConfigure,
    public ignition::gazebo::ISystemUpdate
{
  /// \brief Called once when the plugin is loaded.
  public: void Configure(const Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      EntityComponentManager &_ecm,
      EventManager &_eventMgr) override;

  /// \brief Called every simulation iteration.
  public: void Update(const ignition::gazebo::UpdateInfo &_info,
      ignition::gazebo::EntityComponentManager &_ecm) override;

  /// \brief Callback that stores the latest command from transport.
  private: void OnWrenchMessage(const ignition::msgs::EntityWrench &_msg);

  /// \brief Ignition transport node.
  private: ignition::transport::Node node;

  /// \brief Model entity this plugin is attached to.
  private: Model model{kNullEntity};

  /// \brief Name of the attached model.
  private: std::string defaultModelName;

  /// \brief World name, used in diagnostics.
  private: std::string worldName;

  /// \brief Latest wrench command received from transport callback
  private: ignition::msgs::Wrench pendingWrench;

  /// \brief Optional target name provided by incoming command
  private: std::string pendingTargetName;

  /// \brief Flag indicating a pending wrench command to apply in Update
  private: bool hasPendingWrench{false};

  /// \brief Number of updates to keep applying the latest wrench command
  private: uint32_t wrenchHoldSteps{50};

  /// \brief Remaining updates for current pending wrench command
  private: uint32_t wrenchStepsRemaining{0};

  /// \brief Protects shared callback/update command state
  private: std::mutex msgMutex;

};

#endif

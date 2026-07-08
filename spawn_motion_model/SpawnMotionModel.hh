#ifndef SPAWNMOTIONMODEL_HH_
#define SPAWNMOTIONMODEL_HH_

#include <string>
#include <mutex>
#include <vector>

#include <ignition/common/Profiler.hh>
#include <ignition/gazebo/Entity.hh>
#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/components/Link.hh>
#include <ignition/gazebo/components/ParentEntity.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/msgs/pose_v.pb.h>
#include <ignition/transport/Node.hh>

class SpawnMotionModel : public ignition::gazebo::System,
                         public ignition::gazebo::ISystemConfigure,
                         public ignition::gazebo::ISystemUpdate
{
  public: SpawnMotionModel();

  public: ~SpawnMotionModel() override;

  public: void Configure(const ignition::gazebo::Entity &_entity,
                         const std::shared_ptr<const sdf::Element> &_sdf,
                         ignition::gazebo::EntityComponentManager &_ecm,
                         ignition::gazebo::EventManager &_eventMgr) override;

  public: void Update(const ignition::gazebo::UpdateInfo &_info,
                      ignition::gazebo::EntityComponentManager &_ecm) override;

  private: bool SpawnModel();

  private: bool ReadParameters(const std::shared_ptr<const sdf::Element> &_sdf);

  private: void OnMotionCommand(const ignition::msgs::Pose_V &_msg);

  private: bool ApplyPendingCommand();

  private: void AdvanceAlongPolygon(double _distance);

  private: bool ApplyPose();

  private: bool ResolveSpawnedEntities(ignition::gazebo::EntityComponentManager &_ecm);

  private: bool ApplyVelocityCommand(ignition::gazebo::EntityComponentManager &_ecm);

  private: ignition::math::Pose3d BuildPose() const;

  private: ignition::transport::Node node;

  private: std::string worldName;

  private: std::string topicName;

  private: std::string modelUri;

  private: std::string spawnedModelName;

  private: std::vector<ignition::math::Vector3d> waypoints;

  private: ignition::math::Vector3d currentPosition{0.0, 0.0, 0.0};

  private: std::size_t nextWaypointIndex{0};

  private: double speed{0.5};

  private: double fixedRoll{0.0};

  private: double fixedPitch{0.0};

  private: double startYaw{0.0};

  private: double currentYaw{0.0};

  private: int requestTimeoutMs{100};

  private: bool configured{false};

  private: bool spawned{false};

  private: bool hasPendingCommand{false};

  private: ignition::gazebo::Entity spawnedModelEntity{ignition::gazebo::kNullEntity};

  private: ignition::gazebo::Entity spawnedLinkEntity{ignition::gazebo::kNullEntity};

  private: double waypointTolerance{0.15};

  private: bool linkResolveWarningShown{false};

  private: std::mutex commandMutex;
};

#endif

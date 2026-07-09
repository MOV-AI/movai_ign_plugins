#ifndef SPAWNMOTIONMODEL_HH_
#define SPAWNMOTIONMODEL_HH_

#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

#include <ignition/common/Profiler.hh>
#include <ignition/gazebo/Entity.hh>
#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/System.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/msgs/pose_v.pb.h>
#include <ignition/msgs/stringmsg.pb.h>
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

  private: void OnMotionCommand(const ignition::msgs::Pose_V &_msg);

  private: void OnMotionCommandJson(const ignition::msgs::StringMsg &_msg);

  private: void QueueCommand(const std::string &_modelUri,
                             double _speed,
                             std::vector<ignition::math::Vector3d> &&_waypoints);

  private: bool ApplyPendingCommand();

  private: bool SpawnModel(const std::string &_modelUri,
                           const ignition::math::Vector3d &_position,
                           double _yaw);

  private: bool RemoveSpawnedModel();

  private: bool SetModelPose(const ignition::math::Vector3d &_position,
                             double _yaw);

  private: bool ResolveSpawnedLink(ignition::gazebo::EntityComponentManager &_ecm);

  private: bool ApplyVelocity(ignition::gazebo::EntityComponentManager &_ecm);

  private: ignition::transport::Node node;

  private: std::string worldName;

  private: std::string topicName;

  private: std::string jsonTopicName;

  private: std::string spawnedModelName{"spawn_motion_model"};

  private: std::string requestedModelUri;

  private: std::vector<ignition::math::Vector3d> requestedWaypoints;

  private: double requestedSpeed{0.0};

  private: bool hasPendingCommand{false};

  private: std::string activeModelUri;

  private: std::vector<ignition::math::Vector3d> activeWaypoints;

  private: std::size_t nextWaypointIndex{0};

  private: double activeSpeed{0.0};

  private: bool spawned{false};

  private: uint32_t respawnDelayTicks{0};

  private: ignition::gazebo::Entity spawnedModelEntity{ignition::gazebo::kNullEntity};

  private: ignition::gazebo::Entity spawnedLinkEntity{ignition::gazebo::kNullEntity};

  private: bool linkResolveWarningShown{false};

  private: std::mutex commandMutex;
};

#endif

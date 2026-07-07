#include <ignition/plugin/Register.hh>

#include <chrono>
#include <cmath>

#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/World.hh>
#include <ignition/msgs/Utility.hh>
#include <ignition/msgs/boolean.pb.h>
#include <ignition/msgs/entity_factory.pb.h>
#include <ignition/msgs/pose.pb.h>
#include <ignition/msgs/pose_v.pb.h>

#include "SpawnMotionModel.hh"

using ignition::gazebo::components::Name;
using ignition::gazebo::components::World;

namespace
{
double SegmentYaw(const ignition::math::Vector3d &_from,
                  const ignition::math::Vector3d &_to,
                  double _fallbackYaw)
{
  const auto delta = _to - _from;
  if (delta.Length() < 1e-9)
    return _fallbackYaw;

  return std::atan2(delta.Y(), delta.X());
}

std::string HeaderValue(const ignition::msgs::Header &_header,
                        const std::string &_key)
{
  for (int index = 0; index < _header.data_size(); ++index)
  {
    const auto &entry = _header.data(index);
    if (entry.key() == _key && entry.value_size() > 0)
      return entry.value(0);
  }

  return "";
}

std::string HeaderDebugString(const ignition::msgs::Header &_header)
{
  std::string result;

  for (int index = 0; index < _header.data_size(); ++index)
  {
    const auto &entry = _header.data(index);
    if (!result.empty())
      result += ", ";

    result += entry.key();
    result += "=";

    if (entry.value_size() > 0)
      result += entry.value(0);
    else
      result += "<empty>";
  }

  if (result.empty())
    return "<no header data>";

  return result;
}
}

SpawnMotionModel::SpawnMotionModel() = default;

SpawnMotionModel::~SpawnMotionModel() = default;

void SpawnMotionModel::Configure(const ignition::gazebo::Entity &_entity,
                                 const std::shared_ptr<const sdf::Element> &_sdf,
                                 ignition::gazebo::EntityComponentManager &_ecm,
                                 ignition::gazebo::EventManager & /*_eventMgr*/)
{
  ignmsg << "SpawnMotionModel::Configure" << std::endl;

  ignmsg << "SpawnMotionModel attached to entity [" << _entity << "]" << std::endl;

  const auto worldEntity = _ecm.EntityByComponents(World());
  if (worldEntity == ignition::gazebo::kNullEntity)
  {
    ignerr << "SpawnMotionModel could not find the world entity." << std::endl;
    return;
  }

  const auto *worldNameComp = _ecm.Component<Name>(worldEntity);
  if (worldNameComp == nullptr)
  {
    ignerr << "SpawnMotionModel could not read the world name." << std::endl;
    return;
  }

  this->worldName = worldNameComp->Data();
  ignmsg << "SpawnMotionModel resolved world name [" << this->worldName << "]" << std::endl;
  this->configured = this->ReadParameters(_sdf);

  if (!this->configured)
    return;

  const bool ok = this->node.Subscribe(this->topicName,
                                       &SpawnMotionModel::OnMotionCommand,
                                       this);
  if (!ok)
  {
    ignerr << "SpawnMotionModel failed to subscribe to topic ["
           << this->topicName << "]" << std::endl;
    this->configured = false;
    return;
  }

  ignmsg << "SpawnMotionModel subscribed to topic [" << this->topicName
         << "]" << std::endl;
}

void SpawnMotionModel::Update(const ignition::gazebo::UpdateInfo &_info,
                              ignition::gazebo::EntityComponentManager & /*_ecm*/)
{
  IGN_PROFILE("SpawnMotionModel::Update");

  if (_info.paused || !this->configured)
    return;

  if (this->hasPendingCommand)
  {
    if (!this->ApplyPendingCommand())
      return;
  }

  if (!this->spawned)
    return;

  const double dt = std::chrono::duration<double>(_info.dt).count();
  if (dt <= 0.0 || this->waypoints.size() < 2 || this->speed <= 0.0)
    return;

  this->AdvanceAlongPolygon(this->speed * dt);
  this->ApplyPose();
}

bool SpawnMotionModel::ReadParameters(const std::shared_ptr<const sdf::Element> &_sdf)
{
  this->topicName = "/world/" + this->worldName + "/spawn_motion_model";
  this->spawnedModelName = "spawn_motion_model";

  if (_sdf && _sdf->HasElement("roll"))
    this->fixedRoll = _sdf->Get<double>("roll");

  if (_sdf && _sdf->HasElement("pitch"))
    this->fixedPitch = _sdf->Get<double>("pitch");

  if (_sdf && _sdf->HasElement("request_timeout_ms"))
    this->requestTimeoutMs = _sdf->Get<int>("request_timeout_ms");

  ignmsg << "SpawnMotionModel is waiting for motion commands on ["
         << this->topicName << "]" << std::endl;
  return true;
}

void SpawnMotionModel::OnMotionCommand(const ignition::msgs::Pose_V &_msg)
{
  const std::string modelName = HeaderValue(_msg.header(), "model_name");
  const std::string velocityValue = HeaderValue(_msg.header(), "velocity");

  ignmsg << "SpawnMotionModel received topic message with " << _msg.pose_size()
         << " poses and headers [" << HeaderDebugString(_msg.header())
         << "]" << std::endl;

  if (modelName.empty())
  {
    ignerr << "SpawnMotionModel command is missing header field [model_name]. "
           << "Expected header data like key=model_name value=<model_name>."
           << std::endl;
    return;
  }

  if (_msg.pose_size() < 2)
  {
    ignerr << "SpawnMotionModel command needs at least two poses." << std::endl;
    return;
  }

  double velocity = 0.0;
  try
  {
    velocity = std::stod(velocityValue);
  }
  catch (const std::exception &)
  {
    ignerr << "SpawnMotionModel command has invalid header field [velocity]. "
           << "Received value [" << velocityValue << "]" << std::endl;
    return;
  }

  if (velocity < 0.0)
  {
    ignerr << "SpawnMotionModel command requires non-negative velocity." << std::endl;
    return;
  }

  std::vector<ignition::math::Vector3d> receivedWaypoints;
  receivedWaypoints.reserve(_msg.pose_size());
  for (int index = 0; index < _msg.pose_size(); ++index)
  {
    receivedWaypoints.emplace_back(
        _msg.pose(index).position().x(),
        _msg.pose(index).position().y(),
        _msg.pose(index).position().z());
  }

  {
    std::lock_guard<std::mutex> lock(this->commandMutex);
    this->modelUri = modelName;
    this->speed = velocity;
    this->waypoints = std::move(receivedWaypoints);
    this->currentPosition = this->waypoints.front();
    this->nextWaypointIndex = 1;
    this->currentYaw = SegmentYaw(this->waypoints[0], this->waypoints[1], this->startYaw);
    this->hasPendingCommand = true;
  }

  ignmsg << "SpawnMotionModel received motion command for model [" << modelName
         << "] with " << _msg.pose_size() << " waypoints and speed "
         << velocity << " m/s" << std::endl;
}

bool SpawnMotionModel::ApplyPendingCommand()
{
  std::lock_guard<std::mutex> lock(this->commandMutex);

  if (!this->hasPendingCommand)
    return true;

  if (!this->spawned)
  {
    ignmsg << "SpawnMotionModel applying first command by spawning model://"
           << this->modelUri << std::endl;
    this->spawned = this->SpawnModel();
    if (!this->spawned)
      return false;
  }
  else if (!this->ApplyPose())
  {
    ignerr << "SpawnMotionModel failed to reset pose for model ["
           << this->spawnedModelName << "]" << std::endl;
    return false;
  }

  this->hasPendingCommand = false;
  ignmsg << "SpawnMotionModel accepted command and reset path start to ["
         << this->currentPosition << "]" << std::endl;
  return true;
}

bool SpawnMotionModel::SpawnModel()
{
  ignition::msgs::EntityFactory request;
  ignition::msgs::Boolean response;
  bool result = false;

  const std::string sdf =
      "<sdf version='1.7'>"
      "  <model name='" + this->spawnedModelName + "'>"
      "    <include>"
      "      <uri>model://" + this->modelUri + "</uri>"
      "    </include>"
      "  </model>"
      "</sdf>";

  ignition::msgs::Set(request.mutable_pose(), this->BuildPose());
  request.set_sdf(sdf);

  const std::string service = "/world/" + this->worldName + "/create";
  this->node.Request(service, request, this->requestTimeoutMs, response, result);

  if (!result || !response.data())
  {
    ignerr << "SpawnMotionModel failed to spawn model [" << this->spawnedModelName
           << "] from [model://" << this->modelUri << "] using service ["
           << service << "]" << std::endl;
    return false;
  }

  ignmsg << "SpawnMotionModel spawned model [" << this->spawnedModelName << "]" << std::endl;
  return true;
}

void SpawnMotionModel::AdvanceAlongPolygon(double _distance)
{
  double remainingDistance = _distance;

  while (remainingDistance > 0.0)
  {
    const auto &target = this->waypoints[this->nextWaypointIndex];
    const auto segment = target - this->currentPosition;
    const double segmentLength = segment.Length();

    if (segmentLength < 1e-9)
    {
      this->currentPosition = target;
      this->nextWaypointIndex = (this->nextWaypointIndex + 1) % this->waypoints.size();
      continue;
    }

    this->currentYaw = SegmentYaw(this->currentPosition, target, this->currentYaw);

    if (remainingDistance < segmentLength)
    {
      this->currentPosition += segment.Normalized() * remainingDistance;
      return;
    }

    this->currentPosition = target;
    remainingDistance -= segmentLength;
    this->nextWaypointIndex = (this->nextWaypointIndex + 1) % this->waypoints.size();
  }
}

bool SpawnMotionModel::ApplyPose()
{
  ignition::msgs::Pose request;
  ignition::msgs::Boolean response;
  bool result = false;

  request.set_name(this->spawnedModelName);
  ignition::msgs::Set(request.mutable_position(), this->currentPosition);
  ignition::msgs::Set(
      request.mutable_orientation(),
      ignition::math::Quaterniond(this->fixedRoll, this->fixedPitch, this->currentYaw));

  const std::string service = "/world/" + this->worldName + "/set_pose";
  this->node.Request(service, request, this->requestTimeoutMs, response, result);
  return result && response.data();
}

ignition::math::Pose3d SpawnMotionModel::BuildPose() const
{
  return ignition::math::Pose3d(
      this->currentPosition,
      ignition::math::Quaterniond(this->fixedRoll, this->fixedPitch, this->currentYaw));
}

IGNITION_ADD_PLUGIN(SpawnMotionModel,
                    ignition::gazebo::System,
                    SpawnMotionModel::ISystemConfigure,
                    SpawnMotionModel::ISystemUpdate)

IGNITION_ADD_PLUGIN_ALIAS(SpawnMotionModel,
                          "ignition::gazebo::systems::SpawnMotionModel")

#include <ignition/plugin/Register.hh>

#include <cmath>

#include <ignition/gazebo/components/LinearVelocityCmd.hh>
#include <ignition/gazebo/components/Link.hh>
#include <ignition/gazebo/components/Model.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/ParentEntity.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/components/World.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/msgs/Utility.hh>
#include <ignition/msgs/boolean.pb.h>
#include <ignition/msgs/entity.pb.h>
#include <ignition/msgs/entity_factory.pb.h>
#include <ignition/msgs/pose.pb.h>
#include <ignition/msgs/pose_v.pb.h>
#include <ignition/msgs/stringmsg.pb.h>
#include <nlohmann/json.hpp>

#include "SpawnMotionModel.hh"

using ignition::gazebo::components::Name;
using ignition::gazebo::components::World;
using json = nlohmann::json;

namespace
{
  constexpr int kRequestTimeoutMs = 100;
  constexpr double kWaypointTolerance = 0.15;

  double HeadingYaw(const ignition::math::Vector3d &_from,
                    const ignition::math::Vector3d &_to,
                    double _fallbackYaw = 0.0)
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

  bool ParseJsonCommand(const std::string &_json,
                        std::string &_modelUri,
                        double &_speed,
                        std::vector<ignition::math::Vector3d> &_waypoints,
                        std::string &_error)
  {
    static constexpr const char *kModelNameField = "model_name";
    static constexpr const char *kVelocityField = "velocity";
    static constexpr const char *kPosesField = "poses";

    const auto fail = [&_error](const std::string &_message)
    {
      _error = _message;
      return false;
    };

    json root;
    try
    {
      root = json::parse(_json);
    }
    catch (const std::exception &e)
    {
      return fail(std::string("invalid JSON: ") + e.what());
    }

    const auto modelNameIt = root.find(kModelNameField);
    if (modelNameIt == root.end() || !modelNameIt->is_string())
      return fail("missing field [model_name]");

    _modelUri = modelNameIt->get<std::string>();
    if (_modelUri.empty())
      return fail("missing field [model_name]");

    const auto velocityIt = root.find(kVelocityField);
    if (velocityIt == root.end() || !velocityIt->is_number())
      return fail("missing or invalid field [velocity]");

    _speed = velocityIt->get<double>();

    const auto posesIt = root.find(kPosesField);
    if (posesIt == root.end() || !posesIt->is_array())
      return fail("missing field [poses]");

    _waypoints.clear();
    for (const auto &pose : *posesIt)
    {
      if (!pose.is_array() || pose.size() != 3 ||
          !pose[0].is_number() || !pose[1].is_number() || !pose[2].is_number())
        return fail("invalid pose entry in [poses]; expected [x, y, z] numbers");

      _waypoints.emplace_back(pose[0].get<double>(), pose[1].get<double>(), pose[2].get<double>());
    }

    if (_waypoints.size() < 2)
      return fail("[poses] must contain at least two [x, y, z] entries");

    return true;
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
  this->topicName = "/world/" + this->worldName + "/spawn_motion_model";
  this->jsonTopicName = "/world/" + this->worldName + "/spawn_motion_model_json";
  if (_sdf != nullptr)
  {
    ignmsg << "SpawnMotionModel ignores SDF parameters and uses its built-in topic and request timeout."
           << std::endl;
  }

  const bool ok = this->node.Subscribe(this->topicName,
                                       &SpawnMotionModel::OnMotionCommand,
                                       this);
  if (!ok)
  {
    ignerr << "SpawnMotionModel failed to subscribe to topic ["
           << this->topicName << "]" << std::endl;
    return;
  }

  const bool jsonOk = this->node.Subscribe(this->jsonTopicName,
                                           &SpawnMotionModel::OnMotionCommandJson,
                                           this);
  if (!jsonOk)
  {
    ignerr << "SpawnMotionModel failed to subscribe to JSON topic ["
           << this->jsonTopicName << "]" << std::endl;
  }

  ignmsg << "SpawnMotionModel subscribed to topic [" << this->topicName
         << "]" << std::endl;
  ignmsg << "SpawnMotionModel subscribed to JSON topic [" << this->jsonTopicName
         << "]" << std::endl;
}

void SpawnMotionModel::Update(const ignition::gazebo::UpdateInfo &_info,
                              ignition::gazebo::EntityComponentManager &_ecm)
{
  IGN_PROFILE("SpawnMotionModel::Update");

  if (_info.paused)
    return;

  if (this->hasPendingCommand)
  {
    if (!this->ApplyPendingCommand())
      return;
  }

  if (!this->spawned)
    return;

  if (!this->ResolveSpawnedLink(_ecm))
    return;

  if (this->activeWaypoints.size() < 2 || this->activeSpeed <= 0.0)
    return;

  this->ApplyVelocity(_ecm);
}

void SpawnMotionModel::OnMotionCommand(const ignition::msgs::Pose_V &_msg)
{
  const std::string modelName = HeaderValue(_msg.header(), "model_name");
  const std::string velocityValue = HeaderValue(_msg.header(), "velocity");

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

  this->QueueCommand(modelName, velocity, std::move(receivedWaypoints));

  ignmsg << "SpawnMotionModel received motion command for model [" << modelName
         << "] with " << _msg.pose_size() << " waypoints and speed "
         << velocity << " m/s" << std::endl;
}

void SpawnMotionModel::OnMotionCommandJson(const ignition::msgs::StringMsg &_msg)
{
  std::string modelName;
  double velocity = 0.0;
  std::vector<ignition::math::Vector3d> waypoints;
  std::string error;

  if (!ParseJsonCommand(_msg.data(), modelName, velocity, waypoints, error))
  {
    ignerr << "SpawnMotionModel invalid JSON command: " << error << std::endl;
    return;
  }

  if (velocity < 0.0)
  {
    ignerr << "SpawnMotionModel JSON command requires non-negative velocity." << std::endl;
    return;
  }

  this->QueueCommand(modelName, velocity, std::move(waypoints));

  ignmsg << "SpawnMotionModel received JSON command for model [" << modelName
         << "] with speed " << velocity << " m/s" << std::endl;
}

void SpawnMotionModel::QueueCommand(const std::string &_modelUri,
                                    double _speed,
                                    std::vector<ignition::math::Vector3d> &&_waypoints)
{
  std::lock_guard<std::mutex> lock(this->commandMutex);
  this->requestedModelUri = _modelUri;
  this->requestedSpeed = _speed;
  this->requestedWaypoints = std::move(_waypoints);
  this->hasPendingCommand = true;
}

bool SpawnMotionModel::ApplyPendingCommand()
{
  std::lock_guard<std::mutex> lock(this->commandMutex);

  if (!this->hasPendingCommand)
    return true;

  if (this->respawnDelayTicks > 0)
  {
    --this->respawnDelayTicks;
    return false;
  }

  if (this->requestedWaypoints.size() < 2)
  {
    this->hasPendingCommand = false;
    return false;
  }

  const auto startPosition = this->requestedWaypoints.front();
  const double startYaw = HeadingYaw(this->requestedWaypoints[0], this->requestedWaypoints[1]);

  if (!this->spawned)
  {
    ignmsg << "SpawnMotionModel applying first command by spawning model://"
           << this->requestedModelUri << std::endl;
    this->spawned = this->SpawnModel(this->requestedModelUri, startPosition, startYaw);
    if (!this->spawned)
      return false;
  }
  else if (this->requestedModelUri != this->activeModelUri)
  {
    ignmsg << "SpawnMotionModel received different model [model://" << this->requestedModelUri
           << "] and will replace current model [model://"
           << this->activeModelUri << "]" << std::endl;

    if (!this->RemoveSpawnedModel())
      return false;

    this->spawned = false;
    this->spawnedModelEntity = ignition::gazebo::kNullEntity;
    this->spawnedLinkEntity = ignition::gazebo::kNullEntity;
    this->linkResolveWarningShown = false;
    this->respawnDelayTicks = 1;
    return false;
  }
  else if (!this->SetModelPose(startPosition, startYaw))
  {
    ignerr << "SpawnMotionModel failed to reset pose for model ["
           << this->spawnedModelName << "]" << std::endl;
    return false;
  }

  this->activeModelUri = this->requestedModelUri;
  this->activeWaypoints = this->requestedWaypoints;
  this->activeSpeed = this->requestedSpeed;
  this->nextWaypointIndex = 1;

  this->hasPendingCommand = false;
  this->spawnedModelEntity = ignition::gazebo::kNullEntity;
  this->spawnedLinkEntity = ignition::gazebo::kNullEntity;
  ignmsg << "SpawnMotionModel accepted command and reset path start to ["
         << startPosition << "]" << std::endl;
  return true;
}

bool SpawnMotionModel::SpawnModel(const std::string &_modelUri,
                                  const ignition::math::Vector3d &_position,
                                  double _yaw)
{
  ignition::msgs::EntityFactory request;
  ignition::msgs::Boolean response;
  bool result = false;

  const std::string sdf =
      "<sdf version='1.7'>"
      "  <model name='" +
      this->spawnedModelName + "'>"
                               "    <static>false</static>"
                               "    <self_collide>false</self_collide>"
                               "    <allow_auto_disable>false</allow_auto_disable>"
                               "    <include>"
                               "      <uri>model://" +
      _modelUri + "</uri>"
                  "    </include>"
                  "  </model>"
                  "</sdf>";

  auto *poseMsg = request.mutable_pose();
  ignition::msgs::Set(poseMsg->mutable_position(), _position);
  ignition::msgs::Set(poseMsg->mutable_orientation(), ignition::math::Quaterniond(0.0, 0.0, _yaw));
  request.set_sdf(sdf);

  const std::string service = "/world/" + this->worldName + "/create";
  this->node.Request(service, request, kRequestTimeoutMs, response, result);

  if (!result || !response.data())
  {
    ignerr << "SpawnMotionModel failed to spawn model [" << this->spawnedModelName
           << "] from [model://" << _modelUri << "] using service ["
           << service << "]" << std::endl;
    return false;
  }

  ignmsg << "SpawnMotionModel spawned model [" << this->spawnedModelName << "]" << std::endl;
  return true;
}

bool SpawnMotionModel::RemoveSpawnedModel()
{
  ignition::msgs::Entity request;
  ignition::msgs::Boolean response;
  bool result = false;

  request.set_name(this->spawnedModelName);
  request.set_type(ignition::msgs::Entity::MODEL);

  const std::string service = "/world/" + this->worldName + "/remove";
  this->node.Request(service, request, kRequestTimeoutMs, response, result);

  if (!result || !response.data())
  {
    ignerr << "SpawnMotionModel failed to remove model [" << this->spawnedModelName
           << "] using service [" << service << "]" << std::endl;
    return false;
  }

  ignmsg << "SpawnMotionModel removed model [" << this->spawnedModelName << "]"
         << std::endl;
  return true;
}

bool SpawnMotionModel::SetModelPose(const ignition::math::Vector3d &_position,
                                    double _yaw)
{
  ignition::msgs::Pose request;
  ignition::msgs::Boolean response;
  bool result = false;

  request.set_name(this->spawnedModelName);
  ignition::msgs::Set(request.mutable_position(), _position);
  ignition::msgs::Set(request.mutable_orientation(), ignition::math::Quaterniond(0.0, 0.0, _yaw));

  const std::string service = "/world/" + this->worldName + "/set_pose";
  this->node.Request(service, request, kRequestTimeoutMs, response, result);
  return result && response.data();
}

bool SpawnMotionModel::ResolveSpawnedLink(ignition::gazebo::EntityComponentManager &_ecm)
{
  if (this->spawnedModelEntity == ignition::gazebo::kNullEntity)
  {
    this->spawnedModelEntity = _ecm.EntityByComponents(
        ignition::gazebo::components::Model(),
        ignition::gazebo::components::Name(this->spawnedModelName));

    if (this->spawnedModelEntity == ignition::gazebo::kNullEntity)
    {
      ignerr << "SpawnMotionModel could not resolve spawned model entity ["
             << this->spawnedModelName << "]" << std::endl;
      return false;
    }
  }

  if (this->spawnedLinkEntity == ignition::gazebo::kNullEntity)
  {
    ignition::gazebo::Model model(this->spawnedModelEntity);
    this->spawnedLinkEntity = model.CanonicalLink(_ecm);
    if (this->spawnedLinkEntity == ignition::gazebo::kNullEntity)
    {
      auto links = model.Links(_ecm);
      if (!links.empty())
        this->spawnedLinkEntity = links.front();
    }

    if (this->spawnedLinkEntity == ignition::gazebo::kNullEntity)
    {
      std::vector<ignition::gazebo::Entity> modelsToVisit{this->spawnedModelEntity};
      std::size_t index = 0;
      while (index < modelsToVisit.size() &&
             this->spawnedLinkEntity == ignition::gazebo::kNullEntity)
      {
        const auto modelEntity = modelsToVisit[index++];

        const auto childLinks = _ecm.EntitiesByComponents(
            ignition::gazebo::components::ParentEntity(modelEntity),
            ignition::gazebo::components::Link());
        if (!childLinks.empty())
        {
          this->spawnedLinkEntity = childLinks.front();
          break;
        }

        const auto childModels = _ecm.EntitiesByComponents(
            ignition::gazebo::components::ParentEntity(modelEntity),
            ignition::gazebo::components::Model());
        for (const auto childModel : childModels)
          modelsToVisit.push_back(childModel);
      }
    }

    if (this->spawnedLinkEntity == ignition::gazebo::kNullEntity)
    {
      if (!this->linkResolveWarningShown)
      {
        ignerr << "SpawnMotionModel could not resolve a link for model ["
               << this->spawnedModelName << "]" << std::endl;
        this->linkResolveWarningShown = true;
      }
      return false;
    }

    this->linkResolveWarningShown = false;

    ignmsg << "SpawnMotionModel resolved link entity ["
           << this->spawnedLinkEntity << "] for model ["
           << this->spawnedModelName << "]" << std::endl;
  }

  return true;
}

bool SpawnMotionModel::ApplyVelocity(ignition::gazebo::EntityComponentManager &_ecm)
{
  const auto *poseComp = _ecm.Component<ignition::gazebo::components::Pose>(
      this->spawnedModelEntity);
  if (poseComp == nullptr)
    return false;

  const ignition::math::Vector3d current = poseComp->Data().Pos();
  if (this->nextWaypointIndex >= this->activeWaypoints.size())
    this->nextWaypointIndex = 0;

  const auto &target = this->activeWaypoints[this->nextWaypointIndex];

  auto toTarget = target - current;
  if (toTarget.Length() <= kWaypointTolerance)
  {
    this->nextWaypointIndex = (this->nextWaypointIndex + 1) % this->activeWaypoints.size();
    toTarget = this->activeWaypoints[this->nextWaypointIndex] - current;
  }

  if (toTarget.Length() < 1e-9)
    return true;

  const ignition::math::Vector3d velocityCmd = toTarget.Normalized() * this->activeSpeed;

  auto *velComp = _ecm.Component<ignition::gazebo::components::LinearVelocityCmd>(
      this->spawnedLinkEntity);
  if (velComp == nullptr)
  {
    _ecm.CreateComponent(
        this->spawnedLinkEntity,
        ignition::gazebo::components::LinearVelocityCmd(velocityCmd));
  }
  else
  {
    _ecm.SetComponentData<ignition::gazebo::components::LinearVelocityCmd>(
        this->spawnedLinkEntity,
        velocityCmd);
  }

  return true;
}

IGNITION_ADD_PLUGIN(SpawnMotionModel,
                    ignition::gazebo::System,
                    SpawnMotionModel::ISystemConfigure,
                    SpawnMotionModel::ISystemUpdate)

IGNITION_ADD_PLUGIN_ALIAS(SpawnMotionModel,
                          "ignition::gazebo::systems::SpawnMotionModel")

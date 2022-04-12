#include "MaterialController.hh"
#include <string>
#include <vector>

#include <ignition/common/Profiler.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>

#include "ignition/gazebo/components/Material.hh"
#include "ignition/gazebo/components/Visual.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/Geometry.hh"
#include "ignition/gazebo/components/Transparency.hh"
#include "ignition/common/Material.hh"
#include "ignition/gazebo/Model.hh"
#include "ignition/math/Color.hh"
#include "ignition/gazebo/Util.hh"

#include <ignition/gazebo/components/Geometry.hh>
#include <ignition/gazebo/components/Light.hh>
#include <ignition/gazebo/components/Link.hh>
#include <ignition/gazebo/components/Material.hh>
#include <ignition/gazebo/components/Model.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/ParentEntity.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/components/Transparency.hh>
#include <ignition/gazebo/components/Visual.hh>
#include <ignition/gazebo/components/World.hh>
#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/Util.hh>

#include <sdf/Light.hh>
#include <sdf/Mesh.hh>
#include <sdf/Model.hh>
#include <sdf/Visual.hh>

#include <ignition/common/Material.hh>
#include <ignition/common/MeshManager.hh>
#include <ignition/common/Mesh.hh>
#include <ignition/common/SubMesh.hh>

#include <ignition/math/Matrix4.hh>

using namespace ignition;
using namespace gazebo;
using namespace systems;

class ignition::gazebo::systems::MaterialControllerPrivate
{
  /// \brief Callback for joint force subscription
  /// \param[in] _msg Joint force message
  public: void OnChangeCollor(const ignition::msgs::Color &_msg);

  /// \brief Ignition communication node.
  public: transport::Node node;

  /// \brief Joint Entity
  public: Entity linkEntity;

  /// \brief Joint name
  public: std::string linkName;

  /// \brief Commanded joint force
  public: double jointForceCmd;

  /// \brief mutex to protect jointForceCmd
  public: std::mutex jointForceCmdMutex;

  /// \brief Model interface
  public: Model model{kNullEntity};

  /// \brief color to apply
  public: ignition::math::Color newEmissive;
};

//////////////////////////////////////////////////
MaterialController::MaterialController()
  : dataPtr(std::make_unique<MaterialControllerPrivate>())
{
}

//////////////////////////////////////////////////
void MaterialController::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &/*_eventMgr*/)
{
  this->dataPtr->model = Model(_entity);

  if (!this->dataPtr->model.Valid(_ecm))
  {
    ignerr << "MaterialController plugin should be attached to a model entity. "
           << "Failed to initialize." << std::endl;
    return;
  }

  auto sdfClone = _sdf->Clone();

  // Get params from SDF
  auto sdfElem = sdfClone->GetElement("link_name");
  if (sdfElem)
  {
    this->dataPtr->linkName = sdfElem->Get<std::string>();
  }

  if (this->dataPtr->linkName == "")
  {
    ignerr << "MaterialController found an empty linkName parameter. "
           << "Failed to initialize.";
    return;
  }

  ignerr << "Link name: " << this->dataPtr->linkName << std::endl;

  // Subscribe to commands
  auto topic = transport::TopicUtils::AsValidTopic("/model/" +
      this->dataPtr->model.Name(_ecm) + "/link/" + this->dataPtr->linkName +
      "/color");
  if (topic.empty())
  {
    ignerr << "Failed to create valid topic for [" << this->dataPtr->linkName
           << "]" << std::endl;
    return;
  }
  this->dataPtr->node.Subscribe(topic, &MaterialControllerPrivate::OnChangeCollor,
                                this->dataPtr.get());

  ignmsg << "MaterialController subscribing to Double messages on [" << topic
         << "]" << std::endl;
  std::string topicName = "/model/" + this->dataPtr->model.Name(_ecm) + "/link/" + this->dataPtr->linkName + "/color";
  ignerr << "Subscribing to" << topicName << std::endl;
}

//////////////////////////////////////////////////
void MaterialController::PreUpdate(const ignition::gazebo::UpdateInfo &_info,
    ignition::gazebo::EntityComponentManager &_ecm)
{
  IGN_PROFILE("MaterialController::PreUpdate");

  // If the joint hasn't been identified yet, look for it
  if (this->dataPtr->linkEntity == kNullEntity)
  {
    this->dataPtr->linkEntity =
        this->dataPtr->model.LinkByName(_ecm, this->dataPtr->linkName);
  }

  if (this->dataPtr->linkEntity == kNullEntity)
    return;

  // Nothing left to do if paused.
  if (_info.paused)
    return;
  
  auto visual = _ecm.EntityByComponents(
      components::ParentEntity(this->dataPtr->linkEntity),
      components::Visual());

  auto material = _ecm.Component<components::Material>(visual);
  
  material->Data().SetEmissive(this->dataPtr->newEmissive);
}

//////////////////////////////////////////////////
void MaterialControllerPrivate::OnChangeCollor(const msgs::Color &_msg)
{
  ignerr << "Received Color R: " << _msg.r() << std::endl;
  ignerr << "Received Color G: " << _msg.g() << std::endl;
  ignerr << "Received Color B: " << _msg.b() << std::endl;
  ignerr << "Received Color A: " << _msg.a() << std::endl;
  this->dataPtr->newEmissive.R() = _msg.r();
  this->dataPtr->newEmissive.G() = _msg.g();
  this->dataPtr->newEmissive.B() = _msg.b();
  this->dataPtr->newEmissive.A() = _msg.a();
}

IGNITION_ADD_PLUGIN(MaterialController,
                    ignition::gazebo::System,
                    MaterialController::ISystemConfigure,
                    MaterialController::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(MaterialController,
                          "ignition::gazebo::systems::MaterialController")

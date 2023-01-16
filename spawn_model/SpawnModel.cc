#include <ignition/plugin/Register.hh>

#include "SpawnModel.hh"

using namespace ignition;
using namespace gazebo;
using namespace systems;

/////////////////////////////////////////////////
/// \brief Constructor
SpawnModel::SpawnModel() = default;

/////////////////////////////////////////////////
/// \brief Destructor
SpawnModel::~SpawnModel() = default;

//////////////////////////////////////////////////
/// \brief This function is called when the model attached is loaded in the simulation
/// \param[in] _entity Object model that this plugin is attached 
/// \param[in] _sdf SDF element of the plugin in the model attached 
/// \param[in] _ecm Entity Component Manager
void SpawnModel::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &/*_eventMgr*/)
{
  // Get the model and check if it is valid
  this->model = Model(_entity);
  if (!this->model.Valid(_ecm))
  {
    ignerr << "Spawn Model plugin should be attached to a model entity. "
           << "Failed to initialize." << std::endl;
    return;
  }
  
  // Get the world name in the simulation
  auto worldEntity = _ecm.EntityByComponents(components::World());
  if (worldEntity == kNullEntity)
    return;

  // Get the components name to create the spawn topic subscriber
  std::string objectName = _ecm.Component<components::Name>(_entity)->Data();
  this->worldName = _ecm.Component<components::Name>(worldEntity)->Data();
  //Change name of the topic to subscribe to
  std::string topic = "/world/" + worldName + "/create";

  // Subscribe to the spawn topic
  this->node.Subscribe(topic, &SpawnModel::OnSpawnCmd,
                                this);

}

//////////////////////////////////////////////////
/// \brief This function is called in each simulation step update.
/// \param[in] _info Simulation state information
/// \param[in] _ecm Entity Component Manager
void SpawnModel::Update(const ignition::gazebo::UpdateInfo &_info,
                            ignition::gazebo::EntityComponentManager &_ecm)
{
  IGN_PROFILE("SpawnModel::Update");

  // Nothing left to do if paused.
  if (_info.paused)
    return;

    bool result;
    ignition::msgs::EntityFactory req;
    ignition::msgs::Boolean res;
    
    int timeout = 3;
    std::string reqStr = std::string("<sdf version='1.7'>") +
    "<model name='" + this->objectName +"_" + std::to_string(this->nrObjectsSpawned) + "'> " +
      "<include>"
        "<uri>model://"+ this->objectName+ "</uri>"+
        "<pose>" +
        std::to_string(this->objectPosition.x()) + " " +
        std::to_string(this->objectPosition.y()) + " " +
        std::to_string(this->objectPosition.z()) + " " +
        std::to_string(this->objectOrientation.x()) + " " +
        std::to_string(this->objectOrientation.y()) + " " +
        std::to_string(this->objectOrientation.z()) +
        "</pose>" +
      "</include>"+
    "</model>"+
  "</sdf>";

    // TODO: transform quaternion to euler
    req.set_sdf(reqStr);
    if (this->spawnObject)
    {
      // Create the spawn service topic using the message informations and call the service to spawn a model
      const std::string srvCreate = "/world/" + this->worldName + "/create";
      //  ignwarn << "Created " << reqStr << std::endl;
      this->node.Request(srvCreate, req,timeout,res,result);
      ignmsg << "Created Object" << std::endl;
    }
    this->spawnObject = false;
    
}

// //////////////////////////////////////////////////
/// \brief Callback for PoseArray message subscription
/// \param[in] _msg Message
void SpawnModel::OnSpawnCmd(const ignition::msgs::Pose_V &_msg)
{   
     std::string j = _msg.header().data(1).value(0);
    ignwarn << j << std::endl;

    if (j != ""){
        this->objectName = j;
        this->objectPosition = _msg.pose(0).position();
         ignwarn << this->objectPosition.x()<< std::endl;
         ignwarn << this->objectPosition.y()<< std::endl;
         ignwarn << this->objectPosition.z()<< std::endl;
        this->objectOrientation = _msg.pose(0).orientation();
         ignwarn << this->objectOrientation.x()<< std::endl;
         ignwarn << this->objectOrientation.y()<< std::endl;
         ignwarn << this->objectOrientation.z()<< std::endl;
        this->spawnObject = true; 
        this->nrObjectsSpawned ++;
    }
}

// Register this plugin
IGNITION_ADD_PLUGIN(SpawnModel,
                    ignition::gazebo::System,
                    SpawnModel::ISystemConfigure,
                    SpawnModel::ISystemUpdate)

IGNITION_ADD_PLUGIN_ALIAS(SpawnModel,
                          "ignition::gazebo::systems::SpawnModel")


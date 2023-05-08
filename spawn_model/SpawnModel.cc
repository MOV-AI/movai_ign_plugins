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

  // Get the component's name to create the spawn topic subscriber
  std::string objectName = _ecm.Component<components::Name>(_entity)->Data();
  this->worldName = _ecm.Component<components::Name>(worldEntity)->Data();
  //Create topics to subscribe to 
  // - "create" -> PoseArray message to get the pose of each object to spawn
  // - "model_array" -> name of the models to spawn
  std::string topic = "/world/" + worldName + "/create";
  std::string topic2 = "/world/" + worldName + "/model_array";

  // Subscribe to the spawn topics
  this->node.Subscribe(topic, &SpawnModel::OnSpawnCmd,
                                this);
  this->node.Subscribe(topic2, &SpawnModel::OnModelArray,
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
    // Create service variables
    ignition::msgs::EntityFactory req;
    ignition::msgs::Boolean res;
    const std::string srvCreate = "/world/" + this->worldName + "/create";
    int timeout = 100;

    //Create lis of models in the simulation world
    if (this->string_ready && this->spawnObject){
    
      for(int i = 0; i<this->objectPosition.size();i++){
        std::string reqStr = std::string("<sdf version='1.7'>") +
        "<model name='" + this->model_name[i] +"_" + std::to_string(this->nrObjectsSpawned) + "'> " +
          "<include>"
            "<uri>model://"+ this->model_name[i]+ "</uri>"+
            "<pose>" +
            std::to_string(this->objectPosition[i].x()) + " " +
            std::to_string(this->objectPosition[i].y()) + " " +
            std::to_string(this->objectPosition[i].z()) + " " +
            std::to_string(this->objectOrientation[i].x()) + " " +
            std::to_string(this->objectOrientation[i].y()) + " " +
            std::to_string(this->objectOrientation[i].z()) +
            "</pose>" +
          "</include>"+
        "</model>"+
      "</sdf>";
       req.set_sdf(reqStr);
       this->node.Request(srvCreate, req,timeout,res,result);

       //Make each object have a unique identifier
       this->nrObjectsSpawned ++;

      }
      // Stop the Update function from spawning duplicates
      this->spawnObject = false;
      this->string_ready = false;


      // Clear variables containing the names and poses of the models spawned
      if(!this->model_name.empty()){
        this->objectPosition.clear();
        this->objectOrientation.clear();
        this->model_name.clear();
      }
      }

}


// //////////////////////////////////////////////////
/// \brief Callback for model PoseArray message subscription
/// \param[in] _msg Message
void SpawnModel::OnSpawnCmd(const ignition::msgs::Pose_V &_msg)
{   
  // Append model poses to class variables
      for(int i = 0; i<_msg.pose().size();i++){
        this->objectPosition.push_back(_msg.pose(i).position());
        this->objectOrientation.push_back(_msg.pose(i).orientation());
      }
         this->spawnObject = true; 
}

// //////////////////////////////////////////////////
/// \brief Callback for model names message subscription
/// \param[in] _msg Message
void SpawnModel::OnModelArray(const ignition::msgs::StringMsg &_msg)
{   
    // Check if the received data is not empty
    if (!_msg.data().empty()){

      //Create a vector with model names
      size_t pos = 0;
      std::string s = _msg.data();
      while((pos = s.find(this->delimiter)) != std::string::npos){
        this->model_name.push_back(s.substr(0,pos)) ;        
        s.erase(0,pos+1);
      }
      this-> string_ready = true;
    }
}
// Register this plugin
IGNITION_ADD_PLUGIN(SpawnModel,
                    ignition::gazebo::System,
                    SpawnModel::ISystemConfigure,
                    SpawnModel::ISystemUpdate)

IGNITION_ADD_PLUGIN_ALIAS(SpawnModel,
                          "ignition::gazebo::systems::SpawnModel")


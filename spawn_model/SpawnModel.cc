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
  std::string spawn_topic = "/world/" + worldName + "/create";
  std::string array_topic = "/world/" + worldName + "/model_array";
  std::string move_model_name_topic = "/world/" + worldName + "/move_model_name";
  std::string move_topic = "/world/" + worldName + "/move_model";
  std::string req_model_pose_topic = "/world/" + worldName + "/req_model_pose";
  std::string res_model_pose_topic = "/world/" + worldName + "/res_model_pose";

  // Subscribe to the spawn topics
  this->node.Subscribe(spawn_topic, &SpawnModel::OnSpawnCmd,
                                this);
  this->node.Subscribe(array_topic, &SpawnModel::OnModelArray,
                                this);
  this->node.Subscribe(move_topic, &SpawnModel::OnMoveCmd,
                                this);
  // Subscribe to the moving models related topics
  this->node.Subscribe(move_model_name_topic, &SpawnModel::OnModelName,
                                this);
  this->node.Subscribe(req_model_pose_topic, &SpawnModel::OnModelPoseReq,
                                this);
  this->PosePub = this->node.Advertise<ignition::msgs::Pose>(res_model_pose_topic);
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
        std::string sdf = std::string("<sdf version='1.7'>") +
        "<model name='" + this->model_name[i] +"_" + std::to_string(this->nrObjectsSpawned) + "'> " +
          "<include>"
            "<uri>model://"+ this->model_name[i]+ "</uri>"+
          "</include>"+
        "</model>"+
      "</sdf>";
      //Define the pose of the spawned object
       ignition::msgs::Set(req.mutable_pose(),math::Pose3d(this->objectPosition[i].x(),this->objectPosition[i].y(),this->objectPosition[i].z(),
       this->objectOrientation[i].w(),this->objectOrientation[i].x(),this->objectOrientation[i].y(),this->objectOrientation[i].z()));
       //Add the xml to the request object
       req.set_sdf(sdf);
       //Call service 
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
    // Update model to new pose
    if (this->move_model_name_ready){
      ignition::msgs::Pose req1;
      ignition::msgs::Boolean res1;
      std::vector<std::string> v;
      auto worldEntity = _ecm.EntityByComponents(components::World());
      const auto models = _ecm.EntitiesByComponents(components::ParentEntity(worldEntity), components::Model());
      for(const auto &m: models){
            v.push_back(_ecm.Component<components::Name>(m)->Data());
      }
      if (std::find(v.begin(), v.end(), this->move_model_name) != v.end()){
        req1.set_name(this->move_model_name);

        math::Quaterniond q (this->qw,this->qx,this->qy,this->qz);
        ignition::msgs::Set(req1.mutable_position(), math::Vector3d( this->x, this->y, this->z));
        ignition::msgs::Set(req1.mutable_orientation(), q);
        std::string poseCmdService("/world/" + this->worldName + "/set_pose");
        this->node.Request(poseCmdService, req1, timeout, res1, result);
        this->move_model_name_ready = false;
      }
    }
  }
// Necessary for retrieving the pose of a specified model
void SpawnModel::PostUpdate(const UpdateInfo &_info,
    const EntityComponentManager &_ecm)
{
  IGN_PROFILE("SpawnModel::PostUpdate");
  if(!this->search_model.empty()){
    auto worldEntity = _ecm.EntityByComponents(components::World());
    const auto models = _ecm.EntitiesByComponents(components::ParentEntity(worldEntity), components::Model());
    for(const auto &m: models){
          if (this->search_model == _ecm.Component<components::Name>(m)->Data()){
            math::Pose3 pose = _ecm.Component<components::Pose>(m)->Data();
            this->PosePub.Publish(ignition::msgs::Convert(pose));
            this->search_model = "";
            break;
          }
    }
    
  }else{
    return;
  }
  //this->dataPtr->UpdateOdometry(_info, _ecm);
}

// //////////////////////////////////////////////////
/// \brief Callback for moving model command
/// \param[in] _msg Message
void SpawnModel::OnMoveCmd(const ignition::msgs::Pose &_msg)
{   
    this->x = _msg.position().x();
    this->y = _msg.position().y();
    this->z = _msg.position().z();
    this->qw = _msg.orientation().w();
    this->qx = _msg.orientation().x();
    this->qy = _msg.orientation().y();
    this->qz = _msg.orientation().z();
}

void SpawnModel::OnModelName(const ignition::msgs::StringMsg &_msg)
{   
    // Check if the received data is not empty
    if (!_msg.data().empty()){
      this->move_model_name = _msg.data();
      this-> move_model_name_ready = true;
    }
}

void SpawnModel::OnModelPoseReq(const ignition::msgs::StringMsg &_msg)
{   
    // Check if the received data is not empty
    if (!_msg.data().empty()){
      this->search_model = _msg.data();
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
                    SpawnModel::ISystemUpdate,
                    SpawnModel::ISystemPostUpdate)

IGNITION_ADD_PLUGIN_ALIAS(SpawnModel,
                          "ignition::gazebo::systems::SpawnModel")


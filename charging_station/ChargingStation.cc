#include <ignition/plugin/Register.hh>

#include "ChargingStation.hh"

using namespace ignition;
using namespace gazebo;
using namespace systems;

/////////////////////////////////////////////////
/// \brief Constructor
ChargingStation::ChargingStation() = default;

/////////////////////////////////////////////////
/// \brief Destructor
ChargingStation::~ChargingStation() = default;

//////////////////////////////////////////////////
/// \brief This function is called when the model attached is loaded in the simulation
/// \param[in] _entity Object model that this plugin is attached (AKA charge stations models)
/// \param[in] _sdf SDF element of the plugin in the model attached 
/// \param[in] _ecm Entity Component Manager
void ChargingStation::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,  
    EventManager &/*_eventMgr*/)
{
  // Get the model and check if it is valid
  this->model = Model(_entity);
  if (!this->model.Valid(_ecm))
  {
    ignerr << "ChargingStation plugin should be attached to a model entity. "
           << "Failed to initialize." << std::endl;
    return;
  }

  // Get the battery name used in the robot. This name is a Ignition Gazebo linear battery plugin name
  this->robotName = _sdf->Get<std::string>("robot_names");
  if (this->robotName.empty())
  {
    ignerr << "ChargingStation found an empty robot_name parameter. This name should be in the world sdf model."
           << "Failed to initialize.";
    return;
  }
  this->tolerance = _sdf->Get<std::string>("tolerance");
  if (this->tolerance.empty())
  {
    ignerr << "ChargingStation found an empty tolerance parameter"
           << "Failed to initialize.";
    return;
  }

  // Get the world name in the simulation
  auto worldEntity = _ecm.EntityByComponents(components::World());
  if (worldEntity == kNullEntity)
    return;

  // Get the components name to create the contact topic subscriber
  this->ent = this->model.Entity();
  std::string chargeName = _ecm.Component<components::Name>(_entity)->Data();
  this->dockLink = _ecm.ChildrenByComponents(_entity, components::Link());
  std::string worldName = _ecm.Component<components::Name>(worldEntity)->Data();
  std::vector<std::string> topics;
  std::string delimiter = ",";
  size_t pos = 0;
  std::string token;
  ignition::transport::Node::Publisher publisher;

  while((pos = this->robotName.find(delimiter)) != std::string::npos){
    token = this->robotName.substr(0,pos);
    this->robots.push_back(token);
    publisher = this->node.Advertise<ignition::msgs::Boolean>("/world/" + worldName + "/model/" + chargeName +"/"+ token);
    this->publishers.push_back(publisher);
    this->robotToPublisher.insert({token, publisher});
    this->robotName.erase(0, pos + delimiter.length());
  }
  
  this->node.Subscribe("/start_docking", &ChargingStation::OnDockCmd, this);

  ignmsg << "ChargingStation Started" << std::endl;
}

//////////////////////////////////////////////////
/// \brief This function is called in each simulation step update.
/// \param[in] _info Simulation state information
/// \param[in] _ecm Entity Component Manager
void ChargingStation::Update(const ignition::gazebo::UpdateInfo &_info,
    ignition::gazebo::EntityComponentManager &_ecm)
{
  IGN_PROFILE("ChargingStation::Update");

  // Nothing left to do if paused.
  if (_info.paused)
    return;

  if(this->checkDocking){
    const auto dock_pose_ = _ecm.Component<components::Pose>(this->ent)->Data();
    const auto link_ = _ecm.Component<components::Pose>(this->dockLink[0]); 
    ignition::math::Pose3d dockPose = dock_pose_ - link_->Data();
    std::map<std::string,math::Pose3d> robotPoses;
    auto worldEntity = _ecm.EntityByComponents(components::World());
    const auto models = _ecm.EntitiesByComponents(components::ParentEntity(worldEntity), components::Model());
    for(const auto &m: models){
        if((std::find(this->robots.begin(), this->robots.end(),_ecm.Component<components::Name>(m)->Data()))!= this->robots.end()){
          robotPoses.insert({_ecm.Component<components::Name>(m)->Data(), _ecm.Component<components::Pose>(m)->Data()});
        }
    }
    std::vector<std::string> t = ComputeDistances(robotPoses, dockPose);
    if (!t.empty()){
      for(int i = 0; i<t.size();i++){
        this->robotToPublisher[t[i]].Publish(ignition::msgs::Convert(true));
      }
    }else{
      return;
    }

  }
}

//////////////////////////////////////////////////
/// \brief Callback for contact subscription
/// \param[in] _msg Message
void ChargingStation::OnDockCmd(const ignition::msgs::Boolean &_msg){
  this->checkDocking = false;
  if(_msg.data()){
    this->checkDocking = true;
  }


}

std::vector<std::string> ChargingStation::ComputeDistances(std::map<std::string, math::Pose3d>robotPoses, ignition::math::Pose3d dockPose){

  double dock_x = dockPose.X();
  double dock_y = dockPose.Y();
  double robot_x, robot_y, dist_x, dist_y, dist_, dist;

  //ignwarn << "dock position is " << dock_x << " " << dock_y <<std::endl;
  std::vector<std::string> result;

  for(const auto &pair : robotPoses){
    dist_x = dock_x - pair.second.X();
    dist_y = dock_y - pair.second.Y();
    //ignwarn << "robot position is " << pair.second.X() << " " << pair.second.Y() <<std::endl;
    dist_ = pow(dist_x, 2) + pow(dist_y, 2);
    dist = sqrt(dist_);
    //ignerr << dist << "for " << pair.first << std::endl;
    if (abs(dist) < std::stod(this->tolerance)){
      
      result.push_back(pair.first);
    }
  }

  return result;

} 

/////////////////////////////////////////////////
/// \brief Function used to split a string by a delimiter character.
/// \param[in] str std::string to be splited.
/// \param[in] delim char to delimiter the strings separation.
/// \return Returns a Vector of strings. std::vector<std::string>
std::vector<std::string> ChargingStation::SplitMsg(std::string const &str, const char delim)
{
  size_t start;
  size_t end = 0;
  std::vector<std::string> out;

  while ((start = str.find_first_not_of(delim, end)) != std::string::npos)
  {
    end = str.find(delim, start);
    out.push_back(str.substr(start, end - start));
  }
  return out;
}


// Register this plugin
IGNITION_ADD_PLUGIN(ChargingStation,
                    ignition::gazebo::System,
                    ChargingStation::ISystemConfigure,
                    ChargingStation::ISystemUpdate)

IGNITION_ADD_PLUGIN_ALIAS(ChargingStation,
                          "ignition::gazebo::systems::ChargingStation")


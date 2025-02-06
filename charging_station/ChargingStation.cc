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

  // Get the robot name(s) associated with the charging station.
  this->robotName = _sdf->Get<std::string>("robot_names");
  if (this->robotName.empty())
  {
    ignerr << "ChargingStation found an empty robot_name parameter. This name should be in the world sdf model."
           << "Failed to initialize.";
    return;
  }
  // Get the minimum distance between the robot and the "center" of the charging station to decide to send the charging signal  
  this->tolerance = _sdf->Get<std::string>("tolerance");
  if (this->tolerance.empty())
  {
    ignerr << "ChargingStation found an empty tolerance parameter"
           << "Failed to initialize.";
    return;
  }

  // Get the robot frame to use in the distance computation 
  this->robotFrame = _sdf->Get<std::string>("robot_frame");
  if (this->robotFrame.empty())
  {
    ignwarn << "ChargingStation found an empty robotFrame parameter"
           << " will use base link as default.";
  }

  // Get the world name in the simulation
  auto worldEntity = _ecm.EntityByComponents(components::World());
  if (worldEntity == kNullEntity)
    return;

  // Get the components name to publish the signal for the robot to start charging
  this->ent = this->model.Entity();
  std::string chargeName = _ecm.Component<components::Name>(_entity)->Data();

  this->dockLink = _ecm.ChildrenByComponents(_entity, components::Link());
  std::string worldName = _ecm.Component<components::Name>(worldEntity)->Data();
  std::vector<std::string> topics;
  std::string delimiter = ",";
  size_t pos = 0;
  std::string token, start_topic, stop_topic;
  ignition::transport::Node::Publisher publisher_start, publisher_stop;

  // get one or more robot names to create the charging publisher
  if (this->robotName.find(delimiter) == std::string::npos){
    this->PopulateMap(this->robotName);
  }
  else{
    while((pos = this->robotName.find(delimiter)) != std::string::npos){
      token = this->robotName.substr(0,pos);
      this->PopulateMap(token);
      this->robotName.erase(0, pos + delimiter.length());
    }

  }

  // Check if we want to check distance from the charging station to a specific robot frame. If not we use the base_link
  Entity modelEntity  = _ecm.EntityByComponents(components::Name(this->robots[0]));

  if(!this->robotFrame.empty()){
    Entity linkEntity =  _ecm.EntityByComponents(components::Name(this->robotFrame), components::ParentEntity(modelEntity));
    this->poseLink = _ecm.Component<components::Pose>(linkEntity)->Data();
   
  }else{
    
    ignition::math::Pose3d emptyPose(0.0,0.0,0.0,0.0,0.0,0.0);
    this->poseLink = emptyPose;
  }



  // optional subscriber to start the charging station plugin logic
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
      // create a map where the key is a robot name and the value is its pose
        if((std::find(this->robots.begin(), this->robots.end(),_ecm.Component<components::Name>(m)->Data()))!= this->robots.end()){
          robotPoses.insert({_ecm.Component<components::Name>(m)->Data(), _ecm.Component<components::Pose>(m)->Data()});
        }
    }
    // Calculate distance between robots and the charging station
    std::vector<std::string> t = ComputeDistances(robotPoses, dockPose);
    if (!t.empty()){
      for(int i = 0; i<t.size();i++){
        // if a robot is close enough to a specific link of the charging station check if we already sent the charging signal. If not, publish it
        if(!this->robotsMap[t[i]].getLastMsg()){
          this->robotsMap[t[i]].getTopicStart().Publish(ignition::msgs::Convert(true));
          this->robotsMap[t[i]].setLastMsg(true);
        }
      }
    }else{
      // if the charging signal was sent previously, but the robot is no longer in the desired range of the charging station, send a signal to stop charging
      for(auto &pair: this->robotsMap){
        if(pair.second.getLastMsg()){
          pair.second.getTopicStop().Publish(ignition::msgs::Convert(true));
          pair.second.setLastMsg(false);
        }
      }
      return;
    }

  }
}
//////////////////////////////////////////////////
/// \brief Callback for start docking logic subscription
/// \param[in] _msg Message
void ChargingStation::OnDockCmd(const ignition::msgs::Boolean &_msg){
  this->checkDocking = false;
  if(_msg.data()){
    this->checkDocking = true;
  }
}
/////////////////////////////////////////////////
/// \brief Function used to compute the distance between the robot and the charging station
/// \param[in] robotPoses map of all the robots' pose in the world frame
/// \param[in] dockPose Dock pose in the world frame
/// \return Returns a Vector of robot names that are very close to the charging station
std::vector<std::string> ChargingStation::ComputeDistances(std::map<std::string, math::Pose3d>robotPoses, ignition::math::Pose3d dockPose){

  // Norm of the distance between the charging station and a given robot
  double dist;
  // vector of names of robots that are close enough to this charging station
  std::vector<std::string> result;

  // iterate through robot map
  for(const auto &pair : robotPoses){

    // Create a rotation matrix that converts the world frame into the robot frame
    Eigen::Matrix3d robotRot = rpyToRotationMatrix(pair.second.Roll(),pair.second.Pitch(),pair.second.Yaw()); 

    // Create a vector of the translation of the specific link in the robot frame
    Eigen::Vector3d linkTranslation(this->poseLink.X(),this->poseLink.Y(),this->poseLink.Z());
    
    // Convert the link translation to the world frame
    Eigen::Vector3d linkTranslationWorld = robotRot*(linkTranslation);

    // Vector that holds the robot current position in the world frame
    Eigen::Vector3d robotPosition(pair.second.X(),pair.second.Y(),pair.second.Z());
    
    // Get the position of the specific link
    robotPosition += linkTranslationWorld;

    // Vector that holds the dock position
    Eigen::Vector3d dockPosition(dockPose.X(), dockPose.Y(), dockPose.Z());

    // Distance between the dock and the robot link
    Eigen::Vector3d result_vector = dockPosition-robotPosition;

    // euclidian distance
    dist = sqrt(pow(result_vector[0], 2) + pow(result_vector[1], 2));

    // Return a list of robots that are closer to a charging station than a parameterizable threshold
    if (dist < std::stod(this->tolerance)){
      result.push_back(pair.first);
    }
  }

  return result;

}

/////////////////////////////////////////////////
/// \brief Function used to instatiate, populate and associate UserVars to a robot
/// \param[in] token name of the robot
void ChargingStation::PopulateMap(std::string &token){

  std::string start_topic, stop_topic;
  ignition::transport::Node::Publisher publisher_start, publisher_stop;
  start_topic = "/model/" + token + "/battery/linear_battery/recharge/start";
  stop_topic = "/model/" + token + "/battery/linear_battery/recharge/stop";
  publisher_start = this->node.Advertise<ignition::msgs::Boolean>(start_topic);
  publisher_stop = this->node.Advertise<ignition::msgs::Boolean>(stop_topic);
  this->robots.push_back(token);
  this->robotsMap.insert({token, UserVars(publisher_start,publisher_stop, false)});
}

/////////////////////////////////////////////////
/// \brief Function used to create a rotation matrix based on roll pitch yaw
/// \param[in] roll rotation around the x-axis
/// \param[in] pitch rotation around the y-axis
/// \param[in] yaw rotation around the z-axis
Eigen::Matrix3d ChargingStation::rpyToRotationMatrix(double roll, double pitch, double yaw)
{
    // Create rotation matrices for each axis
    Eigen::Matrix3d R_x, R_y, R_z;

    // Roll (rotation around the x-axis)
    R_x << 1, 0, 0,
           0, cos(roll), -sin(roll),
           0, sin(roll), cos(roll);

    // Pitch (rotation around the y-axis)
    R_y << cos(pitch), 0, sin(pitch),
           0, 1, 0,
           -sin(pitch), 0, cos(pitch);

    // Yaw (rotation around the z-axis)
    R_z << cos(yaw), -sin(yaw), 0,
           sin(yaw), cos(yaw), 0,
           0, 0, 1;

    // Combine the rotations
    Eigen::Matrix3d R = R_z * R_y * R_x;
    
    return R;
}
/////////////////////////////////////////////////
/// \brief Function used to create an homogeneous matrix based on translation values and a rotation matrix
/// \param[in] x translation in x-axis
/// \param[in] y translation in y-axis
/// \param[in] z translation in z-axis
/// \param[in] mat rotation matrix
Eigen::Matrix4d ChargingStation::createHomogenousMat(double x, double y, double z, Eigen::Matrix3d mat)
{
  Eigen::Vector3d translation(x, y, z);

  // initialize homogeneous matrix with identity (diagonal is 1, rest is zeros)
  Eigen::Matrix4d transformation = Eigen::Matrix4d::Identity();

  // Replace the first 3x3 block by the rotation matrix
  transformation.block<3, 3>(0, 0) = mat;

  // Repalce the last column (with the exception of the last value) with the translation
  transformation.block<3, 1>(0, 3) = translation;

  return transformation;
}

// Register this plugin
IGNITION_ADD_PLUGIN(ChargingStation,
                    ignition::gazebo::System,
                    ChargingStation::ISystemConfigure,
                    ChargingStation::ISystemUpdate)

IGNITION_ADD_PLUGIN_ALIAS(ChargingStation,
                          "ignition::gazebo::systems::ChargingStation")


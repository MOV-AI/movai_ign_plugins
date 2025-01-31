#ifndef CHARGINGSTATION_HH_
#define CHARGINGSTATION_HH_

#include <ignition/gazebo/System.hh>
#include <ignition/transport/Node.hh>
#include "ignition/gazebo/Model.hh"
#include "ignition/gazebo/Link.hh"
#include "ignition/gazebo/Entity.hh"
#include "ignition/gazebo/components/Link.hh"
#include <ignition/common/Profiler.hh>
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/World.hh"
#include "ignition/gazebo/components/Model.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/components/ParentEntity.hh"
#include <ignition/msgs/Utility.hh>
#include<math.h>

using namespace ignition;
using namespace gazebo;
using namespace systems;

/// \brief Class that holds the variables associated with each robot and respective utility functions
class UserVars{
public:
  ignition::transport::Node::Publisher topicStart;
  ignition::transport::Node::Publisher topicStop;
  bool lastMsg;

  UserVars() = default;
  UserVars(ignition::transport::Node::Publisher topic1, ignition::transport::Node::Publisher topic2, bool last): topicStart(topic1), topicStop(topic2), lastMsg(last){}

  ignition::transport::Node::Publisher getTopicStart(){
    return topicStart;
  }
  ignition::transport::Node::Publisher getTopicStop(){
    return topicStop;
  } 
  bool getLastMsg(){
    return lastMsg;
  }
  void setTopicStart(ignition::transport::Node::Publisher newTopic){
    topicStart = newTopic;
  }
  void setTopicStop(ignition::transport::Node::Publisher newTopic){
    topicStop = newTopic;
  }
  void setLastMsg(bool newMsg){
    lastMsg = newMsg;
  }
};
/// \brief Battery Station Plugin that can identify and trigger the charge of a linear battery plugin attached to
/// a robot in the simulation.
class ChargingStation : public ignition::gazebo::System,
    public ignition::gazebo::ISystemConfigure,
    public ignition::gazebo::ISystemUpdate
{
  /// \brief Constructor
  public: ChargingStation();

  /// \brief Destructor
  public: ~ChargingStation() override;

  //////////////////////////////////////////////////
  /// \brief This function is called when the model attached is loaded in the simulation
  /// \param[in] _entity Object model that this plugin is attached (AKA charge stations models)
  /// \param[in] _sdf SDF element of the plugin in the model attached 
  /// \param[in] _ecm Entity Component Manager
  public: void Configure(const Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           EntityComponentManager &_ecm,
                           EventManager &_eventMgr) override;

  //////////////////////////////////////////////////
  /// \brief This function is called in each simulation step update.
  /// \param[in] _info Simulation state information
  /// \param[in] _ecm Entity Component Manager
  public: void Update(const ignition::gazebo::UpdateInfo &_info,
              ignition::gazebo::EntityComponentManager &_ecm) override;

  //////////////////////////////////////////////////
  /// \brief Callback for start docking subscription
  /// \param[in] _msg Message
  private: void OnDockCmd(const ignition::msgs::Boolean &_msg);

  /////////////////////////////////////////////////
  /// \brief Function used to compute the distance between the robot and the charging station
  /// \param[in] robotPoses map of all the robots' pose in the world frame
  /// \param[in] dockPose Dock pose in the world frame
  /// \return Returns a Vector of robot names that are very close to the charging station
  private: std::vector<std::string> ComputeDistances(std::map<std::string, math::Pose3d> robotPoses, ignition::math::Pose3d dockPose);
 
  /////////////////////////////////////////////////
  /// \brief Function used to instatiate, populate and associate UserVars to a robot
  /// \param[in] token name of the robot
  private: void PopulateMap(std::string &token);

  /// \brief Ignition communication node.
  public: ignition::transport::Node node;

  /// \brief Robot name var to get the model in contact with the charge
  public: std::string contactRobotName;

  /// \brief Robot link name that is used to charge the battery
  public: std::string robotLinkName;

  /// \brief Robot battery name used in the ignition plugin Linear Battery
  public: std::string batteryName;
  
  /// \brief Charge state var in the update moment
  public: bool chargeState{true};

  /// \brief Charge state var in the last update
  public: bool oldState{false};

  /// \brief optional boolean to decide when to run the charging station plugin logic 
  public: bool checkDocking{true};

  /// \brief Model entity that this plugin is attached
  public: Model model{kNullEntity};

  // list of robot names
  public: std::vector<std::string> robots;

  /// \brief variable to read the list of robots from the sdf
  public: std::string robotName;

  /// \brief minimum distance beteen the robot and the charging station
  public: std::string tolerance;

  /// \brief link of the charging station 
  public: std::vector<Entity> dockLink;
  
  /// \brief Publishers
  public: std::map<std::string, ignition::transport::Node::Publisher> robotToPublisherStart;
  public: std::map<std::string, ignition::transport::Node::Publisher> robotToPublisherStop;
  
  /// \brief Last message sent
  public: std::map<std::string, bool> robotLastMsgPub;

  public: Entity ent;

  /// \brief Map that holds the robot name and the variables associated with it
  public: std::map<std::string, UserVars> robotsMap;
};


  
#endif

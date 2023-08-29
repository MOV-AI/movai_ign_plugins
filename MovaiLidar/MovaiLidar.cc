#include <ignition/plugin/Register.hh>

#include "MovaiLidar.hh"
#include <random>
#include <ignition/msgs/PointCloudPackedUtils.hh>
#include <ignition/msgs/pointcloud_packed.pb.h>

using namespace ignition;
using namespace gazebo;
using namespace systems;

/////////////////////////////////////////////////
/// \brief Constructor
MovaiLidar::MovaiLidar() = default;

/////////////////////////////////////////////////
/// \brief Destructor
MovaiLidar::~MovaiLidar() = default;

//////////////////////////////////////////////////
/// \brief This function is called when the model attached is loaded in the simulation
/// \param[in] _entity Object model that this plugin is attached 
/// \param[in] _sdf SDF element of the plugin in the model attached 
/// \param[in] _ecm Entity Component Manager
void MovaiLidar::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &/*_eventMgr*/)
{
  // Get the model and check if it is valid
  this->model = Model(_entity);
  if (!this->model.Valid(_ecm))
  {
    ignerr << "Lidar Model plugin should be attached to a model entity. "
           << "Failed to initialize." << std::endl;
    return;
  }
  
  // Get the world name in the simulation
  auto worldEntity = _ecm.EntityByComponents(components::World());
  if (worldEntity == kNullEntity)
    return;

  // Get the component's name to create the scan and points topic subscribers
  std::string objectName = _ecm.Component<components::Name>(_entity)->Data();

  // Get world name to build the topic string
  this->worldName = _ecm.Component<components::Name>(worldEntity)->Data();

  // Topics to subscribe
  std::string scan_topic = "/world/" + this->worldName + "/model/" + objectName + "/link/scan_omni/sensor/scan_omni/scan";
  std::string points_topic = "/world/" + this->worldName + "/model/" + objectName + "/link/scan_omni/sensor/scan_omni/scan/points";
  std::string intensity_flag = "/allow_intensities";

  // Topics to publish
  std::string scan_pub_topic = "/2d_intensities";
  std::string points_pub_topic = "/3d_intensities";

  // Subscribers
  this->node.Subscribe(scan_topic, &MovaiLidar::On2dScan,
                                this);
  this->node.Subscribe(points_topic, &MovaiLidar::On3dScan,
                                this);
  this->node.Subscribe(intensity_flag, &MovaiLidar::OnIntensityFlag,
                                this);
  //Publishers
  this->pubScan = this->node.Advertise<ignition::msgs::LaserScan>(
        scan_pub_topic);

  this->pubPoints = this->node.Advertise<ignition::msgs::PointCloudPacked>(
        points_pub_topic);
}

//////////////////////////////////////////////////
/// \brief This function is called in each simulation step update.
/// \param[in] _info Simulation state information
/// \param[in] _ecm Entity Component Manager
void MovaiLidar::Update(const ignition::gazebo::UpdateInfo &_info,
                            ignition::gazebo::EntityComponentManager &_ecm)
{
  IGN_PROFILE("MovaiLidar::Update");
  
  //Publish scan and points messages with intensities in each timestep
  this->pubScan.Publish(this->intensityScan);
  this->pubPoints.Publish(this->intensityPoints);
}

// //////////////////////////////////////////////////
/// \brief Callback for model LaserScan message subscription
/// \param[in] _msg Message
void MovaiLidar::On2dScan(const ignition::msgs::LaserScan &_msg)
{   
  // check if we want to turn on intensities
  if(this->allowIntensities){
    
    //Copy incoming message
    auto to_send = _msg;
    int j = -1;

    //For each sample set intensity to a random value between 0-255
    for(int i = 0; i<to_send.ranges_size();i++){
      j = ( std::rand() % ( 255 + 1 ) );
      to_send.set_intensities(i,j);
    }
    this->intensityScan = to_send;
  }
}

// //////////////////////////////////////////////////
/// \brief Callback for model PoseArray message subscription
/// \param[in] _msg Message
void MovaiLidar::OnIntensityFlag(const ignition::msgs::Boolean &_msg)
{   
    //Flag that defines if intensities are to be published
    this->allowIntensities = _msg.data();

   
}

void MovaiLidar::On3dScan(const ignition::msgs::PointCloudPacked &_msg)
{ 
  if(this->allowIntensities){
    ignition::msgs::PointCloudPacked pcp = _msg;

    //declare var to iterate over pcd message, but only through intensity field
    // We replace each value of intensity by a random value between 0-255
    ignition::msgs::PointCloudPackedIterator<float> iter(pcp, "intensity");
    while(iter != iter.End()){
      *iter =  ( std::rand() % ( 255 + 1 ) );
      ++iter;
    }
    this->intensityPoints = pcp;
  }
}
// Register this plugin
IGNITION_ADD_PLUGIN(MovaiLidar,
                    ignition::gazebo::System,
                    MovaiLidar::ISystemConfigure,
                    MovaiLidar::ISystemUpdate)

IGNITION_ADD_PLUGIN_ALIAS(MovaiLidar,
                          "ignition::gazebo::systems::MovaiLidar")


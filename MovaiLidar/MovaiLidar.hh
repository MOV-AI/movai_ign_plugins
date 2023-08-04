#ifndef MOVAILIDAR_HH_
#define MOVAILIDAR_HH_

#include <ignition/gazebo/System.hh>
#include <ignition/transport/Node.hh>
#include "ignition/gazebo/Model.hh"
#include <ignition/common/Profiler.hh>
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/World.hh"


using namespace ignition;
using namespace gazebo;
using namespace systems;

/// \brief MovaiLidar Plugin that can publish a scan/points message with random intensities
class MovaiLidar : public ignition::gazebo::System,
    public ignition::gazebo::ISystemConfigure,
    public ignition::gazebo::ISystemUpdate
{
  /// \brief Constructor
  public: MovaiLidar();

  /// \brief Destructor
  public: ~MovaiLidar() override;



  //////////////////////////////////////////////////
  /// \brief This function is called when the model attached is loaded in the simulation
  /// \param[in] _entity Object model that this plugin is attached 
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
              ignition::gazebo::EntityComponentManager &_ecm);

  //////////////////////////////////////////////////
  /// \brief Callbacks for scan/points topics subscription
  /// \param[in] _msg Message
  private: void On3dScan(const ignition::msgs::PointCloudPacked &_msg);
  private: void On2dScan(const ignition::msgs::LaserScan &_msg);

  /// \brief Callbacks for control topic subscription
  /// \param[in] _msg Message that dictates if intensities are published
  private: void OnIntensityFlag(const ignition::msgs::Boolean &_msg);
  
  /// \brief Declare the needed publishers
  private: ignition::transport::Node::Publisher pubScan;
  private: ignition::transport::Node::Publisher pubPoints;

  /// \brief Node responsible for subring and publishing desired topics
  private: ignition::transport::Node node;

  /// \brief Messages to send
  private: ignition::msgs::PointCloudPacked intensityPoints;
  private: ignition::msgs::LaserScan intensityScan;
  

  /// \brief Name of the model to which the plugin is attached
   public: std::string  objectName;

  /// \brief World name
  public: std::string  worldName;
  
  /// \brief Model entity that this plugin is attached
  public: Model model{kNullEntity};

  /// \brief Control flag
  public: bool allowIntensities{false};

};

#endif

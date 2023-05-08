#ifndef SPAWNMODEL_HH_
#define SPAWNMODEL_HH_

#include <ignition/gazebo/System.hh>
#include <ignition/transport/Node.hh>
#include "ignition/gazebo/Model.hh"
#include <ignition/common/Profiler.hh>
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/World.hh"


using namespace ignition;
using namespace gazebo;
using namespace systems;

/// \brief Spawn Object Plugin that can identify and trigger the creation of the object it is attached to
class SpawnModel : public ignition::gazebo::System,
    public ignition::gazebo::ISystemConfigure,
    public ignition::gazebo::ISystemUpdate
{
  /// \brief Constructor
  public: SpawnModel();

  /// \brief Destructor
  public: ~SpawnModel() override;



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
  /// \brief Callback for spawn topic subscription
  /// \param[in] _msg Message
  private: void OnSpawnCmd(const ignition::msgs::Pose_V &_msg);

  private: void OnModelArray(const ignition::msgs::StringMsg &_msg);

  /// \brief Ignition communication node.
  public: ignition::transport::Node node;

  /// \brief Name of the model to spawn
   public: std::string  objectName;

  /// \brief World name
  public: std::string  worldName;
  
  /// @brief Object Pose
  public: std::vector<ignition::msgs::Quaternion> objectOrientation;

  public: std::vector<ignition::msgs::Vector3d> objectPosition;
  /// @brief Number of objects spawned so far
  public: int nrObjectsSpawned{0};

  /// \brief Spawn trigger variable in the update function
  public: bool spawnObject{false};

  /// \brief Model entity that this plugin is attached
  public: Model model{kNullEntity};

  /// \brief Model names variablle in the update function
  public: bool string_ready{false};
 
  /// \brief To spawn model names vector 
  public: std::vector<std::string> model_name;

  /// \brief Character between model names 
  private: char delimiter{'!'};

};

#endif

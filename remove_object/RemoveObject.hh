#ifndef REMOVEOBJECT_HH_
#define REMOVEOBJECT_HH_

#include <ignition/gazebo/System.hh>
#include <ignition/transport/Node.hh>
#include "ignition/gazebo/Model.hh"
#include <ignition/common/Profiler.hh>
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/World.hh"


using namespace ignition;
using namespace gazebo;
using namespace systems;

/// \brief Remove Object Plugin that can identify and trigger the removal of the object it is attached to
class RemoveObject : public ignition::gazebo::System,
    public ignition::gazebo::ISystemConfigure,
    public ignition::gazebo::ISystemUpdate,
    public ignition::gazebo::ISystemPostUpdate
{
  /// \brief Constructor
  public: RemoveObject();

  /// \brief Destructor
  public: ~RemoveObject() override;

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
  /// \brief Callback for removal subscription
  /// \param[in] _msg Message
  // private: void OnTrigger(const ignition::msgs::StringMsg_V &_msg);
  private: void OnTrigger(const msgs::StringMsg &_msg);

  public: void PostUpdate(const UpdateInfo &/*_info*/,
                          const EntityComponentManager &_ecm) ;

  /// \brief Ignition communication node.
  public: ignition::transport::Node node;

  /// \brief Robot name var to get the model to remove
  public: std::string objectName;

  /// \brief World name in which to remove entities
  public: std::string  worldName;

  /// \brief Vector to populate with models' names 
  public: std::vector<std::string> vector_name;
  
  /// \brief Boolean to trigger the removal of a specific entity
  public: bool remove_object{false};

  /// \brief Model entity that this plugin is attached
  public: Model model{kNullEntity};

  /// \brief Delimiter between model names
  private: char delimiter{'!'};

  /// \brief Removal service string
  public: std::string srvRemove;
};

#endif

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
    public ignition::gazebo::ISystemUpdate
{
  /// \brief Constructor
  public: RemoveObject();

  /// \brief Destructor
  public: ~RemoveObject() override;

  //////////////////////////////////////////////////
  /// \brief This function is called when the model attached is loaded in the simulation
  /// \param[in] _entity Object model that this plugin is attached (AKA pallet models)
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
  /// \brief Callback for contact subscription
  /// \param[in] _msg Message
  private: void OnTrigger(const ignition::msgs::StringMsg &_msg);


  /// \brief Ignition communication node.
  public: ignition::transport::Node node;

  /// \brief Robot name var to get the model in contact with the charge
  public: std::string objectName;

  // /// \brief Robot link name that is used to charge the battery
  // public: std::string robotLinkName;

  // /// \brief Robot battery name used in the ignition plugin Linear Battery
  public: std::string  worldName;
  
  /// \brief Charge state var in the update moment
  public: bool remove_object{false};

  // /// \brief Charge state var in the last update
  // public: bool oldState{false};

  /// \brief Model entity that this plugin is attached
  public: Model model{kNullEntity};
};

#endif

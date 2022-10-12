#ifndef BATTERYSTATION_HH_
#define BATTERYSTATION_HH_

#include <ignition/gazebo/System.hh>
#include <ignition/transport/Node.hh>
#include "ignition/gazebo/Model.hh"
#include <ignition/common/Profiler.hh>
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/World.hh"


using namespace ignition;
using namespace gazebo;
using namespace systems;

/// \brief Battery Station Plugin that can identify and trigger the charge of a linear battery plugin attached to
/// a robot in the simulation.
class BatteryStation : public ignition::gazebo::System,
    public ignition::gazebo::ISystemConfigure,
    public ignition::gazebo::ISystemUpdate
{
  /// \brief Constructor
  public: BatteryStation();

  /// \brief Destructor
  public: ~BatteryStation() override;

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
  /// \brief Callback for contact subscription
  /// \param[in] _msg Message
  private: void OnContact(const ignition::msgs::Contacts &_msg);

  /////////////////////////////////////////////////
  /// \brief Function used to split a string by a delimiter character.
  /// \param[in] str std::string to be splited.
  /// \param[in] delim char to delimiter the strings separation.
  /// \return Returns a Vector of strings. std::vector<std::string>
  public: std::vector<std::string> SplitMsg(std::string const &str, const char delim);

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

  /// \brief Model entity that this plugin is attached
  public: Model model{kNullEntity};
};

#endif

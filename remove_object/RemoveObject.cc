#include <ignition/plugin/Register.hh>

#include "RemoveObject.hh"

using namespace ignition;
using namespace gazebo;
using namespace systems;

/////////////////////////////////////////////////
/// \brief Constructor
RemoveObject::RemoveObject() = default;

/////////////////////////////////////////////////
/// \brief Destructor
RemoveObject::~RemoveObject() = default;

//////////////////////////////////////////////////
/// \brief This function is called when the model attached is loaded in the simulation
/// \param[in] _entity Object model that this plugin is attached
/// \param[in] _sdf SDF element of the plugin in the model attached 
/// \param[in] _ecm Entity Component Manager
void RemoveObject::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &/*_eventMgr*/)
{
  // Get the model and check if it is valid
  this->model = Model(_entity);
  if (!this->model.Valid(_ecm))
  {
    ignerr << "RemoveObject plugin should be attached to a model entity. "
           << "Failed to initialize." << std::endl;
    return;
  }
  
  // Get the world name in the simulation
  auto worldEntity = _ecm.EntityByComponents(components::World());

  if (worldEntity == kNullEntity)
    return;

  // Get the components name to create the removal topic subscriber
  std::string palletName = _ecm.Component<components::Name>(_entity)->Data();
  this->worldName = _ecm.Component<components::Name>(worldEntity)->Data();
  //Change name of the topic to subscribe to
  std::string topic = "/model/remove";

  // Subscribe to the removal topic
  this->node.Subscribe(topic, &RemoveObject::OnTrigger,
                                this);

}

//////////////////////////////////////////////////
/// \brief This function is called in each simulation step update.
/// \param[in] _info Simulation state information
/// \param[in] _ecm Entity Component Manager
void RemoveObject::Update(const ignition::gazebo::UpdateInfo &_info,
                            ignition::gazebo::EntityComponentManager &_ecm)
{
  IGN_PROFILE("RemoveObject::Update");

  // Nothing left to do if paused.
  if (_info.paused)
    return;
  
  // Create the ignition service request for the removal service and call it

    bool result ;
    ignition::msgs::Entity req;
    ignition::msgs::Boolean res;
    int timeout = 100;
    if (this->remove_object)
    {
        const std::string srvRemove = std::string("/world/") + this->worldName + "/remove";
        auto entities = _ecm.EntitiesByComponents(components::Name(this->objectName));
        ignwarn << entities.size() << std::endl;
        for (int i =0 ;i<entities.size();i++){
          req.set_id(entities[i]);
          bool temp = this->node.Request(srvRemove, req, timeout,res,result);
          ignwarn << entities[i] << " removed"<< std::endl;
        }       
    
       
       
      // ignmsg << "Removed Object" << std::endl;
    }

  // Set the state to false to only try to remove each object once
  this->remove_object = false;
    
}

// //////////////////////////////////////////////////
/// \brief Callback for removal subscription
/// \param[in] _msg Message
void RemoveObject::OnTrigger(const ignition::msgs::StringMsg &_msg)
{
    if (_msg.data() != ""){
        this->remove_object = true;
        this->objectName = _msg.data();

    }
}

// Register this plugin
IGNITION_ADD_PLUGIN(RemoveObject,
                    ignition::gazebo::System,
                    RemoveObject::ISystemConfigure,
                    RemoveObject::ISystemUpdate)

IGNITION_ADD_PLUGIN_ALIAS(RemoveObject,
                          "ignition::gazebo::systems::RemoveObject")


#include <ignition/plugin/Register.hh>
#include <ignition/gazebo/components/World.hh>
#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/components/Model.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/ParentEntity.hh>
#include <string>
#include <vector>
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
  this->srvRemove = std::string("/world/") + this->worldName + "/remove";

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
    bool cv = true;
    std::vector<std::string> v;
    auto worldEntity = _ecm.EntityByComponents(components::World());
    const auto models = _ecm.EntitiesByComponents(components::ParentEntity(worldEntity), components::Model());

    // Populate a vector with the models present in the world
    for(const auto &m: models){
        v.push_back(_ecm.Component<components::Name>(m)->Data());
        
    }
    // Check if we want to remove objects
    if (this->remove_object)
    {
      //Check if models to remove are present in the world
      for(const auto &m : this->vector_name){
        if (std::find(v.begin(), v.end(), m) != v.end()){

          //Create removal service request
          auto entities = _ecm.EntityByComponents(components::Name(m));
          req.set_id(entities);
          bool temp = this->node.Request(srvRemove, req, timeout,res,result);

          if(!temp){
            ignerr<<"Failed to request removal" << std::endl;
            break;
            }
        }else{
          ignerr << "Model " << m << " not found in world " << worldEntity << std::endl;
          continue;
        }
      }

        }       
        // Set the state to false to only try to remove each object once and clear model names vector
       this->remove_object = false;
       this->vector_name.clear();
    }
void RemoveObject::PostUpdate(const ignition::gazebo::UpdateInfo &/*_info*/,
                              const ignition::gazebo::EntityComponentManager &_ecm){

      

}
// //////////////////////////////////////////////////
/// \brief Callback for removal subscription
/// \param[in] _msg Message

// void RemoveObject::OnTrigger(const ignition::msgs::StringMsg_V &_msg)
void RemoveObject::OnTrigger(const ignition::msgs::StringMsg &_msg)
{
    // Check if received data is empty
    if (!_msg.data().empty()){
      size_t pos = 0;
      std::string s = _msg.data();
      int counter = 0;

      //Create a vector with model names
      while((pos = s.find(this->delimiter)) != std::string::npos){
        this->vector_name.push_back(s.substr(0,pos)) ;        
        s.erase(0,pos+1);
        counter++;
      }
      this->remove_object = true;  
    }

}

// Register this plugin
IGNITION_ADD_PLUGIN(RemoveObject,
                    ignition::gazebo::System,
                    RemoveObject::ISystemConfigure,
                    RemoveObject::ISystemUpdate,
                    RemoveObject::ISystemPostUpdate)

IGNITION_ADD_PLUGIN_ALIAS(RemoveObject,
                          "ignition::gazebo::systems::RemoveObject")



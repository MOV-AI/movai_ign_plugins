#ifndef MATERIALCONTROLLER_HH_
#define MATERIALCONTROLLER_HH_

#include <ignition/gazebo/System.hh>
#include <memory>

namespace ignition
{
  namespace gazebo
  {
    // Inline bracket to help doxygen filtering.
    inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
      namespace systems
      {
        // Forward declaration
        class MaterialControllerPrivate;
      
        /// \brief This system applies a force to the first axis of a specified joint.
        class MaterialController
            : public System,
              public ISystemConfigure,
              public ISystemPreUpdate
        {
          /// \brief Constructor
          public: MaterialController();
      
          /// \brief Destructor
          public: ~MaterialController() override = default;
      
          // Documentation inherited
          public: void Configure(const Entity &_entity,
                                 const std::shared_ptr<const sdf::Element> &_sdf,
                                 EntityComponentManager &_ecm,
                                 EventManager &_eventMgr) override;
      
          // Documentation inherited
          public: void PreUpdate(
                      const ignition::gazebo::UpdateInfo &_info,
                      ignition::gazebo::EntityComponentManager &_ecm) override;
      
          /// \brief Private data pointer
          private: std::unique_ptr<MaterialControllerPrivate> dataPtr;
        };
      }
    }
  }
}

#endif

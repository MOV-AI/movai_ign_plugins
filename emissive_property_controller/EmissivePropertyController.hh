#ifndef EMISSIVEPROPERTYCONTROLLER_HH_
#define EMISSIVEPROPERTYCONTROLLER_HH_

#include <ignition/gazebo/gui/GuiSystem.hh>
#include <ignition/rendering/Scene.hh>
#include <ignition/transport/Node.hh>
#include "ignition/math/Color.hh"

/// \brief GUI plugin that can change the material color of a model link in runtime simulation
class EmissivePropertyController : public ignition::gazebo::GuiSystem
{
  Q_OBJECT

  /// \brief Constructor
  public: EmissivePropertyController();

  /// \brief Destructor
  public: ~EmissivePropertyController() override;

  /// \brief `ignition::gui::Plugin`s can overload this function to
  /// receive custom configuration from an XML file. Here, it comes from the
  /// SDF.
  /// \param[in] _pluginElem SDF <plugin> element. Will be null if the plugin
  /// is loaded without any XML configuration.
  public: void LoadConfig(const tinyxml2::XMLElement *_pluginElem) override;

  /// \brief Callback for all installed event filters.
  /// \param[in] _obj Object that received the event
  /// \param[in] _event Event
  private: bool eventFilter(QObject *_obj, QEvent *_event) override;

  /// \brief All rendering operations must happen within this call
  private: void PerformRenderingOperations();

  /// \brief Encapsulates the logic to find the rendering scene through the
  /// render engine singleton.
  private: void FindScene();

  /// \brief Callback for link and material subscription
  /// \param[in] _msg Message in "model::link-R-G-B-A" string
  public: void OnChangeCollor(const ignition::msgs::StringMsg &_msg);

  /////////////////////////////////////////////////
  /// \brief Function used to split a string by a delimiter character.
  /// \param[in] str std::string to be splited.
  /// \param[in] delim char to delimiter the strings separation.
  /// \return Returns a Vector of strings. std::vector<std::string>
  public: std::vector<std::string> SplitMsg(std::string const &str, const char delim);

  /// \brief Marks when a new change has been requested.
  private: bool materialRequest{false};

  /// \brief Pointer to the rendering scene.
  private: ignition::rendering::ScenePtr scene{nullptr};

  /// \brief Ignition communication node.
  public: ignition::transport::Node node;

  /// \brief color to apply
  public: ignition::math::Color newEmissive;

  /// \brief link name to change the material
  public: std::string linkName;
};

#endif

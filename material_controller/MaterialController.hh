#ifndef MATERIALCONTROLLER_HH_
#define MATERIALCONTROLLER_HH_

#include <ignition/gazebo/gui/GuiSystem.hh>
#include <ignition/rendering/Scene.hh>
#include <ignition/transport/Node.hh>
#include "ignition/math/Color.hh"

/// \brief Example of a GUI plugin that has access to entities and components.
class MaterialController : public ignition::gazebo::GuiSystem
{
  Q_OBJECT

    /// \brief Custom property. Use this to create properties that can be read
    /// from the QML file. See the declarations below.
    Q_PROPERTY(
      QString customProperty
      READ CustomProperty
      WRITE SetCustomProperty
      NOTIFY CustomPropertyChanged
    )

  /// \brief Constructor
  public: MaterialController();

  /// \brief Destructor
  public: ~MaterialController() override;

  /// \brief `ignition::gui::Plugin`s can overload this function to
  /// receive custom configuration from an XML file. Here, it comes from the
  /// SDF.
  ///
  /// <gui>
  ///   <plugin ...> <!-- this is the plugin element -->
  ///     ...
  ///   </plugin>
  /// </gui>
  ///
  /// \param[in] _pluginElem SDF <plugin> element. Will be null if the plugin
  /// is loaded without any XML configuration.
  public: void LoadConfig(const tinyxml2::XMLElement *_pluginElem) override;

  /// \brief GUI systems can overload this function to receive updated simulation
  /// state. This is called whenever the server sends state updates to the GUI.
  /// \param[in] _info Simulation information such as time.
  /// \param[in] _ecm Entity component manager, which can be used to get the
  /// latest information about entities and components.
  public: void Update(const ignition::gazebo::UpdateInfo &_info,
      ignition::gazebo::EntityComponentManager &_ecm) override;

  /// \brief Get the custom property as a string.
  /// \return Custom property
  public: Q_INVOKABLE QString CustomProperty() const;

  /// \brief Set the custom property from a string.
  /// \param[in] _customProperty Custom property
  public: Q_INVOKABLE void SetCustomProperty(const QString &_customProperty);

  /// \brief Notify that custom property has changed
  signals: void CustomPropertyChanged();

  /// \brief Callback when user clicks button.
  public slots: void RandomColor();

  /// \brief Callback for all installed event filters.
  /// \param[in] _obj Object that received the event
  /// \param[in] _event Event
  private: bool eventFilter(QObject *_obj, QEvent *_event) override;

  /// \brief All rendering operations must happen within this call
  private: void PerformRenderingOperations();

  /// \brief Encapsulates the logic to find the rendering scene through the
  /// render engine singleton.
  private: void FindScene();

  /// \brief Callback for joint force subscription
  /// \param[in] _msg Joint force message
  public: void OnChangeCollor(const ignition::msgs::StringMsg &_msg);

  /////////////////////////////////////////////////
  /// \brief Function used to split a string by a delimiter character.
  /// \param[in] str std::string to be splited.
  /// \param[in] delim char to delimiter the strings separation.
  /// \return Returns a Vector of strings. std::vector<std::string>
  public: std::vector<std::string> SplitMsg(std::string const &str, const char delim);

  /// \brief Marks when a new change has been requested.
  private: bool dirty{false};

  /// \brief Pointer to the rendering scene.
  private: ignition::rendering::ScenePtr scene{nullptr};

  /// \brief Ignition communication node.
  public: ignition::transport::Node node;

  /// \brief color to apply
  public: ignition::math::Color newEmissive;

  /// \brief Custom property
  private: QString customProperty;

  public: std::string linkName;
};

#endif

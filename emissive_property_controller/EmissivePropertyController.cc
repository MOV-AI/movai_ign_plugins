#include <ignition/plugin/Register.hh>
#include <ignition/gui/Application.hh>
#include <ignition/gui/GuiEvents.hh>
#include <ignition/gui/MainWindow.hh>
#include <ignition/rendering/RenderEngine.hh>
#include <ignition/rendering/RenderingIface.hh>
#include <ignition/rendering/Visual.hh>

#include "EmissivePropertyController.hh"

/////////////////////////////////////////////////
/// \brief Constructor
EmissivePropertyController::EmissivePropertyController() = default;

/////////////////////////////////////////////////
/// \brief Destructor
EmissivePropertyController::~EmissivePropertyController() = default;

/////////////////////////////////////////////////
/// \brief `ignition::gui::Plugin`s can overload this function to
/// receive custom configuration from an XML file. Here, it comes from the
/// SDF.
/// \param[in] _pluginElem SDF <plugin> element. Will be null if the plugin
/// is loaded without any XML configuration.
void EmissivePropertyController::LoadConfig(const tinyxml2::XMLElement * /*_pluginElem*/)
{
  if (this->title.empty())
    this->title = "EmissivePropertyController";
  // Creating an event filter to be able to catch the render event
  ignition::gui::App()->findChild<
      ignition::gui::MainWindow *>()->installEventFilter(this);

  // Creating the subscriber
  std::string topicName = "/model/material";
  auto topic = ignition::transport::TopicUtils::AsValidTopic(topicName);
  if (topic.empty())
  {
    ignerr << "Failed to create valid topic" << std::endl;
    return;
  }
  this->node.Subscribe(topic, &EmissivePropertyController::OnChangeCollor,
                                this);

  ignmsg << "EmissivePropertyController subscribing to string messages 'model_name::link_name-R-G-B-A' on [" << topic
         << "]" << std::endl;
}

//////////////////////////////////////////////////
/// \brief Callback for link and material subscription
/// \param[in] _msg Message in "model::link-R-G-B-A" string
void EmissivePropertyController::OnChangeCollor(const ignition::msgs::StringMsg &_msg)
{
  // Spliting the msg in Model::Link name, Red, Green, Blue and Alfa
  std::vector<std::string> seglist = SplitMsg(_msg.data(), '-');
  if (seglist.size() == 5)
  {
    this->linkName = seglist[0];
    this->newEmissive.R() = std::stof(seglist[1]);
    this->newEmissive.G() = std::stof(seglist[2]);
    this->newEmissive.B() = std::stof(seglist[3]);
    this->newEmissive.A() = std::stof(seglist[4]);
    // Flag to request a change in render event
    this->materialRequest = true;
  }
  else
  {
    ignerr << "String size does not match. The message should be 'model_name::link_name-R-G-B-A'" << std::endl;
  }
}

/////////////////////////////////////////////////
/// \brief Callback for all installed event filters.
/// \param[in] _obj Object that received the event
/// \param[in] _event Event
bool EmissivePropertyController::eventFilter(QObject *_obj, QEvent *_event)
{
  if (_event->type() == ignition::gui::events::Render::kType)
  {
    // This event is called in the render thread, so it's safe to make
    // rendering calls here
    this->PerformRenderingOperations();
  }

  // Standard event processing
  return QObject::eventFilter(_obj, _event);
}

/////////////////////////////////////////////////
/// \brief All rendering operations must happen within this call
void EmissivePropertyController::PerformRenderingOperations()
{
  // Check if there is a material change request. Do nothing if there is no request.
  if (!this->materialRequest)
  {
    return;
  }

  // Get the simulation scene if there is no one selected.
  if (nullptr == this->scene)
  {
    this->FindScene();
  }

  // Nothing to do if there is no scene available
  if (nullptr == this->scene)
  {
    ignerr << "No scene available" << std::endl;
    this->materialRequest = false;
    return;
  }

  // Get the visual propriety attached to the LinkName on the message
  auto visual = this->scene->VisualByName(this->linkName);

  // Check if the visual propriety exist
  if (nullptr == visual)
  {
    ignerr << "No visual with the name: "<< this->linkName << std::endl;
    this->materialRequest = false;
    return;
  }

  // Creating the material and setting the Color to Ambient, Diffuse and Emissive proprieties
  ignition::rendering::MaterialPtr material = this->scene->CreateMaterial();
  material->SetAmbient(this->newEmissive);
  material->SetDiffuse(this->newEmissive);
  material->SetEmissive(this->newEmissive);

  // Applying the material to the selected visual link.
  visual->SetMaterial(material);

  this->materialRequest = false;
}

/////////////////////////////////////////////////
/// \brief Encapsulates the logic to find the rendering scene through the
/// render engine singleton.
void EmissivePropertyController::FindScene()
{
  auto loadedEngNames = ignition::rendering::loadedEngines();
  if (loadedEngNames.empty())
  {
    igndbg << "No rendering engine is loaded yet" << std::endl;
    return;
  }

  // assume there is only one engine loaded
  auto engineName = loadedEngNames[0];
  if (loadedEngNames.size() > 1)
  {
    igndbg << "More than one engine is available. "
      << "Using engine [" << engineName << "]" << std::endl;
  }
  auto engine = ignition::rendering::engine(engineName);
  if (!engine)
  {
    ignerr << "Internal error: failed to load engine [" << engineName
      << "]. Grid plugin won't work." << std::endl;
    return;
  }

  if (engine->SceneCount() == 0)
  {
    igndbg << "No scene has been created yet" << std::endl;
    return;
  }

  // Get first scene
  auto scenePtr = engine->SceneByIndex(0);
  if (nullptr == scenePtr)
  {
    ignerr << "Internal error: scene is null." << std::endl;
    return;
  }

  if (engine->SceneCount() > 1)
  {
    igndbg << "More than one scene is available. "
      << "Using scene [" << scene->Name() << "]" << std::endl;
  }

  if (!scenePtr->IsInitialized() || nullptr == scenePtr->RootVisual())
  {
    return;
  }

  this->scene = scenePtr;
}

/////////////////////////////////////////////////
/// \brief Function used to split a string by a delimiter character.
/// \param[in] str std::string to be splited.
/// \param[in] delim char to delimiter the strings separation.
/// \return Returns a Vector of strings. std::vector<std::string>
std::vector<std::string> EmissivePropertyController::SplitMsg(std::string const &str, const char delim)
{
  size_t start;
  size_t end = 0;
  std::vector<std::string> out;

  while ((start = str.find_first_not_of(delim, end)) != std::string::npos)
  {
    end = str.find(delim, start);
    out.push_back(str.substr(start, end - start));
  }
  return out;
}

// Register this plugin
IGNITION_ADD_PLUGIN(EmissivePropertyController,
                    ignition::gui::Plugin)


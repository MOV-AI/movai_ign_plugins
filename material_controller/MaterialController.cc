#include <ignition/plugin/Register.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/Model.hh>
#include <ignition/gazebo/components/ChildLinkName.hh>
#include <ignition/gazebo/components/ParentEntity.hh>
#include <ignition/gazebo/components/World.hh>
#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/gui/GuiEvents.hh>
#include "ignition/gazebo/components/Factory.hh"

#include <ignition/gui/Application.hh>
//! [includeGuiEvents]
#include <ignition/gui/GuiEvents.hh>
//! [includeGuiEvents]
#include <ignition/gui/MainWindow.hh>
#include <ignition/math/Rand.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/rendering/RenderEngine.hh>
#include <ignition/rendering/RenderingIface.hh>
#include <ignition/rendering/Scene.hh>
#include <ignition/rendering/Visual.hh>

#include "MaterialController.hh"

/////////////////////////////////////////////////
MaterialController::MaterialController() = default;

/////////////////////////////////////////////////
MaterialController::~MaterialController() = default;

/////////////////////////////////////////////////
void MaterialController::LoadConfig(const tinyxml2::XMLElement * /*_pluginElem*/)
{
  if (this->title.empty())
    this->title = "GUI system plugin";
  ignition::gui::App()->findChild<
      ignition::gui::MainWindow *>()->installEventFilter(this);
  //! [connectToGuiEvent]
  // Subscribe to commands
  auto topic = ignition::transport::TopicUtils::AsValidTopic("/model/color");
  if (topic.empty())
  {
    ignerr << "Failed to create valid topic ]" << std::endl;
    return;
  }
  this->node.Subscribe(topic, &MaterialController::OnChangeCollor,
                                this);

  ignmsg << "MaterialController subscribing to Double messages on [" << topic
         << "]" << std::endl;
  std::string topicName = "/model/color";
  ignerr << "Subscribing to" << topicName << std::endl;
  // Here you can read configuration from _pluginElem, if it's not null.
}

/////////////////////////////////////////////////
void MaterialController::RandomColor()
{
  this->dirty = true;
}

//////////////////////////////////////////////////
void MaterialController::OnChangeCollor(const ignition::msgs::StringMsg &_msg)
{
  std::vector<std::string> seglist = SplitMsg(_msg.data(), '-');
  if (seglist.size() == 5)
  {
    this->linkName = seglist[0];
    this->newEmissive.R() = std::stof(seglist[1]);
    this->newEmissive.G() = std::stof(seglist[2]);
    this->newEmissive.B() = std::stof(seglist[3]);
    this->newEmissive.A() = std::stof(seglist[4]);
    this->dirty = true;
  }
  else
  {
    ignerr << "No size match" << std::endl;
  }
}

/////////////////////////////////////////////////
//! [eventFilter]
bool MaterialController::eventFilter(QObject *_obj, QEvent *_event)
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
//! [eventFilter]

/////////////////////////////////////////////////
//! [performRenderingOperations]
void MaterialController::PerformRenderingOperations()
{
  if (!this->dirty)
  {
    return;
  }

  if (nullptr == this->scene)
  {
    this->FindScene();
  }

  if (nullptr == this->scene)
    return;
  
  auto visual = this->scene->VisualByName(this->linkName);

  if (nullptr == visual)
  {
    ignerr << "No visual" << std::endl;
  }

  auto targetVis = std::dynamic_pointer_cast<ignition::rendering::Visual>(visual);

  if (targetVis && targetVis->HasUserData("gazebo-entity"))
  {
    ignition::gazebo::Entity targetEntity =
        std::get<int>(targetVis->UserData("gazebo-entity"));

    ignition::rendering::MaterialPtr material = this->scene->CreateMaterial();
    material->SetAmbient(this->newEmissive);
    material->SetDiffuse(this->newEmissive);
    // material->SetSpecular(1.0, 1.0, 1.0);
    material->SetEmissive(this->newEmissive);
    visual->SetMaterial(material);
  }
  else
  {
    ignerr << "Unable to find entity to change color" << std::endl;
  }

  this->dirty = false;
}
//! [performRenderingOperations]

/////////////////////////////////////////////////
void MaterialController::FindScene()
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

//////////////////////////////////////////////////
void MaterialController::Update(const ignition::gazebo::UpdateInfo & _info,
    ignition::gazebo::EntityComponentManager &_ecm)
{
  // In the update loop, you can for example get the name of the world and set
  // it as a property that can be read from the QML.
  if (_info.paused)
    return;
}

/////////////////////////////////////////////////
QString MaterialController::CustomProperty() const
{
  return this->customProperty;
}

/////////////////////////////////////////////////
void MaterialController::SetCustomProperty(const QString &_customProperty)
{
  this->customProperty = _customProperty;
  this->CustomPropertyChanged();
}


/////////////////////////////////////////////////
/// \brief Function used to split a string by a delimiter character.
/// \param[in] str std::string to be splited.
/// \param[in] delim char to delimiter the strings separation.
/// \return Returns a Vector of strings. std::vector<std::string>
std::vector<std::string> MaterialController::SplitMsg(std::string const &str, const char delim)
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
IGNITION_ADD_PLUGIN(MaterialController,
                    ignition::gui::Plugin)


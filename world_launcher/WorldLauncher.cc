#include "WorldLauncher.hh"
#include <ignition/plugin/Register.hh>

using namespace ignition;
using namespace gui;

/////////////////////////////////////////////////
/// \brief Constructor
WorldLauncher::WorldLauncher()
    : Plugin()
{
}

/////////////////////////////////////////////////
/// \brief Destructor
WorldLauncher::~WorldLauncher()
{
}

/////////////////////////////////////////////////
/// \brief Called by Ignition GUI when plugin is instantiated.
/// \param[in] _pluginElem XML configuration for this plugin.
void WorldLauncher::LoadConfig(const tinyxml2::XMLElement *_pluginElem)
{
  if (!_pluginElem)
    return;

  // Get the Local resource var
  char *resources_path;
  std::vector<std::string> seglist;
  resources_path = getenv("IGN_GAZEBO_RESOURCE_PATH");
  std::string resources_path_string(resources_path);
  // Separate all the directories paths into a path vector
  seglist = GetWorldList(resources_path_string, ':');

  for (size_t i = 0; i < seglist.size(); i++)
  {
    // Uses only the worlds path and ignore the models path
    std::size_t found = seglist[i].find("world");
    if (found != std::string::npos)
    {
      // Search for each world file in this folder and create a list with the world names
      for (common::DirIter file(seglist[i]); file != common::DirIter(); ++file)
      {
        std::vector<std::string> worldNameList;
        std::string currentPath(*file);
        // Get the world name file
        worldNameList = GetWorldList(currentPath, '/');
        // Create the List to show to the User in the GUI
        this->worldsList.push_back(QString::fromStdString(worldNameList.back()));
      }
    }
  }
  this->worldsList.sort(Qt::CaseInsensitive);
  // Inform GUI a update in the Q_PROPERTY
  this->WorldsListChanged();
  // Call the function to Load the FUEL Worlds names
  // Set the default value in the selected world
  SetWorld(this->worldsList[0]);

  // Load the default owner
  this->OnOwnerSelection(QString::fromStdString(this->ownerName));
}

/////////////////////////////////////////////////
/// \brief Called by Ignition GUI when QStringList is instantiated.
QStringList WorldLauncher::WorldsList() const
{
  return this->worldsList;
}

/////////////////////////////////////////////////
/// \brief Called by Ignition GUI when QStringList is instantiated.
/// \param[in] _worldsList QStringList to update
void WorldLauncher::SetWorldsList(const QStringList &_worldsList)
{
  this->worldsList = _worldsList;
  this->worldsList.sort(Qt::CaseInsensitive);
  this->WorldsListChanged();
}

/////////////////////////////////////////////////
/// \brief Called by Ignition GUI when finished to filed this textbox.
/// \param[in] _owner the selected string in the textbox.
void WorldLauncher::OnOwnerSelection(const QString &_owner)
{
  this->ownerName = _owner.toStdString();
  this->loadingStatus = true;
  this->fuelWorldsList.clear();
  this->fuelWorldsList.push_back(QString::fromStdString("Loading worlds from Owner. Please wait."));
  this->FuelWorldsListChanged();
  this->LoadingStatusChanged();
  this->LoadFuelList();
}

/////////////////////////////////////////////////
/// \brief Called to search for the Worlds in the FUEL and create a list of it.
void WorldLauncher::LoadFuelList()
{
  std::cout << "Loading worlds from Owner: " + this->ownerName << std::endl;

  // For each server available in the server configuration, create a list of worlds available
  // Thread to not block the app
  std::thread threadLoadFuel([this]
                             {
                  // Setup ClientConfig.
                  ignition::fuel_tools::ClientConfig conf;
                  conf.SetUserAgent("ExampleList");
                  // Instantiate the FuelClient object with the configuration.
                  ignition::fuel_tools::FuelClient client(conf);
                  auto servers = client.Config().Servers();
                  this->fuelWorldsList.clear();
                  for (const auto &server : servers)
                  {
                    for (auto iter = client.Worlds(server); iter; ++iter)
                    {
                      if (iter->Owner() == this->ownerName)
                      {
                        // Insert the world url to the list in GUI
                        this->fuelWorldsList.push_back(QString::fromStdString(iter->UniqueName()));
                      }
                    }
                  }
                  this->fuelWorldsList.sort(Qt::CaseInsensitive);
                  // Inform GUI a update in the Q_PROPERTY
                  this->FuelWorldsListChanged();
                  SetFuelWorld(this->fuelWorldsList[0]);
                  std::cout << "Finished " << std::endl;
                  this->loadingStatus = false;
                  this->LoadingStatusChanged(); });
  threadLoadFuel.detach();
}

/////////////////////////////////////////////////
/// \brief Called by Ignition GUI when QStringList is instantiated.
QStringList WorldLauncher::FuelWorldsList() const
{
  return this->fuelWorldsList;
}

/////////////////////////////////////////////////
/// \brief Called by Ignition GUI when QStringList is instantiated.
/// \param[in] _worldsList QStringList to update
void WorldLauncher::SetFuelWorldsList(const QStringList &_worldsList)
{
  this->fuelWorldsList = _worldsList;
  this->fuelWorldsList.sort(Qt::CaseInsensitive);
  this->FuelWorldsListChanged();
}

/////////////////////////////////////////////////
/// \brief Called by Ignition GUI when QBool is instantiated.
bool WorldLauncher::LoadingStatus() const
{
  return this->loadingStatus;
}

/////////////////////////////////////////////////
/// \brief Called by Ignition GUI when QBool is instantiated.
/// \param[in] _status QBool to update
void WorldLauncher::SetLoadingStatus(const bool _status)
{
  this->loadingStatus = _status;
  this->LoadingStatusChanged();
}

/////////////////////////////////////////////////
/// \brief Called by Ignition GUI when QBool is instantiated.
bool WorldLauncher::SimulationStatus() const
{
  return this->simulationStatus;
}

/////////////////////////////////////////////////
/// \brief Called by Ignition GUI when QBool is instantiated.
/// \param[in] _status QBool to update
void WorldLauncher::SetSimulationStatus(const bool _status)
{
  this->simulationStatus = _status;
  this->SimulationStatusChanged();
}

/////////////////////////////////////////////////
/// \brief Function used to split a string by a delimiter character.
/// \param[in] str std::string to be splited.
/// \param[in] delim char to delimiter the strings separation.
/// \return Returns a Vector of strings. std::vector<std::string>
std::vector<std::string> WorldLauncher::GetWorldList(std::string const &str, const char delim)
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

/////////////////////////////////////////////////
/// \brief Called by Ignition GUI when there is a change in the combobox of the Local World.
/// \param[in] _sortType the selected string in the combobox.
void WorldLauncher::SetWorld(const QString &_sortType)
{
  // Print chosen world
  std::cout << "Choosen world: " + _sortType.toStdString() << std::endl;
  // Assin to propery
  this->worldName = _sortType.toStdString();
}

/////////////////////////////////////////////////
/// \brief Called by Ignition GUI when there is a change in the combobox of the Fuel World.
/// \param[in] _sortType the selected string in the combobox.
void WorldLauncher::SetFuelWorld(const QString &_sortType)
{
  // Print chosen world
  std::cout << "Choosen world: " + _sortType.toStdString() << std::endl;
  this->fuelWorldName = _sortType.toStdString();
}

/////////////////////////////////////////////////
/// \brief Called by Ignition GUI when click on the start button.
void WorldLauncher::OnButton()
{
  // Print Full command
  std::string full_exec = std::string("ign gazebo ") + std::string(this->worldName) + std::string(" -r -v 4 &");
  std::cout << "Executing: " + full_exec << std::endl;
  // Launch the selected world
  std::string result = this->StartSimulator(full_exec);
  std::cout << "Simulation result " + result << std::endl;
}

/////////////////////////////////////////////////
/// \brief Called by Ignition GUI when click on the start button.
void WorldLauncher::OnFuelButton()
{
  // Print Full command
  std::string full_exec = std::string("ign gazebo -r -v 4 'https://") + std::string(this->fuelWorldName) + std::string("' &");
  std::cout << "Downloading and Executing: " + full_exec << std::endl;
  // Launch the selected world
  std::string result = this->StartSimulator(full_exec);
  std::cout << "Simulation result " + result << std::endl;
}

/////////////////////////////////////////////////
/// \brief Starts the simulator using POPEN function.
/// \param[in] full_exec string to start the POPEN cmd.
/// \return Returns string of the buffer used.
std::string WorldLauncher::StartSimulator(const std::string &_cmd)
{
  // Launch the selected world using popen
  std::string full_exec = _cmd;
  this->simulationResult = "";
  std::thread threadStartSimulator([this, full_exec]
                                   {
                                char buffer[128];
                                this->simulationStatus = false;
                                this->SimulationStatusChanged();

                                FILE *pipe = popen(full_exec.c_str(), "r");

                                if (!pipe)
                                  return "ERROR";

                                while (!feof(pipe))
                                {
                                  if (fgets(buffer, 128, pipe) != nullptr)
                                  {
                                    this->simulationResult += buffer;
                                  }
                                }
                                pclose(pipe); 
                                this->simulationStatus = true;
                                this->SimulationStatusChanged(); 
                                std::cout << "Simulation result " + this->simulationResult << std::endl; });
  threadStartSimulator.detach();
  return this->simulationResult;
}

// Register this plugin
IGNITION_ADD_PLUGIN(ignition::gui::WorldLauncher,
                    ignition::gui::Plugin);

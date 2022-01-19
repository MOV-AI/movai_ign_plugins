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
  char const *resources_path = getenv("IGN_GAZEBO_RESOURCE_PATH");
  std::vector<std::string> seglist;

  // Check if the IGN_GAZEBO_RESOURCE_PATH is not empty
  if (resources_path != NULL)
  {
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
    // Check if it found worlds
    if (this->worldsList.isEmpty())
    {
      this->worldsList.push_back(QString::fromStdString("There are no local worlds available"));
      //  Blocks the Start Button
      this->validLocalWorld = false;
      this->ValidLocalWorldChanged();
    }
    else
    {
      this->worldsList.sort(Qt::CaseInsensitive);
      // Set the default value in the selected world
      SetWorld(this->worldsList[0]);
      // unblocks the Start Button
      this->validLocalWorld = true;
      this->ValidLocalWorldChanged();
    }
  }
  else
  {
    this->worldsList.push_back(QString::fromStdString("IGN_GAZEBO_RESOURCE_PATH is empty"));
    //  Blocks the Start Button
    this->validLocalWorld = false;
    this->ValidLocalWorldChanged();
  }

  // Inform GUI a update in the Q_PROPERTY
  this->WorldsListChanged();

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
                              //  Check if there is worlds of this owner in FUEL
                               if (this->fuelWorldsList.isEmpty())
                               {
                                 this->fuelWorldsList.push_back(QString::fromStdString("This Owner does not have worlds on Fuel available"));
                                //  Blocks the Start Button
                                 this->validFuelWorld = false;
                                 this->ValidFuelWorldChanged();
                               }
                               else
                               {
                                //  Sort by name
                                 this->fuelWorldsList.sort(Qt::CaseInsensitive);
                                 SetFuelWorld(this->fuelWorldsList[0]);
                                 this->validFuelWorld = true;
                                 this->ValidFuelWorldChanged();
                               }
                               // Inform GUI a update in the Q_PROPERTY
                               this->FuelWorldsListChanged();
                               this->loadingStatus = false;
                               this->LoadingStatusChanged();
                               std::cout << "Finished " << std::endl; });
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
/// \brief Called by Ignition GUI when QBool is instantiated.
bool WorldLauncher::ValidFuelWorld() const
{
  return this->validFuelWorld;
}

/////////////////////////////////////////////////
/// \brief Called by Ignition GUI when QBool is instantiated.
/// \param[in] _status QBool to update
void WorldLauncher::SetValidFuelWorld(const bool _status)
{
  this->validFuelWorld = _status;
  this->ValidFuelWorldChanged();
}

/////////////////////////////////////////////////
/// \brief Called by Ignition GUI when QBool is instantiated.
bool WorldLauncher::ValidLocalWorld() const
{
  return this->validLocalWorld;
}

/////////////////////////////////////////////////
/// \brief Called by Ignition GUI when QBool is instantiated.
/// \param[in] _status QBool to update
void WorldLauncher::SetValidLocalWorld(const bool _status)
{
  this->validLocalWorld = _status;
  this->ValidLocalWorldChanged();
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
/// \brief Called by Ignition GUI when click on the start button.
void WorldLauncher::OnCreateButton()
{
  // Print Full command
  std::string full_exec = std::string("ign gazebo empty.sdf -v4 &");
  std::cout << "Empty World " << std::endl;
  // Launch the selected world
  std::string result = this->StartSimulator(full_exec);
  std::cout << "Logs result " + result << std::endl;
}

/////////////////////////////////////////////////
/// \brief Starts the simulator using POPEN function.
/// \param[in] _cmd string to start the POPEN cmd.
/// \return Returns string of the buffer used.
std::string WorldLauncher::StartSimulator(const std::string &_cmd)
{
  std::string full_exec = _cmd;
  this->simulationResult = "";
  // Start a thread to avoid block the app during the Simulation
  std::thread threadStartSimulator([this, full_exec]
                                   {
                                char buffer[128];
                                // Blocks the Start button to avoid duplicated simulations
                                this->simulationStatus = false;
                                this->SimulationStatusChanged();
                                // Launch the selected world using popen
                                FILE *processSim = popen(full_exec.c_str(), "r");

                                if (!processSim)
                                  return "ERROR";
                                // Collect the results of the simulation
                                while (!feof(processSim))
                                {
                                  if (fgets(buffer, 128, processSim) != nullptr)
                                  {
                                    this->simulationResult += buffer;
                                  }
                                }
                                pclose(processSim);
                                // Unblock the Start button
                                this->simulationStatus = true;
                                this->SimulationStatusChanged(); 
                                std::cout << "Simulation result " + this->simulationResult << std::endl; });
  threadStartSimulator.detach();
  return this->simulationResult;
}

// Register this plugin
IGNITION_ADD_PLUGIN(ignition::gui::WorldLauncher,
                    ignition::gui::Plugin);

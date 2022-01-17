#ifndef WORLDLAUNCHER_HH_
#define WORLDLAUNCHER_HH_

#include <iostream>

#include <ignition/gui/Plugin.hh>
#include <ignition/common/Filesystem.hh>
#include <ignition/fuel_tools.hh>
#include <thread>

namespace ignition
{
  namespace gui
  {
    class WorldLauncher : public Plugin
    {
      Q_OBJECT

      /// \brief Fuel Worlds list GUI QT object.
      Q_PROPERTY(
          QStringList fuelWorldsList
              READ FuelWorldsList
                  WRITE SetFuelWorldsList
                      NOTIFY FuelWorldsListChanged)

      /// \brief Worlds list GUI QT object.
      Q_PROPERTY(
          QStringList worldsList
              READ WorldsList
                  WRITE SetWorldsList
                      NOTIFY WorldsListChanged)

      /// \brief Loading status GUI QT object.
      Q_PROPERTY(
          bool loadingStatus
              READ LoadingStatus
                  WRITE SetLoadingStatus
                      NOTIFY LoadingStatusChanged)

      /// \brief Simulation Running status GUI QT object.
      Q_PROPERTY(
          bool simulationStatus
              READ SimulationStatus
                  WRITE SetSimulationStatus
                      NOTIFY SimulationStatusChanged)

      /// \brief Constructor.
    public:
      WorldLauncher();

      /// \brief Destructor.
      virtual ~WorldLauncher();

      /////////////////////////////////////////////////
      /// \brief Called by Ignition GUI when plugin is instantiated.
      /// \param[in] _pluginElem XML configuration for this plugin.
      virtual void LoadConfig(const tinyxml2::XMLElement *_pluginElem)
          override;

      /////////////////////////////////////////////////
      /// \brief Called by Ignition GUI when QStringList is instantiated.
      Q_INVOKABLE QStringList WorldsList() const;

      /////////////////////////////////////////////////
      /// \brief Called by Ignition GUI when QStringList is instantiated.
      /// \param[in] _worldsList QStringList to update
      Q_INVOKABLE void SetWorldsList(const QStringList &_worldsList);

      /////////////////////////////////////////////////
      /// \brief Called by Ignition GUI when QStringList is instantiated.
      Q_INVOKABLE QStringList FuelWorldsList() const;

      /////////////////////////////////////////////////
      /// \brief Called by Ignition GUI when QStringList is instantiated.
      /// \param[in] _worldsList QStringList to update
      Q_INVOKABLE void SetFuelWorldsList(const QStringList &_worldsList);

      /////////////////////////////////////////////////
      /// \brief Called by Ignition GUI when QBool is instantiated.
      Q_INVOKABLE bool LoadingStatus() const;

      /////////////////////////////////////////////////
      /// \brief Called by Ignition GUI when QBool is instantiated.
      /// \param[in] _status QBool to update
      Q_INVOKABLE void SetLoadingStatus(const bool _status);

      /////////////////////////////////////////////////
      /// \brief Called by Ignition GUI when QBool is instantiated.
      Q_INVOKABLE bool SimulationStatus() const;

      /////////////////////////////////////////////////
      /// \brief Called by Ignition GUI when QBool is instantiated.
      /// \param[in] _status QBool to update
      Q_INVOKABLE void SetSimulationStatus(const bool _status);

      /// \brief Selected local world file.
      std::string worldName{"empty.sdf"};

      /// \brief Selected Fuel world uri.
      std::string fuelWorldName{"empty"};

      /// \brief Selected Owner of worlds on Fuel.
      std::string ownerName{"openrobotics"};

      std::string simulationResult{""};

      /// \brief List of worlds in local.
      QStringList worldsList;

      /// \brief List of worlds in Fuel.
      QStringList fuelWorldsList;

      /// \brief Status of loading.
      bool loadingStatus{false};

      /// \brief Status of Simulation.
      bool simulationStatus{true};

    signals:
      /// \brief Notify that the worlds list has changed
      void WorldsListChanged();

      /// \brief Notify that the Fuel worlds list has changed
      void FuelWorldsListChanged();

      /// \brief Notify that the Loading status has changed
      void LoadingStatusChanged();

      /// \brief Notify that the Simulation status has changed
      void SimulationStatusChanged();

    protected slots:
      /////////////////////////////////////////////////
      /// \brief Called to search for the Worlds in the FUEL and create a list of it.
      void LoadFuelList();

      /////////////////////////////////////////////////
      /// \brief Called by Ignition GUI when click on the start button.
      void OnButton();

      /////////////////////////////////////////////////
      /// \brief Called by Ignition GUI when click on the start button.
      void OnFuelButton();

      /////////////////////////////////////////////////
      /// \brief Called by Ignition GUI when finished to filed this textbox.
      /// \param[in] _owner the selected string in the textbox.
      void OnOwnerSelection(const QString &_owner);

      /////////////////////////////////////////////////
      /// \brief Called by Ignition GUI when there is a change in the combobox of the Local World.
      /// \param[in] _sortType the selected string in the combobox.
      void SetWorld(const QString &_sortType);

      /////////////////////////////////////////////////
      /// \brief Called by Ignition GUI when there is a change in the combobox of the Fuel World.
      /// \param[in] _sortType the selected string in the combobox.
      void SetFuelWorld(const QString &_sortType);

    private:
      /////////////////////////////////////////////////
      /// \brief Function used to split a string by a delimiter character.
      /// \param[in] str std::string to be splited.
      /// \param[in] delim char to delimiter the strings separation.
      /// \return Returns a Vector of strings. std::vector<std::string>
      std::vector<std::string> GetWorldList(std::string const &str, const char delim);

      /////////////////////////////////////////////////
      /// \brief Starts the simulator using POPEN function.
      /// \param[in] full_exec string to start the POPEN cmd.
      /// \return Returns string of the buffer used.
      std::string StartSimulator(const std::string &full_exec);
    };
  }
}

#endif

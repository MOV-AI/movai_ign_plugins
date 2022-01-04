/*
 * Copyright (C) 2017 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#ifndef WORLDLAUNCHER_HH_
#define WORLDLAUNCHER_HH_

#include <iostream>

#include <ignition/gui/Plugin.hh>
#include <ignition/common/Filesystem.hh>
#include <ignition/fuel_tools.hh>

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

      /// \brief Selected local world file.
      std::string worldName{"empty.sdf"};

      /// \brief Selected Fuel world uri.
      std::string fuelWorldName{"empty"};

      /// \brief Selected Owner of worlds on Fuel.
      std::string ownerName{"openrobotics"};

      /// \brief List of worlds in local.
      QStringList worldsList;

      /// \brief List of worlds in Fuel.
      QStringList fuelWorldsList;

    signals:
      /// \brief Notify that the worlds list has changed
      void WorldsListChanged();

      /// \brief Notify that the Fuel worlds list has changed
      void FuelWorldsListChanged();

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
    };
  }
}

#endif

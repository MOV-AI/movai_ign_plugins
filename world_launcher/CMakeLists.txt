cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)
project(movai_ign_plugins)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
if(POLICY CMP0100)
  cmake_policy(SET CMP0100 NEW)
endif()

set (CMAKE_AUTOMOC ON)

# Find Qt5
find_package (Qt5
  COMPONENTS
    Core
    Quick
    QuickControls2
  REQUIRED
)

# Find the Ignition gui library
find_package(ignition-gui6 REQUIRED)
find_package(ignition-common4 REQUIRED)
find_package(ignition-fuel_tools7 REQUIRED)

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${IGNITION-GUI_CXX_FLAGS} ${IGNITION-FUEL_TOOLS_CXX_FLAGS}")

# Set Mov.Ai Ignition docker plugins library directory
# set(CMAKE_LIBRARY_OUTPUT_DIRECTORY /movai_ign_plugins/gui)

QT5_ADD_RESOURCES(resources_RCC WorldLauncher.qrc)

include_directories(SYSTEM
  ${IGNITION-COMMON_INCLUDE_DIRS}
  ${IGNITION-GUI_INCLUDE_DIRS}
  ${IGNITION-FUEL_TOOLS_INCLUDE_DIRS}
  ${Qt5Core_INCLUDE_DIRS}
  ${Qt5Qml_INCLUDE_DIRS}
  ${Qt5Quick_INCLUDE_DIRS}
  ${Qt5QuickControls2_INCLUDE_DIRS}
)

link_directories(
  ${IGNITION-COMMON_LIBRARY_DIRS}
  ${IGNITION-GUI_LIBRARY_DIRS}
  ${IGNITION-FUEL_TOOLS_LIBRARY_DIRS}
)

# Generate Library
add_library(WorldLauncher SHARED ${headers_MOC}
  WorldLauncher.cc
  ${resources_RCC}
)
target_link_libraries(WorldLauncher
  ${IGNITION-GUI_LIBRARIES}
  ${IGNITION-FUEL_TOOLS_LIBRARIES}
  ${Qt5Core_LIBRARIES}
  ${Qt5Qml_LIBRARIES}
  ${Qt5Quick_LIBRARIES}
  ${Qt5QuickControls2_LIBRARIES}
)

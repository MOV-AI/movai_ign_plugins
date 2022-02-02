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
find_package(ignition-gazebo6 REQUIRED COMPONENTS gui)
find_package(ignition-transport11 REQUIRED)

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${IGNITION-GUI_CXX_FLAGS} ${IGNITION-FUEL_TOOLS_CXX_FLAGS}")

# Set Mov.Ai Ignition docker plugins library directory
# set(CMAKE_LIBRARY_OUTPUT_DIRECTORY /movai_ign_plugins/gui)

QT5_ADD_RESOURCES(resources_RCC EmergencyButton.qrc)

include_directories(SYSTEM
  ${IGNITION-GUI_INCLUDE_DIRS}
  ${IGNITION-GAZEBO_INCLUDE_DIRS}
  ${IGNITION-TRANSPORT_INCLUDE_DIRS}
  ${Qt5Core_INCLUDE_DIRS}
  ${Qt5Qml_INCLUDE_DIRS}
  ${Qt5Quick_INCLUDE_DIRS}
  ${Qt5QuickControls2_INCLUDE_DIRS}
)

link_directories(
  ${IGNITION-GUI_LIBRARY_DIRS}
  ${IGNITION-GAZEBO_LIBRARY_DIRS}
  ${IGNITION-TRANSPORT_LIBRARY_DIRS}
)

# Generate Library
add_library(EmergencyButton SHARED ${headers_MOC}
  EmergencyButton.cc
  ${resources_RCC}
)
target_link_libraries(EmergencyButton
  ${IGNITION-GUI_LIBRARIES}
  ${IGNITION-GAZEBO_LIBRARIES}
  ${IGNITION-TRANSPORT_LIBRARIES}
  ${Qt5Core_LIBRARIES}
  ${Qt5Qml_LIBRARIES}
  ${Qt5Quick_LIBRARIES}
  ${Qt5QuickControls2_LIBRARIES}
)

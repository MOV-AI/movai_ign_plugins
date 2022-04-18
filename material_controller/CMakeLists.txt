cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)
project(movai_ign_plugins)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
if(POLICY CMP0100)
  cmake_policy(SET CMP0100 NEW)
endif()

set (CMAKE_AUTOMOC ON)

find_package(ignition-gazebo6 REQUIRED COMPONENTS gui)
find_package(ignition-rendering6 REQUIRED)

# Set Mov.Ai Ignition docker plugins library directory
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY /movai_ign_plugins/gui)

QT5_ADD_RESOURCES(resources_RCC MaterialController.qrc)

add_library(MaterialController SHARED
  MaterialController.cc
  ${resources_RCC}
)
target_link_libraries(MaterialController
  PRIVATE 
    ignition-gazebo6::gui
    ignition-rendering6::ignition-rendering6
)
cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)
project(movai_ign_plugins)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(POLICY CMP0100)
  cmake_policy(SET CMP0100 NEW)
endif()

set (CMAKE_AUTOMOC ON)

find_package(ignition-plugin1 REQUIRED COMPONENTS register)
find_package(ignition-gazebo6 REQUIRED)
find_package(ignition-transport11 REQUIRED)

# Set Mov.Ai Ignition docker plugins library directory
 #set(CMAKE_LIBRARY_OUTPUT_DIRECTORY /usr/lib/x86_64-linux-gnu/ign-gazebo-6/plugins)

add_library(SpawnMotionModel SHARED
  SpawnMotionModel.cc
)
target_link_libraries(SpawnMotionModel
  PRIVATE 
    ignition-plugin1::ignition-plugin1
    ignition-gazebo6::ignition-gazebo6
    ignition-rendering6::ignition-rendering6
    ignition-transport11::ignition-transport11
)
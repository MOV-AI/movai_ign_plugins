cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)
project(movai_ign_plugins)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
if(POLICY CMP0100)
  cmake_policy(SET CMP0100 NEW)
endif()

set (CMAKE_AUTOMOC ON)

find_package(ignition-plugin1 REQUIRED COMPONENTS register)
find_package(ignition-gazebo6 REQUIRED)
find_package(ignition-transport11 REQUIRED)
find_package(Eigen3 REQUIRED)

# Set Mov.Ai Ignition docker plugins library directory
# set(CMAKE_LIBRARY_OUTPUT_DIRECTORY /movai_ign_plugins/system)

add_library(ChargingStation SHARED
  ChargingStation.cc
)
target_link_libraries(ChargingStation
  PRIVATE 
    ignition-plugin1::ignition-plugin1
    ignition-gazebo6::ignition-gazebo6
    Eigen3::Eigen

)
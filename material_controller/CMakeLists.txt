cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)
project(movai_ign_plugins)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
if(POLICY CMP0100)
  cmake_policy(SET CMP0100 NEW)
endif()

set (CMAKE_AUTOMOC ON)

# Find the Ignition gui library
find_package(ignition-gazebo6 REQUIRED COMPONENTS gui)
find_package(ignition-transport11 REQUIRED)
find_package(ignition-common4 REQUIRED)

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")

# Set Mov.Ai Ignition docker plugins library directory
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY /movai_ign_plugins/system)

include_directories(SYSTEM
  ${IGNITION-GAZEBO_INCLUDE_DIRS}
  ${IGNITION-TRANSPORT_INCLUDE_DIRS}
  ${IGNITION-COMMON_INCLUDE_DIRS}
)

link_directories(
  ${IGNITION-GAZEBO_LIBRARY_DIRS}
  ${IGNITION-TRANSPORT_LIBRARY_DIRS}
  ${IGNITION-COMMON_LIBRARY_DIRS}
)

# Generate Library
add_library(MaterialController SHARED ${headers_MOC}
  MaterialController.cc
  ${resources_RCC}
)
target_link_libraries(MaterialController
  ${IGNITION-GUI_LIBRARIES}
  ${IGNITION-GAZEBO_LIBRARIES}
  ${IGNITION-TRANSPORT_LIBRARIES}
  ${IGNITION-COMMON_LIBRARIES}
)

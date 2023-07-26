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
find_package(ignition-sensors6 REQUIRED)
find_package(ignition-rendering6 REQUIRED)
find_package(ignition-cmake2 2.13 REQUIRED)
find_package(ignition-msgs8 REQUIRED)

# ign_find_package(ignition-msgs8 REQUIRED VERSION 8.2)
# set(IGN_MSGS_VER ${ignition-msgs8_VERSION_MAJOR})
# ign_find_package(ignition-transport11 REQUIRED)
# set(IGN_TRANSPORT_VER ${ignition-transport11_VERSION_MAJOR})
# # Set Mov.Ai Ignition docker plugins library directory
#  set(IGN_LIB_INSTALL_DIR /usr/lib/x86_64-linux-gnu/ign-gazebo-6/plugins)

add_library(MovaiLidar SHARED
  MovaiLidar.cc
)
target_link_libraries(MovaiLidar
  PUBLIC 
    ignition-plugin1::ignition-plugin1
    ignition-gazebo6::ignition-gazebo6
    ignition-rendering6::ignition-rendering6
    ignition-sensors6::ignition-sensors6
    ignition-msgs8::ignition-msgs8
)

# set(gpu_lidar_sources MovaiLidar.cc)
# ign_add_component(movai_lidar
#   DEPENDS_ON_COMPONENTS lidar
#   SOURCES ${gpu_lidar_sources}
#   GET_TARGET_NAME gpu_lidar_target
# )
# target_compile_definitions(${gpu_lidar_target} PUBLIC GpuLidarSensor_EXPORTS)
# target_link_libraries(${gpu_lidar_target}
#   PRIVATE
#     ignition-msgs${IGN_MSGS_VER}::ignition-msgs${IGN_MSGS_VER}
#     ignition-transport${IGN_TRANSPORT_VER}::ignition-transport${IGN_TRANSPORT_VER}
#     ignition-sensors6::ignition-sensors6
# )
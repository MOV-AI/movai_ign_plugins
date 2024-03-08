cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)
project(movai_ign_plugins)

if(POLICY CMP0100)
  cmake_policy(SET CMP0100 NEW)
endif()
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set (CMAKE_AUTOMOC ON)

set(DEFAULT_BUILD_TYPE "RelWithDebInfo")
if (NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    # Use the default build type if not specified
    message(STATUS "Setting build type to '${DEFAULT_BUILD_TYPE}' as none was specified.")
    set(CMAKE_BUILD_TYPE "${DEFAULT_BUILD_TYPE}" CACHE
        STRING "Choose the type of build." FORCE)
    # Set the possible values of build type for cmake-gui
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
        "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
    if(CMAKE_BUILD_TYPE EQUAL "RelWithDebInfo")
      set(CMAKE_CXX_FLAGS "-pg ${CMAKE_CXX_FLAGS}") #Only if debug mode
    endif()
    
endif()

include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

include(GoogleTest)


find_package(ignition-plugin1 REQUIRED COMPONENTS register)
find_package(ignition-gazebo6 REQUIRED)
find_package(ignition-transport11 REQUIRED)
find_package(ignition-sensors6 REQUIRED)
find_package(ignition-rendering6 REQUIRED)
find_package(ignition-cmake2 2.13 REQUIRED)
find_package(ignition-msgs8 REQUIRED)
find_package(OpenCV REQUIRED)
find_package( PCL REQUIRED )

include_directories( ${OpenCV_INCLUDE_DIRS} )
include_directories( ${PCL_INCLUDE_DIRS} )

add_library(MovaiLidar SHARED
  MovaiLidar.cc
)
add_executable(MovaiLidar_test 
  MovaiLidar_test.cc
)
target_link_libraries(MovaiLidar_test
  PUBLIC 
    ignition-plugin1::ignition-plugin1
    ignition-gazebo6::ignition-gazebo6
    ignition-rendering6::ignition-rendering6
    ignition-sensors6::ignition-sensors6
    ignition-msgs8::ignition-msgs8
    ${OpenCV_LIBS}
    GTest::gtest_main

)
target_link_libraries(MovaiLidar
  PUBLIC 
    ignition-plugin1::ignition-plugin1
    ignition-gazebo6::ignition-gazebo6
    ignition-rendering6::ignition-rendering6
    ignition-sensors6::ignition-sensors6
    ignition-msgs8::ignition-msgs8
    ${OpenCV_LIBS}

)
gtest_discover_tests(MovaiLidar_test)
add_definitions(${PCL_DEFINITIONS})

option(ENABLE_PROFILER "Enable Ignition Profiler" FALSE)

if(ENABLE_PROFILER)
  add_definitions("-DIGN_PROFILER_ENABLE=1")
else()
  add_definitions("-DIGN_PROFILER_ENABLE=0")
endif()

if (UNIX AND NOT APPLE)
  set (EXTRA_TEST_LIB_DEPS stdc++fs)
else()
  set (EXTRA_TEST_LIB_DEPS)
endif()
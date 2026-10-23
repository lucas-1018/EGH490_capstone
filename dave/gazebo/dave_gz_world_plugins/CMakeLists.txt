cmake_minimum_required(VERSION 3.8)

#Define the project name
project(dave_gz_world_plugins)

# Suppress the parameter passing ABI warning
add_compile_options(-Wno-psabi)

# Define BOOST_BIND_GLOBAL_PLACEHOLDERS to suppress Boost deprecation warning
add_definitions(-DBOOST_BIND_GLOBAL_PLACEHOLDERS)

find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(gz-cmake3 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gz-common5 REQUIRED COMPONENTS profiler)
find_package(gz-sim8 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(dave_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Protobuf REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(ament_index_cpp REQUIRED)

# Set version variables with gazebo Harmonic
set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})
set(GZ_COMMON_VER ${gz-common5_VERSION_MAJOR})
set(GZ_SIM_VER ${gz-sim8_VERSION_MAJOR})
set(GZ_MSGS_VER ${gz-msgs10_VERSION_MAJOR})
message(STATUS "compiling against Gazebo Harmonic")

# Define a variable 'GZ_MSGS_VER' holding the version number
set(GZ_MSGS_VER ${gz-msgs10_VERSION_MAJOR})

# Define a variable 'MSGS_PROTOS' listing the .proto files
# cmake-lint: disable=C0301
set(MSGS_PROTOS
    ${CMAKE_CURRENT_SOURCE_DIR}/proto/dave_gz_world_plugins_msgs/msgs/StratifiedCurrentVelocity.proto
)

gz_msgs_generate_messages(
  TARGET msgs
  PROTO_PACKAGE "dave_gz_world_plugins_msgs.msgs"
  MSGS_PATH ${CMAKE_CURRENT_SOURCE_DIR}/proto
  MSGS_PROTOS ${MSGS_PROTOS}
  DEPENDENCIES gz-msgs${GZ_MSGS_VER}::gz-msgs${GZ_MSGS_VER}
)

add_library(OceanCurrentWorldPlugin SHARED
  src/OceanCurrentWorldPlugin.cc
)
add_library(gauss_markov_process SHARED
  src/gauss_markov_process.cc
)
add_library(tidal_oscillation SHARED
  src/tidal_oscillation.cc
)

target_include_directories(OceanCurrentWorldPlugin
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    include ${dave_interfaces_INCLUDE_DIRS}
)
target_include_directories(gauss_markov_process
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)
target_include_directories(tidal_oscillation
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(OceanCurrentWorldPlugin
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
  gauss_markov_process
  tidal_oscillation
  dave_gz_world_plugins-msgs
)
target_link_libraries(gauss_markov_process
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
)
target_link_libraries(tidal_oscillation
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
)

ament_target_dependencies(OceanCurrentWorldPlugin
  rclcpp
  std_msgs
  geometry_msgs
  dave_interfaces
  ament_index_cpp
  gz-msgs${GZ_MSGS_VER}
)

ament_target_dependencies(gauss_markov_process
  rclcpp
  std_msgs
  geometry_msgs
  dave_interfaces
)

ament_target_dependencies(tidal_oscillation
  rclcpp
  std_msgs
  geometry_msgs
  dave_interfaces
)

# Install targets
install(
  TARGETS
    OceanCurrentWorldPlugin
    gauss_markov_process
    tidal_oscillation
    dave_gz_world_plugins-msgs
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Install headers
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  gz-cmake3
  gz-plugin2
  gz-msgs10
  gz-common5
  gz-sim8
  geometry_msgs
  sensor_msgs
  std_msgs
  dave_interfaces
  rclcpp
)

# Environment hooks
ament_environment_hooks(
  "${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")

# Testing setup
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Configure ament
ament_package()

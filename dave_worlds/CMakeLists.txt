cmake_minimum_required(VERSION 3.5)
project(dave_worlds)

# Find dependencies
find_package(ament_cmake REQUIRED)

install(
  DIRECTORY worlds media
  DESTINATION share/${PROJECT_NAME}
)

ament_environment_hooks(
  "${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")

ament_package()

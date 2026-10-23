cmake_minimum_required(VERSION 3.8)
project(dave_demos)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch
  DESTINATION share/dave_demos
)

ament_package()

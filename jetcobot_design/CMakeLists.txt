cmake_minimum_required(VERSION 3.8)
project(jetcobot_design)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch meshes urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

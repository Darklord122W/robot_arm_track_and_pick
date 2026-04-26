cmake_minimum_required(VERSION 3.5)
project(xarm_moveit_config)

# Only need ament_cmake to install config + launch files
find_package(ament_cmake REQUIRED)

# Install the config and launch directories so other nodes can find them
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

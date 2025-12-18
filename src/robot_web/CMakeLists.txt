cmake_minimum_required(VERSION 3.5)
project(robot_web)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install Python scripts
install(PROGRAMS
  scripts/slam_controller.py
  scripts/web_server.py
  scripts/camera_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  www
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

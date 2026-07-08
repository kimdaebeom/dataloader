cmake_minimum_required(VERSION 3.0.2)
project(dataloader)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  rosgraph_msgs
)

catkin_python_setup()

catkin_package()

catkin_install_python(PROGRAMS
  scripts/dataloader_convert.py
  scripts/dataloader_player_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  config
  docs
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


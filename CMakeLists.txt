cmake_minimum_required(VERSION 3.8)
project(dataloader)

if("$ENV{ROS_VERSION}" STREQUAL "2")
  find_package(ament_cmake REQUIRED)
  find_package(ament_cmake_python REQUIRED)

  ament_python_install_package(${PROJECT_NAME} PACKAGE_DIR src/${PROJECT_NAME})

  install(PROGRAMS
    scripts/dataloader_player_node.py
    DESTINATION lib/${PROJECT_NAME}
    RENAME dataloader_player
  )
  install(PROGRAMS
    scripts/dataloader_convert.py
    DESTINATION lib/${PROJECT_NAME}
    RENAME dataloader_convert
  )
  install(PROGRAMS
    scripts/convert_sequences.sh
    DESTINATION lib/${PROJECT_NAME}
  )
  install(DIRECTORY
    config
    docs
    launch
    DESTINATION share/${PROJECT_NAME}
  )

  ament_package()
else()
  find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    rosgraph_msgs
    rospy
    sensor_msgs
  )

  catkin_python_setup()
  catkin_package()

  catkin_install_python(PROGRAMS
    scripts/dataloader_convert.py
    scripts/dataloader_player_node.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
  install(PROGRAMS
    scripts/convert_sequences.sh
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
  install(DIRECTORY
    config
    docs
    launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
endif()

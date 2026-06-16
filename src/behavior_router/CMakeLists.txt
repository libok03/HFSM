cmake_minimum_required(VERSION 3.10.2)
project(behavior_router)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/behavior_router_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

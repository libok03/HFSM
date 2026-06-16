cmake_minimum_required(VERSION 3.10.2)
project(hfsm_dummy_pub)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  nav_msgs
  geometry_msgs
  autoware_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/dummy_publisher_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

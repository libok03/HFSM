cmake_minimum_required(VERSION 3.10.2)
project(hfsm_decision_maker)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  nav_msgs
  geometry_msgs
  autoware_msgs
  smach
  smach_ros
  tf
)

catkin_package()

catkin_install_python(PROGRAMS
  src/hfsm_node.py
  src/behavior_command_node.py
  src/global_path_publisher_node.py

  src/evaluators/lanelet2_event_evaluator_node.py
  src/evaluators/obstacle_evaluator_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

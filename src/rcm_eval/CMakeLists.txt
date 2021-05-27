cmake_minimum_required(VERSION 3.0.2)
project(rcm_eval)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rcm_msgs
  rospy
  std_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy geometry_msgs rcm_msgs std_msgs sensor_msgs
)

# Install
catkin_install_python(
  PROGRAMS python/rcm_eval_node.py python/rcm_deviation_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

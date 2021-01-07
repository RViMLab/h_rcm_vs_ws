cmake_minimum_required(VERSION 3.0.2)
project(rcom_eval)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rcom_msgs
  rospy
  std_msgs
)

# Install
catkin_install_python(
  PROGRAMS python/rcom_eval.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

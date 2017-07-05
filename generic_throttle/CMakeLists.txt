cmake_minimum_required(VERSION 2.8.3)
project(generic_throttle)

find_package(catkin REQUIRED COMPONENTS cv_bridge)

catkin_python_setup()

catkin_package()

install(PROGRAMS scripts/generic_throttle_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

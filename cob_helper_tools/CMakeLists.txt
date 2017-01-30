cmake_minimum_required(VERSION 2.8.3)
project(cob_helper_tools)

find_package(catkin REQUIRED)

catkin_package()

#############
## Install ##
#############

install(PROGRAMS
  scripts/auto_recover.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

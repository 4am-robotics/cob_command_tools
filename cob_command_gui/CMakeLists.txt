cmake_minimum_required(VERSION 2.8.3)
project(cob_command_gui)

find_package(catkin REQUIRED COMPONENTS)

catkin_python_setup()

catkin_package()

### Install ###
catkin_install_python(PROGRAMS src/knoeppkes.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY common
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

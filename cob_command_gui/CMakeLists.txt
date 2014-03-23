# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(cob_command_gui)


find_package(catkin REQUIRED COMPONENTS)

catkin_package(
    CATKIN_DEPENDS
    DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)

install(PROGRAMS ros/src/knoeppkes.py 
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS ros/src/buttons.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY common
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/common
)

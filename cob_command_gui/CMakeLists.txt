# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(cob_command_gui)


find_package(catkin REQUIRED COMPONENTS rospy cob_script_server cob_relayboard)

catkin_package(
    DEPENDS rospy cob_script_server cob_relayboard
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)

install(PROGRAMS ros/src/knoeppkes.py 
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS ros/src/buttons.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

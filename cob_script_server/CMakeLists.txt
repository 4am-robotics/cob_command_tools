cmake_minimum_required(VERSION 2.8.3)
project(cob_script_server)




find_package(catkin REQUIRED COMPONENTS rospy message_generation actionlib actionlib_msgs tf std_msgs cob_srvs trajectory_msgs geometry_msgs control_msgs cob_sound)

catkin_python_setup()


#######################################
## Declare ROS messages and services ##
#######################################

add_action_files(
  FILES
  Script.action
  State.action
)

add_message_files(
  FILES
  ScriptState.msg
)

generate_messages(
  DEPENDENCIES geometry_msgs std_msgs actionlib_msgs
)


###################################
## catkin specific configuration ##
###################################

catkin_package(
    DEPENDS actionlib message_runtime actionlib_msgs tf std_msgs cob_srvs trajectory_msgs geometry_msgs control_msgs move_base_msgs cob_sound
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS 
    LIBRARIES # TODO
)

#############
## Install ##
#############

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY test/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/test
)


install(PROGRAMS src/cob_console.py src/script_server.py src/script_to_graph.py src/script_viewer.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


#install(PROGRAMS src/simple_script_server.py
#         DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

## Mark executables and/or libraries for installation
#install(TARGETS cob_script_server
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

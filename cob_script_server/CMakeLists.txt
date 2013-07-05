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
    DEPENDS actionlib message_runtime actionlib_msgs tf std_msgs cob_srvs trajectory_msgs kinematics_msgs geometry_msgs control_msgs move_base_msgs arm_navigation_msgs cob_sound
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS 
    LIBRARIES # TODO
)



#############
## Install ##
#############

#install(PROGRAMS src/simple_script_server.py
#         DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

## Mark executables and/or libraries for installation
#install(TARGETS cob_script_server
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

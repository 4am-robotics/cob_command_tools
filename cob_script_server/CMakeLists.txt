# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(cob_script_server)

find_package(catkin REQUIRED COMPONENTS rospy message_generation actionlib actionlib_msgs tf std_msgs cob_srvs trajectory_msgs geometry_msgs control_msgs cob_sound rostest)


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
# TODO: fill in what other packages will need to use this package
## DEPENDS: system dependencies of this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## INCLUDE_DIRS: 
## LIBRARIES: libraries you create in this project that dependent projects also need
catkin_package(
    DEPENDS rospy actionlib message_runtime actionlib_msgs tf std_msgs cob_srvs trajectory_msgs kinematics_msgs geometry_msgs control_msgs pr2_controllers_msgs move_base_msgs arm_navigation_msgs cob_sound
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)
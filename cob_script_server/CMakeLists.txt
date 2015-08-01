cmake_minimum_required(VERSION 2.8.3)
project(cob_script_server)

find_package(catkin REQUIRED COMPONENTS message_generation trajectory_msgs actionlib_msgs actionlib rostest)

catkin_python_setup()

### Message Generation ###
add_action_files(
  FILES
  Script.action
  State.action
)

add_message_files(
  FILES
  ScriptState.msg
)

add_service_files(
  FILES
  ComposeTrajectory.srv
)

generate_messages(
  DEPENDENCIES trajectory_msgs actionlib_msgs
)


catkin_package(
    CATKIN_DEPENDS rospy message_runtime std_msgs std_srvs geometry_msgs trajectory_msgs control_msgs move_base_msgs actionlib_msgs actionlib tf cob_sound cob_light cob_mimic rostest
)


### Install ###
install(DIRECTORY common launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS src/cob_console src/cob_console.py src/script_server.py src/script_to_graph.py src/script_viewer.py src/xdot.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/src
)

install(PROGRAMS test/action_interface.py test/move.py test/python_api.py test/test_say.py test/test_script.py test/trigger.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/test
)

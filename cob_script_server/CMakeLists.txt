cmake_minimum_required(VERSION 2.8.3)
project(cob_script_server)

find_package(catkin REQUIRED COMPONENTS actionlib actionlib_msgs message_generation rostest trajectory_msgs)

catkin_python_setup()

### Message Generation ###
add_message_files(
  FILES
  KeyValue.msg
  ScriptState.msg
)

add_service_files(
  FILES
  ComposeTrajectory.srv
)

add_action_files(
  FILES
  Script.action
  State.action
)

generate_messages(
  DEPENDENCIES actionlib_msgs trajectory_msgs
)


catkin_package(
  CATKIN_DEPENDS actionlib_msgs message_runtime trajectory_msgs
)

### Install ###
install(DIRECTORY common launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS src/cob_console src/script_server.py src/script_to_graph.py src/script_viewer.py src/xdot.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS test/test_action_interface.py test/test_move.py test/test_python_api.py test/test_say.py test/test_script_action_client.py test/test_trigger.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

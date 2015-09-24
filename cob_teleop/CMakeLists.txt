cmake_minimum_required(VERSION 2.8.3)
project(cob_teleop)

find_package(catkin REQUIRED COMPONENTS actionlib cob_light cob_script_server cob_sound geometry_msgs roscpp sensor_msgs std_msgs std_srvs)

catkin_package()

### Build ###
add_executable(cob_teleop_keyboard ros/src/cob_teleop_keyboard.cpp)
target_link_libraries(cob_teleop_keyboard ${catkin_LIBRARIES})

add_executable(keyboard_publisher ros/src/keyboard_publisher.cpp)
target_link_libraries(keyboard_publisher ${catkin_LIBRARIES})

add_executable(cob_teleop ros/src/cob_teleop.cpp)
target_link_libraries(cob_teleop ${catkin_LIBRARIES})

### Install ###
install(TARGETS cob_teleop_keyboard keyboard_publisher cob_teleop
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY ros/launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

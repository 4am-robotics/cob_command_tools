cmake_minimum_required(VERSION 2.8.3)
project(cob_teleop)

find_package(catkin REQUIRED COMPONENTS actionlib cob_script_server dynamic_reconfigure geometry_msgs roscpp sensor_msgs std_msgs std_srvs trajectory_msgs)

generate_dynamic_reconfigure_options(cfg/cob_teleop_v2.cfg )

catkin_package(
  INCLUDE_DIRS common/include
  CATKIN_DEPENDS actionlib cob_script_server dynamic_reconfigure geometry_msgs roscpp sensor_msgs std_msgs std_srvs trajectory_msgs
)

### Build ###
include_directories(common/include)
include_directories(${catkin_INCLUDE_DIRS})

add_executable(cob_teleop_v2 ros/src/cob_teleop_v2_ros.cpp)
add_dependencies(cob_teleop_v2 cob_teleop_v2_gencfg)
add_dependencies(cob_teleop_v2 ${cob_teleop_EXPORTED_TARGETS})
target_link_libraries(cob_teleop_v2 ${catkin_LIBRARIES})

add_executable(cob_teleop_v1 ros/src/cob_teleop_v1.cpp)
add_dependencies(cob_teleop_v1 std_srvs_gencpp)
target_link_libraries(cob_teleop_v1 ${catkin_LIBRARIES})

add_executable(cob_teleop_keyboard ros/src/cob_teleop_keyboard.cpp)
target_link_libraries(cob_teleop_keyboard ${catkin_LIBRARIES})

add_executable(keyboard_publisher ros/src/keyboard_publisher.cpp)
target_link_libraries(keyboard_publisher ${catkin_LIBRARIES})

### Install ###
install(TARGETS cob_teleop_v1 cob_teleop_v2 cob_teleop_keyboard keyboard_publisher
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY common/include/${PROJECT_NAME}/
 DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
 FILES_MATCHING PATTERN "*.h"
 PATTERN ".svn" EXCLUDE
)

install(DIRECTORY ros/launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

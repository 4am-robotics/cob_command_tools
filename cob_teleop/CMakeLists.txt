cmake_minimum_required(VERSION 3.0.2)
project(cob_teleop)

find_package(catkin REQUIRED COMPONENTS actionlib cob_actions cob_light cob_script_server cob_sound geometry_msgs roscpp sensor_msgs std_msgs std_srvs)

catkin_package(
  CATKIN_DEPENDS actionlib cob_actions cob_light cob_script_server cob_sound geometry_msgs roscpp sensor_msgs std_msgs std_srvs
)

### Build ###
include_directories(${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_keyboard ros/src/${PROJECT_NAME}_keyboard.cpp)
add_dependencies(${PROJECT_NAME}_keyboard ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_keyboard ${catkin_LIBRARIES})

add_executable(keyboard_publisher ros/src/keyboard_publisher.cpp)
add_dependencies(keyboard_publisher ${catkin_EXPORTED_TARGETS})
target_link_libraries(keyboard_publisher ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME} ros/src/${PROJECT_NAME}.cpp)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

### Install ###
install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_keyboard keyboard_publisher
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY ros/launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

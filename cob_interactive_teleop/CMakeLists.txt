cmake_minimum_required(VERSION 2.8.3)
project(cob_interactive_teleop)

find_package(catkin REQUIRED COMPONENTS geometry_msgs interactive_markers roscpp std_msgs tf visualization_msgs)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  CATKIN_DEPENDS geometry_msgs interactive_markers roscpp std_msgs tf visualization_msgs
  DEPENDS Boost
)

### Build ###
include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_executable(interactive_teleop src/interactive_markers_tools.cpp src/teleop_cob_marker.cpp src/cob_interactive_teleop.cpp)
add_dependencies(interactive_teleop ${catkin_EXPORTED_TARGETS})
target_link_libraries(interactive_teleop ${catkin_LIBRARIES} ${Boost_LIBRARIES})

### Install ###
install(TARGETS interactive_teleop
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

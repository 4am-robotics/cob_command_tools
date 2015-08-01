cmake_minimum_required(VERSION 2.8.3)
project(cob_interactive_teleop)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs geometry_msgs visualization_msgs interactive_markers tf)

catkin_package(
    CATKIN_DEPENDS roscpp std_msgs geometry_msgs visualization_msgs interactive_markers tf rviz
)

### Build ###
include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_executable(interactive_teleop src/interactive_markers_tools.cpp src/teleop_cob_marker.cpp src/cob_interactive_teleop.cpp)
target_link_libraries(interactive_teleop ${catkin_LIBRARIES})

### Install ###
install(TARGETS interactive_teleop
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY config launch 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

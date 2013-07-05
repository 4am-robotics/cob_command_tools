cmake_minimum_required(VERSION 2.8.3)
project(cob_interactive_teleop)




find_package(catkin REQUIRED COMPONENTS roscpp interactive_markers rviz roscpp visualization_msgs interactive_markers std_msgs tf geometry_msgs sensor_msgs message_filters)

###################################
## catkin specific configuration ##
###################################

catkin_package(
    DEPENDS roscpp interactive_markers rviz roscpp visualization_msgs interactive_markers std_msgs tf geometry_msgs sensor_msgs message_filters
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS include
    LIBRARIES # TODO
)


###########
## Build ##
###########

include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_executable(interactive_teleop src/interactive_markers_tools.cpp src/teleop_cob_marker.cpp src/cob_interactive_teleop.cpp)

target_link_libraries(interactive_teleop ${catkin_LIBRARIES})


#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS interactive_teleop
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


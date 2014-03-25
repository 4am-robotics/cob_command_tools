cmake_minimum_required(VERSION 2.8.3)
project(cob_teleop)




find_package(catkin REQUIRED COMPONENTS std_msgs roscpp geometry_msgs sensor_msgs trajectory_msgs cob_srvs brics_actuator)

###################################
## catkin specific configuration ##
###################################

catkin_package(
    DEPENDS
    CATKIN_DEPENDS std_msgs roscpp geometry_msgs sensor_msgs trajectory_msgs cob_srvs brics_actuator
    INCLUDE_DIRS
    LIBRARIES # TODO
)


###########
## Build ##
###########

include_directories(${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} ros/src/cob_teleop.cpp)
add_executable(${PROJECT_NAME}_keyboard ros/src/cob_teleop_keyboard.cpp)
add_executable(keyboard_publisher ros/src/keyboard_publisher.cpp)

add_dependencies(${PROJECT_NAME} cob_srvs_gencpp)


target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_keyboard ${catkin_LIBRARIES})
target_link_libraries(keyboard_publisher ${catkin_LIBRARIES})


#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS cob_teleop
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS cob_teleop_keyboard
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS keyboard_publisher 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY ros/launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#OLD
#find_package(catkin REQUIRED COMPONENTS roscpp std_msgs geometry_msgs sensor_msgs trajectory_msgs cob_srvs brics_actuator)
#DEPENDS roscpp geometry_msgs std_msgs sensor_msgs trajectory_msgs cob_srvs brics_actuator

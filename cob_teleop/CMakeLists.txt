# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(cob_teleop)


find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs sensor_msgs trajectory_msgs cob_srvs brics_actuator)


# add project executable
include_directories(${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})


add_executable(${PROJECT_NAME} ros/src/${PROJECT_NAME}.cpp)
add_executable(${PROJECT_NAME}_keyboard ros/src/${PROJECT_NAME}_keyboard.cpp)
add_executable(keyboard_publisher ros/src/keyboard_publisher.cpp)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_keyboard ${catkin_LIBRARIES})
target_link_libraries(keyboard_publisher ${catkin_LIBRARIES})




# TODO: fill in what other packages will need to use this package
## DEPENDS: system dependencies of this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## INCLUDE_DIRS: 
## LIBRARIES: libraries you create in this project that dependent projects also need
catkin_package(
    DEPENDS roscpp geometry_msgs sensor_msgs trajectory_msgs cob_srvs brics_actuator pr2_teleop
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)
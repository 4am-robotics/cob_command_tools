# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(cob_interactive_teleop)

find_package(catkin REQUIRED COMPONENTS roscpp interactive_markers rviz roscpp visualization_msgs interactive_markers std_msgs tf geometry_msgs sensor_msgs message_filters)


set( COB_TELEOP_SOURCE_FILES src/interactive_markers_tools.cpp
                             src/teleop_cob_marker.cpp 
                             src/cob_interactive_teleop.cpp )


include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

set( COB_TELEOP_PROJECT_NAME interactive_teleop )
add_executable( ${COB_TELEOP_PROJECT_NAME} ${COB_TELEOP_SOURCE_FILES} )
target_link_libraries( ${COB_TELEOP_PROJECT_NAME} ${catkin_LIBRARIES})



# TODO: fill in what other packages will need to use this package
## DEPENDS: system dependencies of this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## INCLUDE_DIRS: 
## LIBRARIES: libraries you create in this project that dependent projects also need
catkin_package(
    DEPENDS roscpp interactive_markers rviz roscpp visualization_msgs interactive_markers std_msgs tf geometry_msgs sensor_msgs message_filters
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)
cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE RelWithDebInfo)


rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()

#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)

#rosbuild_add_boost_directories()
#rosbuild_link_boost( thread)

#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})


# BUT cob teleop
set( BUT_COB_TELEOP_SOURCE_FILES src/but_cob_teleop/interactive_markers_tools.cpp
                                 src/but_cob_teleop/TeleopCOBMarker.cpp 
                                 src/init.cpp )

set( BUT_COB_TELEOP_PROJECT_NAME but_cob_interactive_teleop )
rosbuild_add_library( ${BUT_COB_TELEOP_PROJECT_NAME} ${BUT_COB_TELEOP_SOURCE_FILES} )

rosbuild_add_executable( but_cob_teleop src/but_cob_teleop/but_cob_teleop.cpp )
target_link_libraries( but_cob_teleop ${BUT_COB_TELEOP_PROJECT_NAME} )


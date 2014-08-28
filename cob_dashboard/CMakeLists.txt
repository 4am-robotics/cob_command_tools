# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(cob_dashboard)


find_package(catkin REQUIRED COMPONENTS )

catkin_python_setup()


# TODO: fill in what other packages will need to use this package
## DEPENDS: system dependencies of this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## INCLUDE_DIRS: 
## LIBRARIES: libraries you create in this project that dependent projects also need
catkin_package(
    CATKIN_DEPENDS rospy roslib
    DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)

install(PROGRAMS scripts/cob_dashboard scripts/cob_dashboard_aggregator.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/scripts
)

install(FILES plugin.xml
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

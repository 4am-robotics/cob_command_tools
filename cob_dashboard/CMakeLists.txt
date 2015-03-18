cmake_minimum_required(VERSION 2.8.3)
project(cob_dashboard)

find_package(catkin REQUIRED COMPONENTS )

catkin_python_setup()

catkin_package(
    CATKIN_DEPENDS rospy roslib
)


install(PROGRAMS scripts/cob_dashboard scripts/cob_dashboard_aggregator.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/scripts
)

install(FILES plugin.xml
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

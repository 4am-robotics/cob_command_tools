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
    DEPENDS rxtools robot_monitor rospy roslib std_srvs std_msgs pr2_power_board pr2_msgs pr2_dashboard wxwidgets wxpython
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)

install(PROGRAMS scripts/cob_dashboard
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS src/cob_dashboard/cob_battery.py src/cob_dashboard/cob_dashboard.py src/cob_dashboard/__init__.py
  DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
)

install(FILES
   plugin.xml
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

cmake_minimum_required(VERSION 2.8.3)
project(cob_dashboard)

find_package(catkin REQUIRED COMPONENTS)

catkin_python_setup()

catkin_package()

### Install ###
catkin_install_python(PROGRAMS scripts/${PROJECT_NAME} scripts/${PROJECT_NAME}_aggregator.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

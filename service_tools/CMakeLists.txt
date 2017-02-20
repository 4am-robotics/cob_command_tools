cmake_minimum_required(VERSION 2.8.3)
project(service_tools)

find_package(catkin REQUIRED
             COMPONENTS
            )

catkin_package()


catkin_install_python(PROGRAMS scripts/service_relay
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
                     )
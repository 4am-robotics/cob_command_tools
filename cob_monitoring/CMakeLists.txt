cmake_minimum_required(VERSION 2.8.3)
project(cob_monitoring)

find_package(catkin REQUIRED COMPONENTS roscpp diagnostic_updater topic_tools)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(topic_status_monitor src/topic_status_monitor.cpp)
target_link_libraries(topic_status_monitor ${catkin_LIBRARIES})

### Install ###
install(PROGRAMS
          src/battery_monitor.py
          src/cpu_monitor.py
          src/ddwrt.py
          src/emergency_stop_monitor.py
          src/hd_monitor.py
          src/hz_monitor.py
          src/ntp_monitor.py
          src/wifi_monitor.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS topic_status_monitor
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

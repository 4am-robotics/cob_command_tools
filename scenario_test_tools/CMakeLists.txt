cmake_minimum_required(VERSION 2.8.3)
project(scenario_test_tools)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package()

catkin_install_python(PROGRAMS scripts/dummy_behavior.py scripts/example_test.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

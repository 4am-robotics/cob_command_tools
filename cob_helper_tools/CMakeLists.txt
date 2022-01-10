cmake_minimum_required(VERSION 3.0.2)
project(cob_helper_tools)

find_package(catkin REQUIRED COMPONENTS dynamic_reconfigure message_generation)

catkin_python_setup()

generate_dynamic_reconfigure_options(
  cfg/HelperTools.cfg
)

catkin_package(
  CATKIN_DEPENDS dynamic_reconfigure
)

#############
## Install ##
#############

catkin_install_python(PROGRAMS
  scripts/auto_tools.py
  scripts/auto_init.py
  scripts/auto_recover.py
  scripts/fake_diagnostics.py
  scripts/fake_driver.py
  scripts/visualize_navigation_goals.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

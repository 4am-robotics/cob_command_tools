cmake_minimum_required(VERSION 2.8.3)
project(generic_throttle)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roslib
  std_msgs
  sensor_msgs
  visualization_msgs
)

catkin_python_setup()

catkin_package()

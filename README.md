# cob_generic_throttle
> Tested on ROS Indigo - Ubuntu 14.04 - Python 2.7

cob_generic_throttle is a ROS package offering a Python implementation of a throttle for topics.
Despite the "cob_" in the name, the package is a general ROS tool.

Class GenericThrottle runs a ROS node which subcribes to a list of topics and published them again at a desired rate. List of topics and desired rate are specified through ROS parameters.

Why should I use this? well, if you remotely control a robot (e.g. via Wifi), you want some topic to be streamed from the robot to you, e.g. to your rviz for simulation purpose. Now, let's say you stream several Pointcloud topics over Wifi at 30 hz. This is going to flood your network with (heavy) messages and you will only get unreliable/delayed data. With a GenericThrottle node running on the robot, you could reduce the rate of the topics your are interested in and read the "throttled" topics from remote without flooding your network.

# TODO
- for sensor_msgs/Image enable reduction of the resolution of the throttled topic
- for sensor_msgs/PointCloud2 enable reduction of points in the throttled topic

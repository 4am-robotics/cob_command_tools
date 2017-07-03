# generic_throttle

generic_throttle is package offering a Python implementation of a throttle for topics.

Class GenericThrottle runs a ROS node which subscribes to a list of topics and
publishes them again at a desired rate. List of topics and desired rate are
specified through ROS parameters.

By default, the throttle operates in a latched mode: if no new message are available
from the input topics, it will continue to publish the last available message.
However, after a certain time (to be defined in parameter /generic_throttle/delay
in seconds) it will throw a warning and stop publishing the old message. As soon as
new messages become available, the throttle will start to publish again.

Each throttled topic is treated as independent (if one stops, the other are not
influenced by that).

# TODO
- for sensor_msgs/Image enable reduction of the resolution of the throttled topic
- for sensor_msgs/PointCloud2 enable reduction of points in the throttled topic

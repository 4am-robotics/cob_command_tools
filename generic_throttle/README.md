# generic_throttle

generic_throttle is package offering a Python implementation of a throttle for topics.

The package is composed of
- `GenericThrottle`: class implementing the throttle
- `generic_throttle_node`: ROS node wrapping around an istance of `GenericThrottle`

The parameters are set with ROS parameters. An example of .yaml file:
```
test_throttle:          # Throttle namespace
  topics:               # List of topics to be throttled
    - /topic1: {latched: False, lazy: True, topic_rate: 1.0}
    - /topic2: {latched: True, lazy: False, topic_rate: 20.0}
```
For each topic, 3 parameters must be specified:
- `latched`: if `True`, the publisher of the throttled topic acts as latched (see  http://docs.ros.org/api/rospy/html/rospy.topics.Publisher-class.html)
- `lazy`: if `True`, the publisher of the throttled topics will check if the number of subscribers and if there are no, it will not publish the message
- `topic_rate`: desired rate (in Hz) for the throttled topic.

The throttle will publish on topics with name `/topic1_throttled` (it appends `_throttled` to the original name).
When running an instance of `generic_throttle_node`, you must define the throttle namespace to let the node discover the required parameters. An example from .launch file:
```
<node pkg="generic_throttle" type="generic_throttle_node.py" name="throttle" ns="test_throttle" />
```
# TODO
- for `sensor_msgs/Image` enable reduction of the resolution of the throttled topic
- for `sensor_msgs/PointCloud2` enable reduction (VoxelGrid sampling) of points in the throttled topic
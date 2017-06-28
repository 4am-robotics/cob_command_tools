#!/usr/bin/python

import rospy
import roslib
from std_msgs.msg import String


class GenericThrottle:
    def __init__(self):
        self.namespace = None
        self.topic_dictionary = None
        self.topic_framerate = None

        # Read /generic_throttle/* parameters from server
        if rospy.has_param('/generic_throttle/namespace'):
            self.namespace = rospy.get_param('/generic_throttle/namespace')
        else:
            rospy.logerr('Parameter /generic_throttle/namespace '
                         'is not available.')
            exit(-1)
        if rospy.has_param('/generic_throttle/framerate'):
            self.topic_framerate = rospy.get_param(
                '/generic_throttle/framerate')
        else:
            rospy.logerr('Parameter /generic_throttle/framerate '
                         'is not available.')
            exit(-1)

        # Check if each entry of topic_framerate has 2 entries
        size_flag = all(len(item) == 2 for item in self.topic_framerate)

        if(not(size_flag)):
            rospy.logerr('Parameter /generic_throttle/framerate must be'
                         'a list of size 2 lists')
            exit(-1)

        self.populate_dictionary()
        rospy.init_node('generic_throttle')

    def populate_dictionary(self):
        self.topic_dictionary = {key: [value] for (key, value)
                                 in self.topic_framerate}
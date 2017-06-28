#!/usr/bin/python

import rospy
import roslib
from std_msgs.msg import String

    from functools import partial


class GenericThrottle:
    def __init__(self):
        self.namespace = None
        self.topic_dictionary = None
        self.topic_framerate = None

        # Read /generic_throttle/* parameters from server
        if rospy.has_param('/generic_throttle/namespace'):
            self.namespace = rospy.get_param('/generic_throttle/namespace')
        else:
            rospy.logerr('Parameter /generic _throttle/namespace '
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

        rospy.init_node('generic_throttle')
        self.populate_dictionary()
        rospy.spin()

    def timer_callback(self,event,topic_id):
        print topic_id


    def populate_dictionary(self):
        # Topic dictionary structure
        # {topic_name: [max_framerate, timer, last_message,
        # subscriber, publisher]
        self.topic_dictionary = {key: [value, None, None, None, None]
                                 for (key, value) in self.topic_framerate}

        # Create Timer for each topic
        for key, element in self.topic_dictionary.iteritems():
            personal_callback = partial(self.timer_callback,topic_id=key)
            element[1] = rospy.Timer(
                rospy.Duration(1./element[0]),
                personal_callback)
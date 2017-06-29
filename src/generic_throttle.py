#!/usr/bin/python

import rospy
import roslib
import rostopic
from threading import Lock
from std_msgs.msg import String
from functools import partial



class GenericThrottle:
    def __init__(self):
        self.namespace = None
        self.topic_dictionary = None
        self.topic_framerate = None
        self.node_handle = rospy.init_node('generic_throttle')

        # Read /generic_throttle/* parameters from server
        if rospy.has_param('/generic_throttle/namespace'):
            self.namespace = rospy.get_param('/generic_throttle/namespace')
        else:
            rospy.logerr('Parameter /generic _throttle/namespace '
                         'is not available.')
            exit(5)
        if rospy.has_param('/generic_throttle/framerate'):
            self.topic_framerate = rospy.get_param(
                '/generic_throttle/framerate')
        else:
            rospy.logerr('Parameter /generic_throttle/framerate '
                         'is not available.')
            exit(5)

        # Check if each entry of topic_framerate has 2 entries
        size_flag = all(len(item) == 2 for item in self.topic_framerate)

        if(not(size_flag)):
            rospy.logerr('Parameter /generic_throttle/framerate must be'
                         ' a list of size 2 lists')
            exit(10)

        self.populate_dictionary()
        rospy.spin()

    def timer_callback(self, event, topic_id):

        # Lock into the dictionary entry
            # You can't? Warning. exit

        # If relative publisher is None
            # Test the topic
                # it does not exit. Warning. exit
            # Create publisher and subscriber
            # exit

        # If last_message is None
            # Warning. exit

        # Everything seems fine --> publish last_message on publisher

        # Unlock
    def subscriber_callback(self, data):
        print data

    def populate_dictionary(self):
        # Topic dictionary structure
        # {topic_name: [max_framerate, timer, last_message,
        # subscriber, publisher, lock]
        self.topic_dictionary = {key: [value, None, None, None, None, None]
                                 for (key, value) in self.topic_framerate}

        for key, element in self.topic_dictionary.iteritems():
            # Create Timer for each topic
            personal_callback = partial(self.timer_callback, topic_id=key)
            element[1] = rospy.Timer(rospy.Duration(1./element[0]),
                personal_callback)
            # Create Lock for each topic
            element[5] = Lock()
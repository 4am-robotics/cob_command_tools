#!/usr/bin/python

import rospy
import rostopic
from threading import Lock
from threading import currentThread
from functools import partial


class GenericThrottle:
    def __init__(self):
        self.topic_dictionary = None
        self.parameters = {'namespace': None,
                           'topic_framerate': None,
                           'latched': True,
                           'lazy': False}

        rospy.init_node('generic_throttle')
        rospy.on_shutdown(self._shutdown)

        # Read mandatory /generic_throttle/* parameters from server
        parameter_list = ['namespace', 'topic_framerate']
        for parameter_name in parameter_list:
            parameter_string = '/generic_throttle/' + parameter_name
            if rospy.has_param(parameter_string):
                self.parameters[parameter_name] = \
                    rospy.get_param(parameter_string)
            else:
                rospy.logerr('Parameter ' + parameter_string + ' not available')
                exit(5)

        # Read optional /generic_throttle/* parameters from server
        parameter_list = ['latched', 'lazy']
        for parameter_name in parameter_list:
            parameter_string = '/generic_throttle/' + parameter_name
            if rospy.has_param(parameter_string):
                self.parameters[parameter_name] = \
                    rospy.get_param(parameter_string)

        # Check if each entry of topic_framerate has 2 entries
        size_flag = all(len(item) == 2 for item in
                        self.parameters['topic_framerate'])

        if(not(size_flag)):
            rospy.logerr('Parameter /generic_throttle/framerate must be'
                         ' a list of size 2 lists')
            exit(10)

        # Populate the dictionary for the ros topic throttling
        self._populate_dictionary()
        rospy.spin()

    def timer_callback(self, event, topic_id):
        # The False argument is for a non blocking call
        locking = self.topic_dictionary[topic_id]['lock'].acquire_lock(False)


        if not(locking):
            current_t = currentThread()
            rospy.logdebug(str(current_t._Thread__name) + ': cannot lock topic '
                          + topic_id)
            return

        publisher = self.topic_dictionary[topic_id]['publisher']
        subscriber = self.topic_dictionary[topic_id]['subscriber']

        # Check if pub and sub already exist, if not try to create them after
        # checking the ros topic

        if None in(publisher, subscriber):
            topic_info = rostopic.get_topic_class(topic_id)
            if topic_info[0] is None:
                rospy.logwarn('Cannot find topic ' + topic_id)

                self.topic_dictionary[topic_id]['lock'].release_lock()
                return
            else:
                # Create publisher
                self.topic_dictionary[topic_id]['publisher'] = \
                    rospy.Publisher(self.parameters['namespace'] + topic_id,
                                    topic_info[0], queue_size=1)
                rospy.loginfo('Created publisher for ' +
                              self.parameters['namespace'] + topic_id)
                # Create subscriber
                subscriber_partial = partial(self.subscriber_callback,
                                             topic_id=topic_id)
                self.topic_dictionary[topic_id]['subscriber'] = \
                    rospy.Subscriber(topic_id, topic_info[0],
                                     subscriber_partial)
                rospy.loginfo('Created subscriber for ' + topic_id)

                self.topic_dictionary[topic_id]['lock'].release_lock()
                return

        last_message = self.topic_dictionary[topic_id]['last_message']

        if last_message is not None:
            self.topic_dictionary[topic_id]['publisher'].publish(
                last_message)
            if not(self.parameters['latched']):
                self.topic_dictionary[topic_id]['last_message'] = None
            self.topic_dictionary[topic_id]['lock'].release_lock()
            return

        self.topic_dictionary[topic_id]['lock'].release_lock()
        return

    def subscriber_callback(self, data, topic_id):
        locking = self.topic_dictionary[topic_id]['lock'].acquire_lock(False)
        # The False argument is for a non blocking call

        if not (locking):
            current_t = currentThread()
            rospy.logdebug(str(current_t._Thread__name) + ': cannot lock topic '
                          + topic_id)
            return

        if data._has_header:
            self.topic_dictionary[topic_id]['last_timestamp'] = \
                data.header.stamp
        else:
            self.topic_dictionary[topic_id]['last_timestamp'] = \
                rospy.Time.now()

        self.topic_dictionary[topic_id]['last_message'] = data
        self.topic_dictionary[topic_id]['lock'].release_lock()

    def _populate_dictionary(self):
        # Topic dictionary structure
        # {topic_name: {framerate, timer, last_message,
        # subscriber, publisher, lock}
        self.topic_dictionary = {key: {'framerate': value, 'timer': None,
                                       'last_message': None,
                                       'last_timestamp': None,
                                       'subscriber': None,
                                       'publisher': None, 'lock': Lock()}
                                 for (key, value) in
                                 self.parameters['topic_framerate']}

        for key, element in self.topic_dictionary.iteritems():
            element['lock'].acquire_lock()
            # Create Timer for each topic
            personal_callback = partial(self.timer_callback, topic_id=key)
            element['timer'] = rospy.Timer(
                rospy.Duration(1. / element['framerate']),
                personal_callback)
            element['lock'].release_lock()

    def _shutdown(self):
        rospy.loginfo('Stopping ' + str(rospy.get_name()))

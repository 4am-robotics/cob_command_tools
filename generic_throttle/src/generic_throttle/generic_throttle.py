#!/usr/bin/python

import rospy
import rostopic
from threading import Lock
from threading import currentThread
from functools import partial


class GenericThrottle:
    def __init__(self):
        self.topics = None

        topics_param_name = str(rospy.get_namespace()) + '/topics'

        if rospy.has_param(topics_param_name):
            topics_list = rospy.get_param(topics_param_name)
        else:
            rospy.logerr('Parameter ' + topics_param_name + ' not available')
            exit(5)

        # create dictionary out of the topic list
        self.topics = {item.keys()[0]: item.values()[0] for item in topics_list}

        # Check if each entry of topics has 3 parameters (
        size_flag = all(len(item) == 3 for item in self.topics.values())

        if(not(size_flag)):
            rospy.logerr('Each throttled topic needs 3 parameters ' +
                         '[latched, lazy, topic_rate]')
            exit(10)

        # Populate the dictionary for the ros topic throttling
        self._populate_dictionary()

    def timer_callback(self, event, topic_id):
        # The False argument is for a non blocking call
        locking = self.topics[topic_id]['lock'].acquire_lock(False)

        if not(locking):
            current_t = currentThread()
            rospy.logdebug(str(current_t._Thread__name) + ': cannot lock topic '
                          + topic_id)
            return

        publisher = self.topics[topic_id]['publisher']
        subscriber = self.topics[topic_id]['subscriber']

        # Check if pub and sub already exist, if not try to create them after
        # checking the ros topic

        if None in(publisher, subscriber):
            topic_info = rostopic.get_topic_class(topic_id)
            if topic_info[0] is None:
                rospy.logwarn('Cannot find topic ' + topic_id)

                self.topics[topic_id]['lock'].release_lock()
                return
            else:
                # Create publisher
                self.topics[topic_id]['publisher'] = \
                    rospy.Publisher(topic_id + '_throttled',
                                    topic_info[0], queue_size=1)
                rospy.loginfo('Created publisher for ' + topic_id)
                # Create subscriber
                subscriber_partial = partial(self.subscriber_callback,
                                             topic_id=topic_id)
                self.topics[topic_id]['subscriber'] = \
                    rospy.Subscriber(topic_id, topic_info[0],
                                     subscriber_partial)
                rospy.loginfo('Created subscriber for ' + topic_id)

                self.topics[topic_id]['lock'].release_lock()
                return

        last_message = self.topics[topic_id]['last_message']

        if last_message is not None:
            if self.topics[topic_id]['lazy']:
                # Lazy behavior: if nobody is listening don't publish
                if self.topics[topic_id]['publisher']\
                        .get_num_connections() > 0:
                    self.topics[topic_id]['publisher'].publish(
                        last_message)
            else:
                self.topics[topic_id]['publisher'].publish(
                    last_message)

            if not(self.topics[topic_id]['latched']):
                # Not latched behavior: delete last message that was just sent
                self.topics[topic_id]['last_message'] = None
            self.topics[topic_id]['lock'].release_lock()
            return

        self.topics[topic_id]['lock'].release_lock()
        return

    def subscriber_callback(self, data, topic_id):
        locking = self.topics[topic_id]['lock'].acquire_lock(False)
        # The False argument is for a non blocking call

        if not (locking):
            current_t = currentThread()
            rospy.logdebug(str(current_t._Thread__name) + ': cannot lock topic '
                          + topic_id)
            return

        self.topics[topic_id]['last_message'] = data
        self.topics[topic_id]['lock'].release_lock()

    def _populate_dictionary(self):
        # Topic dictionary structure
        # {topic_id: {topic_rate, lazy, latched, subscriber,
        # publisher, lock, timer, last_message}

        for key, element in self.topics.iteritems():
            element['lock'] = Lock()
            element['lock'].acquire_lock()
            element['publisher'] = None
            element['subscriber'] = None
            # Create Timer for each topic
            personal_callback = partial(self.timer_callback, topic_id=key)
            element['timer'] = rospy.Timer(
                rospy.Duration(1.0 / element['topic_rate']), personal_callback)
            element['last_message'] = None
            element['lock'].release_lock()

    def _shutdown(self):
        rospy.loginfo('Stopping ' + str(rospy.get_name()))

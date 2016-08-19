#!/usr/bin/env python

import sys

import rospy
import rostopic

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

class HzTest():
    def __init__(self):
        rospy.init_node("hz_monitor")

        self.message_received = False
        # get parameters
        try:
            # topic to test
            self.topic = rospy.get_param('~topic')
            # expected publishing rate
            self.hz = rospy.get_param('~hz', None)
            # margin of error allowed
            self.hzerror = rospy.get_param('~hzerror', None)
            # length of test
            self.window_size = float(rospy.get_param('~window_size', -1))
            # name for diagnostic message
            self.diagnostic_name = rospy.get_param('~diagnostic_name', "hz_monitor_" + self.topic)
            self.diagnostic_name = self.diagnostic_name.replace('/','_')
        except KeyError as e:
            rospy.logerr('hztest not initialized properly. Parameter [%s] not set. debug[%s] debug[%s]'%(str(e), rospy.get_caller_id(), rospy.resolve_name(e.args[0])))
            sys.exit(1)

        self.pub_diagnostics = rospy.Publisher('~diagnostics', DiagnosticArray, queue_size = 1)

    def run(self):
        r = rospy.Rate(1)

        # wait for first message
        while not rospy.is_shutdown():
            msg_class, real_topic, _ = rostopic.get_topic_class(self.topic, blocking=False) #pause hz until topic is published
            if real_topic:
                break
            rospy.loginfo("hz monitor is waiting for first message to be published on %s."%self.topic)
            self.publish_diagnostics()
            r.sleep()

        # call rostopic hz
        rt = rostopic.ROSTopicHz(self.window_size)
        rospy.Subscriber(real_topic, rospy.AnyMsg, rt.callback_hz)
        print("subscribed to [%s]"%real_topic)

        # publish diagnostics continuously

        while not rospy.is_shutdown():
            #rt.print_hz() # taken from 'rostopic hz' (/opt/ros/indigo/lib/python2.7/dist-packages/rostopic/__init__.py)
            self.publish_diagnostics(rt)
            r.sleep()
    
    def publish_diagnostics(self, rt = None):
        # set desired rates
        if self.hzerror:
            if isinstance(self.hzerror, float) or isinstance(self.hzerror, int):
                min_rate = self.hz - self.hzerror
                max_rate = self.hz + self.hzerror
            else:
                rospy.logerr("hzerror not float or int")
                sys.exit(1)
        else:
            min_rate = None
            max_rate = None

        # create diagnostic message
        array = DiagnosticArray()
        hz_status = DiagnosticStatus()
        hz_status.name = self.diagnostic_name
        hz_status.values.append(KeyValue("topic", str(self.topic)))

        # calculate actual rates
        if not rt or not rt.times:
            hz_status.level = DiagnosticStatus.ERROR
            hz_status.message = 'never received a message'
            hz_status.values.append(KeyValue("rate", str(0.0)))
        elif rt.msg_tn == rt.last_printed_tn:
            hz_status.level = DiagnosticStatus.ERROR
            hz_status.message = 'no messages anymore'
            hz_status.values.append(KeyValue("rate", str(0.0)))
        else:
            with rt.lock: # calculation taken from /opt/ros/indigo/lib/python2.7/dist-packages/rostopic/__init__.py
                n = len(rt.times)
                mean = sum(rt.times) / n
                rate = 1./mean if mean > 0. else 0
                rt.last_printed_tn = rt.msg_tn
            hz_status.values.append(KeyValue("rate", str(rate)))
            if min_rate and rate < min_rate:
                hz_status.level = DiagnosticStatus.OK
                hz_status.message = 'publishing rate is too low'
            elif max_rate and rate > max_rate:
                hz_status.level = DiagnosticStatus.OK
                hz_status.message = 'publishing rate is too high'
            else:
                hz_status.level = DiagnosticStatus.OK
                hz_status.message = 'publishing rate is ok'

        hz_status.values.append(KeyValue("desired_rate", str(self.hz)))
        hz_status.values.append(KeyValue("min_rate", str(min_rate)))
        hz_status.values.append(KeyValue("max_rate", str(max_rate)))
        hz_status.values.append(KeyValue("window_size", str(self.window_size)))
        array.status.append(hz_status)
        self.pub_diagnostics.publish(array)

if __name__ == '__main__':
    hzt = HzTest()
    hzt.run()
        

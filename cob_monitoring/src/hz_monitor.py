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
            self.topics = rospy.get_param('~topics')
            # expected publishing rate
            self.hz = rospy.get_param('~hz', None)
            # margin of error allowed
            self.hzerror = rospy.get_param('~hzerror', None)
            # length of test
            self.window_size = float(rospy.get_param('~window_size', 100))
            # name for diagnostic message
            self.diagnostic_name = rospy.get_param('~diagnostic_name')
            self.diagnostic_name = self.diagnostic_name.replace('/','_')
        except KeyError as e:
            rospy.logerr('hztest not initialized properly. Parameter [%s] not set. debug[%s] debug[%s]'%(str(e), rospy.get_caller_id(), rospy.resolve_name(e.args[0])))
            sys.exit(1)

        self.pub_diagnostics = rospy.Publisher('~diagnostics', DiagnosticArray, queue_size = 1)

    def run(self):
        r = rospy.Rate(1)

        # wait for first message
        while not rospy.is_shutdown():
            real_topic_store = []
            for topic in self.topics:
                msg_class, real_topic, _ = rostopic.get_topic_class(topic, blocking=False) #pause hz until topic is published
                if real_topic:
                    if real_topic not in real_topic_store:
                        real_topic_store.append(real_topic)
            if len(real_topic_store) == len(self.topics):
                break
            rospy.loginfo("hz monitor is waiting for first message to be published on %s."%topic)
            self.publish_diagnostics()
            r.sleep()

        # call rostopic hz
        #rt = rostopic.ROSTopicHz(self.window_size)
        rt_HZ_store = []
        for r_topic in real_topic_store:
            rt = rostopic.ROSTopicHz(self.window_size)
            rospy.Subscriber(r_topic, rospy.AnyMsg, rt.callback_hz)
            rt_HZ_store.append(rt)
            print("subscribed to [%s]"%r_topic)

        # publish diagnostics continuously

        while not rospy.is_shutdown():
            #rt.print_hz() # taken from 'rostopic hz' (/opt/ros/indigo/lib/python2.7/dist-packages/rostopic/__init__.py)
            self.publish_diagnostics(rt_HZ_store)
            r.sleep()

    def publish_diagnostics(self, rt_HZ_store = None):
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
        array.header.stamp = rospy.Time.now()
        hz_status = DiagnosticStatus()
        hz_status.name = self.diagnostic_name
        hz_status.values.append(KeyValue("topic", str(self.topics)))
        publishing_rate_error = False
        rates_store = [] ## to display individual rates per topics
        
        consolidated_error_messages = {} ## store and display consolidated erros messages for all the topics
        consolidated_error_messages.setdefault("never received message for topics", [])
        consolidated_error_messages.setdefault("no messages anymore for topics", [])
        consolidated_error_messages.setdefault("publishing rate is too low for topics", [])
        consolidated_error_messages.setdefault("publishing rate is too high for topics", [])

        # calculate actual rates
        for rt, topic in zip(rt_HZ_store, self.topics):
            if not rt or not rt.times:
                hz_status.level = DiagnosticStatus.ERROR
                hz_status.values.append(KeyValue("rate", str(0.0)))
                consolidated_error_messages["never received message for topics"].append(topic)
            elif rt.msg_tn == rt.last_printed_tn:
                hz_status.level = DiagnosticStatus.ERROR
                hz_status.values.append(KeyValue("rate", str(0.0)))
                consolidated_error_messages["no messages anymore for topics"].append(topic)
            else:
                with rt.lock: # calculation taken from /opt/ros/indigo/lib/python2.7/dist-packages/rostopic/__init__.py
                    n = len(rt.times)
                    mean = sum(rt.times) / n
                    rate = 1./mean if mean > 0. else 0
                    rt.last_printed_tn = rt.msg_tn
                rates_store.append("%.2f" % rate)
                if min_rate and rate < min_rate:
                    hz_status.level = DiagnosticStatus.WARN
                    publishing_rate_error = True
                    consolidated_error_messages["publishing rate is too low for topics"].append(topic)
                elif max_rate and rate > max_rate:
                    hz_status.level = DiagnosticStatus.WARN
                    publishing_rate_error = True
                    consolidated_error_messages["publishing rate is too high for topics"].append(topic)
                else:
                    if not publishing_rate_error:
                        hz_status.level = DiagnosticStatus.OK
                        hz_status.message = 'all publishing rates are ok'

        if publishing_rate_error:
            message = ""
            key = "publishing rate is too low for topics"
            if len(consolidated_error_messages[key]) > 0:
                message += key +" "+ str(consolidated_error_messages[key])
            key = "publishing rate is too high for topics"
            if len(consolidated_error_messages[key]) > 0:
                message += ", "+key +" "+ str(consolidated_error_messages[key])
            key = "never received message for topics"
            if len(consolidated_error_messages[key]) > 0:
                message += ", "+key +" "+ str(consolidated_error_messages[key])
            key = "no messages anymore for topics"
            if len(consolidated_error_messages[key]) > 0:
                message += ", "+key +" "+ str(consolidated_error_messages[key])
            hz_status.message = message
        

        hz_status.values.append(KeyValue("rate", str(rates_store)))
        hz_status.values.append(KeyValue("desired_rate", str(self.hz)))
        hz_status.values.append(KeyValue("min_rate", str(min_rate)))
        hz_status.values.append(KeyValue("max_rate", str(max_rate)))
        hz_status.values.append(KeyValue("window_size", str(self.window_size)))
        array.status.append(hz_status)
        self.pub_diagnostics.publish(array)

if __name__ == '__main__':
    hzt = HzTest()
    hzt.run()

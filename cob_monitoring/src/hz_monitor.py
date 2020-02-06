#!/usr/bin/env python
#
# Copyright 2017 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import sys

import rospy
import rostopic
import copy

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

class HzTest():
    def __init__(self):
        rospy.init_node("hz_monitor")

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
            self.diagnostics_name = rospy.get_param('~diagnostics_name',"")
            self.diagnostics_name = self.diagnostics_name.replace('/','_')
            # sampling rate
            self.sampling_rate = rospy.get_param('~sampling_rate', 1)
            if self.hz < 2*self.sampling_rate:
                rospy.logwarn("sampling_rate is probably to low: desired hz rate should at least be double the sampling_rate. (desired hz rate: %.2f, sampling_rate: %.2f)",self.hz, self.sampling_rate)
        except KeyError as e:
            rospy.logerr('hztest not initialized properly. Parameter [{}] not set. debug[{}] debug[{}]'.format(e, rospy.get_caller_id(), rospy.resolve_name(e.args[0])))
            sys.exit(1)

        self.pub_diagnostics = rospy.Publisher('~diagnostics', DiagnosticArray, queue_size = 1)
        self.missing_topics = copy.deepcopy(self.topics)

    def run(self):
        r = rospy.Rate(self.sampling_rate)

        # wait for first message
        while not rospy.is_shutdown():
            for topic in self.topics:
                msg_class, real_topic, _ = rostopic.get_topic_class(topic, blocking=False) #pause hz until topic is published
                if real_topic:
                    if real_topic in self.missing_topics:
                        self.missing_topics.remove(real_topic)
            if len(self.missing_topics) == 0:
                break
            rospy.logdebug("hz monitor is waiting for type of topics {}.".format(self.missing_topics))
            self.publish_diagnostics()
            
            try:
                r.sleep()
            except rospy.exceptions.ROSInterruptException:
                pass

        # call rostopic hz
        #rt = rostopic.ROSTopicHz(self.window_size)
        rt_HZ_store = []
        for topic in self.topics:
            rt = rostopic.ROSTopicHz(self.window_size)
            rospy.Subscriber(topic, rospy.AnyMsg, rt.callback_hz)
            rt_HZ_store.append(rt)
            rospy.loginfo("subscribed to [{}]".format(topic))

        # publish diagnostics continuously
        while not rospy.is_shutdown():
            #rt.print_hz() # taken from 'rostopic hz' (/opt/ros/indigo/lib/python2.7/dist-packages/rostopic/__init__.py)
            self.publish_diagnostics(rt_HZ_store)
            try:
                r.sleep()
            except rospy.exceptions.ROSInterruptException:
                pass

    def publish_diagnostics(self, rt_HZ_store = []):
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
        hz_status.name = self.diagnostics_name
        hz_status.values.append(KeyValue("topics", str(self.topics)))
        hz_status.values.append(KeyValue("desired_rate", str(self.hz)))
        hz_status.values.append(KeyValue("min_rate", str(min_rate)))
        hz_status.values.append(KeyValue("max_rate", str(max_rate)))
        hz_status.values.append(KeyValue("window_size", str(self.window_size)))
        publishing_rate_error = False
        rates = []
        
        consolidated_error_messages = {} ## store and display consolidated erros messages for all the topics
        consolidated_error_messages.setdefault("never received message for topics", [])
        consolidated_error_messages.setdefault("no messages anymore for topics", [])
        consolidated_error_messages.setdefault("publishing rate is too low for topics", [])
        consolidated_error_messages.setdefault("publishing rate is too high for topics", [])

        if len(rt_HZ_store) != len(self.topics):
            hz_status.level = DiagnosticStatus.WARN
            hz_status.message = "could not determine type for topics %s. Probably never published on that topics."%self.missing_topics
            array.status.append(hz_status)
            self.pub_diagnostics.publish(array)
            return

        # calculate actual rates
        for rt, topic in zip(rt_HZ_store, self.topics):
            if not rt or not rt.times:
                hz_status.level = DiagnosticStatus.ERROR
                publishing_rate_error = True
                rates.append(0.0)
                consolidated_error_messages["never received message for topics"].append(topic)
            elif rt.msg_tn == rt.last_printed_tn:
                hz_status.level = DiagnosticStatus.ERROR
                publishing_rate_error = True
                rates.append(0.0)
                consolidated_error_messages["no messages anymore for topics"].append(topic)
            else:
                with rt.lock: # calculation taken from /opt/ros/indigo/lib/python2.7/dist-packages/rostopic/__init__.py
                    n = len(rt.times)
                    mean = sum(rt.times) / n
                    rate = 1./mean if mean > 0. else 0
                    rt.last_printed_tn = rt.msg_tn
                rates.append(round(rate, 2))
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
            key = "never received message for topics"
            if len(consolidated_error_messages[key]) > 0:
                message += key + " " + str(consolidated_error_messages[key]) + ", "
            key = "no messages anymore for topics"
            if len(consolidated_error_messages[key]) > 0:
                message += key + " " + str(consolidated_error_messages[key]) + ", "
            key = "publishing rate is too low for topics"
            if len(consolidated_error_messages[key]) > 0:
                message += key + " " + str(consolidated_error_messages[key]) + ", "
            key = "publishing rate is too high for topics"
            if len(consolidated_error_messages[key]) > 0:
                message += key + " " + str(consolidated_error_messages[key])
            hz_status.message = message

        hz_status.values.append(KeyValue("rates", str(rates)))
        array.status.append(hz_status)
        self.pub_diagnostics.publish(array)

if __name__ == '__main__':
    hzt = HzTest()
    hzt.run()

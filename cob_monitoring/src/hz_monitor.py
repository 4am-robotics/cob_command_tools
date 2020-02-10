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
            self.hz = rospy.get_param('~hz')
            # margin of error allowed
            self.hzerror = rospy.get_param('~hzerror')
            # length of test
            self.window_size = float(rospy.get_param('~window_size', 100))
            # name for diagnostic message
            self.diagnostics_name = rospy.get_param('~diagnostics_name',"")
            self.diagnostics_name = self.diagnostics_name.replace('/','_')
            self.diagnostics_period = rospy.Duration(1.0)
        except KeyError as e:
            rospy.logerr('hztest not initialized properly. Parameter [{}] not set. debug[{}] debug[{}]'.format(e, rospy.get_caller_id(), rospy.resolve_name(e.args[0])))
            sys.exit(1)

        self.rt_HZ_store = dict()
        self.missing_topics = copy.deepcopy(self.topics)
        self.pub_diagnostics = rospy.Publisher('~diagnostics', DiagnosticArray, queue_size = 1)
        self.timer_diagnostics = rospy.Timer(self.diagnostics_period, self.publish_diagnostics)


    def run(self):
        # wait for first message
        while len(self.missing_topics) != 0 and not rospy.is_shutdown():
            for topic in self.topics:
                msg_class, real_topic, msg_eval = rostopic.get_topic_class(topic, blocking=False) #pause hz until topic is published
                if real_topic:
                    if real_topic in self.missing_topics:
                        self.missing_topics.remove(real_topic)

            try:
                rospy.logdebug("hz monitor is waiting for type of topics {}.".format(self.missing_topics))
                rospy.sleep(1.0)
            except rospy.exceptions.ROSInterruptException:
                pass

        # call rostopic hz
        self.rt_HZ_store = dict()
        for topic in self.topics:
            rt = rostopic.ROSTopicHz(self.window_size)
            rospy.Subscriber(topic, rospy.AnyMsg, rt.callback_hz)
            self.rt_HZ_store[topic] = rt
            rospy.loginfo("subscribed to [{}]".format(topic))

    def publish_diagnostics(self, event):
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
        hz_status.level = DiagnosticStatus.OK
        hz_status.message = 'all publishing rates are ok'
        hz_status.values.append(KeyValue("topics", str(self.topics)))
        hz_status.values.append(KeyValue("desired_rate", str(self.hz)))
        hz_status.values.append(KeyValue("min_rate", str(min_rate)))
        hz_status.values.append(KeyValue("max_rate", str(max_rate)))
        hz_status.values.append(KeyValue("window_size", str(self.window_size)))
        rates = []
        
        consolidated_error_messages = {} ## store and display consolidated erros messages for all the topics
        consolidated_error_messages.setdefault("never received message for topics", [])
        consolidated_error_messages.setdefault("no messages anymore for topics", [])
        consolidated_error_messages.setdefault("publishing rate is too low for topics", [])
        consolidated_error_messages.setdefault("publishing rate is too high for topics", [])

        if not all (k in self.rt_HZ_store for k in self.topics):
            hz_status.level = DiagnosticStatus.WARN
            hz_status.message = "could not determine type for topics {}. Probably never published on that topics.".format(self.missing_topics)
            array.status.append(hz_status)
            self.pub_diagnostics.publish(array)
            return

        # calculate actual rates
        for topic, rt in self.rt_HZ_store.items():
            #print("rt.times {}".format(rt.times))
            #print("rt.msg_tn {}".format(rt.msg_tn))
            #print("rt.last_printed_tn {}".format(rt.last_printed_tn))
            #print("rt.delta: {}".format(rt.msg_tn - rt.last_printed_tn))
            #print("event.current_real: {}".format(event.current_real.to_sec()))
            #print("event.last_real: {}".format(event.last_real.to_sec()))
            #print("event.delta: {}".format((event.current_real - event.last_real).to_sec()))
            if not rt or not rt.times:
                hz_status.level = DiagnosticStatus.ERROR
                rates.append(0.0)
                consolidated_error_messages["never received message for topics"].append(topic)
            elif rt.msg_tn == rt.last_printed_tn \
                 and not (event.current_real.to_sec() < rt.msg_tn + 1/self.hz):  # condition to prevent WARN for topics with hz<diagnostics_rate
                hz_status.level = DiagnosticStatus.ERROR
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
                    consolidated_error_messages["publishing rate is too low for topics"].append(topic)

                if max_rate and rate > max_rate:
                    hz_status.level = DiagnosticStatus.WARN
                    consolidated_error_messages["publishing rate is too high for topics"].append(topic)

        if hz_status.level != DiagnosticStatus.OK:
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
    rospy.spin()

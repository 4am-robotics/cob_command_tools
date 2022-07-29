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


import rospy
from std_srvs.srv import Trigger, TriggerResponse
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

class FakeDriver():

    def __init__(self):
        self.init_srv = rospy.Service('driver/init', Trigger, self.init_cb)
        self.recover_srv = rospy.Service('driver/recover', Trigger, self.recover_cb)
        self.halt_srv = rospy.Service('driver/halt', Trigger, self.halt_cb)
        self.shutdown_srv = rospy.Service('driver/shutdown', Trigger, self.shutdown_cb)

        self.initialized = False
        self.active = False

        self._fake_diag_pub = rospy.Publisher('/diagnostics', DiagnosticArray, queue_size=1)
        rospy.Timer(rospy.Duration(1.0), self.publish_diagnostics)

    def publish_diagnostics(self, event):
        msg = DiagnosticArray()
        msg.header.stamp = rospy.get_rostime()

        status = DiagnosticStatus()
        status.name = rospy.get_name()
        if self.initialized:
            if self.active:
                status.level = DiagnosticStatus.OK
                status.message = "active"
            else:
                status.level = DiagnosticStatus.WARN
                status.message = "not active"
        else:
            status.level = DiagnosticStatus.WARN
            status.message = "not initialized"
        status.hardware_id = rospy.get_name()
        status.values.append(KeyValue(key="initialized", value=str(self.initialized)))
        status.values.append(KeyValue(key="active", value=str(self.active)))
        msg.status.append(status)

        self._fake_diag_pub.publish(msg)

    def init_cb(self, req):
        rospy.loginfo("init_cb")
        self.initialized = True
        self.active = True
        return TriggerResponse(success=True)

    def recover_cb(self, req):
        rospy.loginfo("recover_cb")
        self.active = True
        return TriggerResponse(success=True)

    def halt_cb(self, req):
        rospy.loginfo("halt_cb")
        self.active = False
        return TriggerResponse(success=True)

    def shutdown_cb(self, req):
        rospy.loginfo("shutdown_cb")
        self.initialized = False
        self.active = False
        return TriggerResponse(success=True)


if __name__ == "__main__":
   rospy.init_node('fake_driver')
   FakeDriver()
   rospy.loginfo("fake_driver running")
   rospy.spin()


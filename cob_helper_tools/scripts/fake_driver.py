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
from std_srvs.srv import *
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus

class FakeDriver():

    def __init__(self):
        self.init_srv = rospy.Service('driver/init', Trigger, self.srv_cb)
        self.recover_srv = rospy.Service('driver/recover', Trigger, self.srv_cb)
        self.halt_srv = rospy.Service('driver/halt', Trigger, self.srv_cb)
        self.shutdown_srv = rospy.Service('driver/shutdown', Trigger, self.srv_cb)

        self._fake_diag_pub = rospy.Publisher('/diagnostics', DiagnosticArray, queue_size=1)
        rospy.Timer(rospy.Duration(1.0), self.publish_diagnostics)

    def publish_diagnostics(self, event):
        msg = DiagnosticArray()
        msg.header.stamp = rospy.get_rostime()

        status = DiagnosticStatus()
        status.name = rospy.get_name()
        status.level = DiagnosticStatus.OK
        status.message = "fake diagnostics"
        status.hardware_id = rospy.get_name()
        msg.status.append(status)

        self._fake_diag_pub.publish(msg)

    def srv_cb(self, req):
        resp = TriggerResponse()
        resp.success = True
        return resp


if __name__ == "__main__":
   rospy.init_node('fake_driver')
   FakeDriver()
   rospy.loginfo("fake_driver running")
   rospy.spin()


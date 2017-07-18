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

from cob_msgs.msg import EmergencyStopState
from diagnostic_msgs.msg import DiagnosticArray

from simple_script_server import *
sss = simple_script_server()

class AutoRecover():

  def __init__(self):
    self.components = rospy.get_param('~components', [])
    self.em_state = 0
    rospy.Subscriber("/emergency_stop_state", EmergencyStopState, self.em_cb)
    rospy.Subscriber("/diagnostics_agg", DiagnosticArray, self.diagnostics_cb)
    self.last_time_recover = rospy.Time.now()

  def em_cb(self, msg):
    if msg.emergency_state != self.em_state:
      if msg.emergency_state == 0:
        rospy.loginfo("auto_recover from scanner stop")
        self.recover()
      self.em_state = msg.emergency_state

  def recover(self):
    # call recover for all components
    for component in self.components:
      handle = sss.recover(component)
      if not (handle.get_error_code() == 0):
        rospy.logerr("[auto_recover]: Could not recover %s", component)
      else:
        rospy.loginfo("[auto_recover]: Component %s recovered successfully", component)
        self.last_time_recover = rospy.Time.now()

  # auto recover based on diagnostics
  def diagnostics_cb(self, msg):
    for status in msg.status:
      if status.level > 1: # only recover on error, not on warning status
        if "Actuators" in status.name and self.em_state == 0 and (rospy.Time.now() - self.last_time_recover) > rospy.Duration(10):
          rospy.loginfo("auto_recover from diagnostic failure")
          self.recover()

if __name__ == "__main__":
  rospy.init_node("auto_recover")
  AR = AutoRecover()
  rospy.loginfo("auto recover running")
  rospy.spin()

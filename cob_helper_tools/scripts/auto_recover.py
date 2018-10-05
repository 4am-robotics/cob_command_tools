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


import copy
import rospy

from cob_msgs.msg import EmergencyStopState
from diagnostic_msgs.msg import DiagnosticArray

from simple_script_server import *
sss = simple_script_server()

class AutoRecover():

  def __init__(self):
    now = rospy.Time.now()
    self.em_state = 0
    self.recover_emergency = rospy.get_parm('~recover_emergency', True)
    self.recover_diagnostics = rospy.get_parm('~recover_diagnostics', True)
    self.components = rospy.get_param('~components', {})
    self.components_recover_time = {}
    for component in self.components.keys():
      self.components_recover_time[component] = now
    rospy.Subscriber("/emergency_stop_state", EmergencyStopState, self.em_cb, queue_size=1)
    rospy.Subscriber("/diagnostics_agg", DiagnosticArray, self.diagnostics_cb, queue_size=1)

  # auto recover based on emergency stop
  def em_cb(self, msg):
    if not self.recover_emergency:
      return
    if msg.emergency_state == 0 and self.em_state != 0:
      rospy.loginfo("auto_recover from emergency state")
      self.recover(self.components.keys())
    self.em_state = copy.deepcopy(msg.emergency_state)

  # auto recover based on diagnostics
  def diagnostics_cb(self, msg):
    if not self.recover_diagnostics:
      return
    for status in msg.status:
      for component in self.components.keys():
        if status.name.lower().startswith(self.components[component].lower()) and status.level > 0 and self.em_state == 0 and (rospy.Time.now() - self.components_recover_time[component] > rospy.Duration(10)):
          rospy.loginfo("auto_recover from diagnostic failure")
          self.recover([component])

  def recover(self, components):
    for component in components:
      handle = sss.recover(component)
      if not (handle.get_error_code() == 0):
        rospy.logerr("[auto_recover]: Could not recover %s", component)
      else:
        rospy.loginfo("[auto_recover]: Component %s recovered successfully", component)
        self.components_recover_time[component] = rospy.Time.now()

if __name__ == "__main__":
  rospy.init_node("auto_recover")
  AR = AutoRecover()
  rospy.loginfo("auto recover running")
  rospy.spin()

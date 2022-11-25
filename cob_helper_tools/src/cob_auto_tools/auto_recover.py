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
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from std_srvs.srv import Trigger, TriggerResponse

from simple_script_server import *
sss = simple_script_server()

class AutoRecover():

  def __init__(self):
    now = rospy.Time.now()
    self.em_state = EmergencyStopState.EMFREE
    self.recover_emergency = rospy.get_param('~recover_emergency', True)
    self.recover_diagnostics = rospy.get_param('~recover_diagnostics', True)
    self.components = rospy.get_param('~components', {})
    self.components_recover_time = {}
    for component in list(self.components.keys()):
      self.components_recover_time[component] = now

    rospy.Service('~enable',Trigger,self.enable_cb)
    rospy.Service('~disable',Trigger,self.disable_cb)
    self.enabled = True
    self.subscribe()

  # auto recover based on emergency stop
  def em_cb(self, msg):
    if msg.emergency_state == EmergencyStopState.EMFREE and self.em_state != EmergencyStopState.EMFREE:
      if not self.recover_emergency:
        rospy.loginfo("auto_recover from emergency state is disabled")
      else:
        rospy.loginfo("auto_recover from emergency state")
        self.recover(list(self.components.keys()))
    self.em_state = copy.deepcopy(msg.emergency_state)

  # auto recover based on diagnostics
  def diagnostics_cb(self, msg):
    if self.recover_diagnostics:
      for status in msg.status:
        for component in list(self.components.keys()):
          if status.name.lower().startswith(self.components[component].lower()) and status.level > DiagnosticStatus.OK and self.em_state == EmergencyStopState.EMFREE and (rospy.Time.now() - self.components_recover_time[component] > rospy.Duration(10)):
              rospy.loginfo("auto_recover from diagnostic failure")
              self.recover([component])
    else:
      rospy.loginfo_once("auto_recover from diagnostic failure is disabled")  # pylint: disable=no-member

  # callback for enable service
  def enable_cb(self, req):
    if not self.enabled:
      self.enabled = True
      self.subscribe()
    return TriggerResponse(True, "auto recover enabled")

  # callback for disable service
  def disable_cb(self, req):
    if self.enabled:
      self.enabled = False
      self.unsubscribe()
    return TriggerResponse(True, "auto recover disabled")

  def subscribe(self):
    self.em_stop_state_subscriber = rospy.Subscriber("/emergency_stop_state", EmergencyStopState, self.em_cb, queue_size=1)
    self.diagnostics_subscriber = rospy.Subscriber("/diagnostics_agg", DiagnosticArray, self.diagnostics_cb, queue_size=1)

  def unsubscribe(self):
    self.em_stop_state_subscriber.unregister()
    self.diagnostics_subscriber.unregister()

  def recover(self, components):
    if self.enabled:
      for component in components:
        handle = sss.recover(component)
        if not (handle.get_error_code() == 0):
          rospy.logerr("[auto_recover]: Could not recover %s", component)
        else:
          rospy.loginfo("[auto_recover]: Component %s recovered successfully", component)
          self.components_recover_time[component] = rospy.Time.now()
          
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

from simple_script_server import *
sss = simple_script_server()

class AutoInit():

  def __init__(self):
    self.components = rospy.get_param('~components', {})
    self.max_retries = rospy.get_param('~max_retries', 50)
    self.em_state_ignore = rospy.get_param('~em_state_ignore', False)
    self.em_state = EmergencyStopState.EMSTOP
    rospy.Subscriber("/emergency_stop_state", EmergencyStopState, self.em_cb, queue_size=1)

    # wait for all components to start
    for component in list(self.components.keys()):
      rospy.loginfo("[auto_init]: Waiting for %s to start...", component)
      rospy.wait_for_service("/" + component + "/driver/init")

    # wait for emergency_stop to be released
    while not rospy.is_shutdown():
      if not self.em_state_ignore and self.em_state == EmergencyStopState.EMSTOP:
        rospy.loginfo("[auto_init]: Waiting for emergency stop to be released...")
        try:
          rospy.sleep(1)
        except rospy.exceptions.ROSInterruptException as e:
          pass
      else: # EMFREE or EMCONFIRMED
        # call init for all components
        rospy.loginfo("[auto_init]: Initializing components")
        for component in list(self.components.keys()):
          retries = 0
          while not rospy.is_shutdown():
            if self.max_retries > 0 and retries >= self.max_retries:
              rospy.logerr("[auto_init]: Could not initialize %s after %s retries", component, str(retries))
              break
            retries += 1
            handle = sss.init(component)
            if not (handle.get_error_code() == 0):
              rospy.logerr("[auto_init]: Could not initialize %s. Retrying...(%s of %s)", component, str(retries), str(self.max_retries))
              rospy.sleep(1.0)
            else:
              rospy.loginfo("[auto_init]: Component %s initialized successfully", component)
              break
        break # done

  def em_cb(self, msg):
    self.em_state = msg.emergency_state
    

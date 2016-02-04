#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# This file has been copied from https://github.com/PR2/pr2_common in order to support this feature for indigo indepenendly from PR2 dependencies
# The message definitions from pr2_msgs can be found in cob_common/cob_msgs

import time
import rospy
from cob_msgs.msg import EmergencyStopState, PowerState, DashboardState
from diagnostic_msgs.msg import DiagnosticStatus

class DashboardAggregator:
  def __init__(self):
    self.msg = DashboardState()

    # Create publisher
    self.pub = rospy.Publisher("dashboard_agg", DashboardState, queue_size=1)

    # Create subscribers
    # Diagnostics
    rospy.Subscriber("diagnostics_toplevel_state", DiagnosticStatus, self.DiagnosticStatusCB)
    # Power state
    rospy.Subscriber("power_state", PowerState, self.PowerStateCB)
    # Emergency stop state
    rospy.Subscriber("emergency_stop_state", EmergencyStopState, self.EmergencyStopStateCB)

  def DiagnosticStatusCB(self, msg):
    self.msg.diagnostics_toplevel_state = msg

  def PowerStateCB(self, msg):
    self.msg.power_state = msg

  def EmergencyStopStateCB(self, msg):
    self.msg.emergency_stop_state = msg

  def publish(self):
    now = time.time()
    self.pub.publish(self.msg)

if __name__ == "__main__":
  rospy.init_node("cob_dashboard_aggregator")
  rospy.sleep(1)
  da = DashboardAggregator()
  r = rospy.Rate(1)
  while not rospy.is_shutdown():
    da.publish()
    r.sleep()

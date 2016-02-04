# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Willow Garage, Inc.
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

import rospy

from python_qt_binding.QtCore import QSize
from rqt_robot_dashboard.widgets import BatteryDashWidget


class COBBattery(BatteryDashWidget):
    """
    Dashboard widget to display COB battery state.
    """
    #TODO When nonbutton Dashboard objects are available rebase this widget
    def __init__(self, context):
        """
        :param context: the plugin context
        :type context: qt_gui.plugin.Plugin
        """
        super(COBBattery, self).__init__('COB Battery')
        self._time_remaining = 0.0
        self._charging = False

        self.setFixedSize(self._icons[1].actualSize(QSize(50, 30)))

        self.update_perc(0)

    def set_power_state(self, msg):
        """
        Sets button state based on msg

        :param msg: message containing the power state of the COB
        :type msg: cob_msgs.PowerState
        """
        last_charging = self._charging
        last_time_remaining = self._time_remaining

        self._time_remaining = msg.time_remaining
        self._charging = msg.charging
        if (last_charging != self._charging or last_time_remaining != self._time_remaining):
            drain_str = "remaining"
            if (self._charging):
                drain_str = "to full charge"
                self.charging = True
            self.setToolTip("Battery: %.2f%% \nTime %s: %d Minutes" % (msg.relative_remaining_capacity, drain_str, self._time_remaining * 60.0))
            self.update_perc(msg.relative_remaining_capacity)

    def set_stale(self):
        self._charging = 0
        self._time_remaining = rospy.rostime.Duration(0)
        self.setToolTip("Battery: Stale")

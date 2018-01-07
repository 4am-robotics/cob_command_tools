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

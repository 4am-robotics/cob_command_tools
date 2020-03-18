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


import roslib
import rospy

from cob_msgs.msg import  DashboardState

from rqt_robot_dashboard.dashboard import Dashboard
from rqt_robot_dashboard.widgets import MonitorDashWidget, ConsoleDashWidget

from python_qt_binding.QtCore import QSize
try:
    from python_qt_binding.QtWidgets import QMessageBox  # Qt 5
except ImportError:
	from python_qt_binding.QtGui import QMessageBox # Qt 4

from .cob_battery import COBBattery
from .cob_runstops import COBRunstops


class CobDashboard(Dashboard):
    """
    Dashboard for Care-O-bots

    :param context: the plugin context
    :type context: qt_gui.plugin.Plugin
    """
    def setup(self, context):
        self.name = 'CobDashboard'
        self.max_icon_size = QSize(50, 30)
        self.message = None

        self._dashboard_message = None
        self._last_dashboard_message_time = 0.0

        self._raw_byte = None
        self.digital_outs = [0, 0, 0]

        self._console = ConsoleDashWidget(self.context, minimal=False)
        self._monitor = MonitorDashWidget(self.context)
        self._runstop = COBRunstops('RunStops')
        self._battery = COBBattery(self.context)

        self._dashboard_agg_sub = rospy.Subscriber("/dashboard_agg", DashboardState, self.db_agg_cb)

    def get_widgets(self):
        return [[self._monitor, self._console], [self._runstop], [self._battery]]

    def db_agg_cb(self, msg):
        self._last_dashboard_message_time = rospy.get_time()
        self._battery.set_power_state(msg.power_state)

        if(msg.emergency_stop_state.emergency_state == 0):
            self._runstop.set_ok()
            self._runstop.setToolTip(self.tr("Button stop: OK\nScanner stop: OK"))
        else:
            if msg.emergency_stop_state.emergency_button_stop:
                self._runstop.set_button_stop()
            elif msg.emergency_stop_state.scanner_stop:
                self._runstop.set_scanner_stop()
            else:
                rospy.logerr("reason for emergency stop not known")
            self._runstop.setToolTip(self.tr("Button stop: %s\nScanner stop: %s" %(str(msg.emergency_stop_state.emergency_button_stop), str(msg.emergency_stop_state.scanner_stop))))

    def shutdown_dashboard(self):
        self._dashboard_agg_sub.unregister()

    def save_settings(self, plugin_settings, instance_settings):
        self._console.save_settings(plugin_settings, instance_settings)
        self._monitor.save_settings(plugin_settings, instance_settings)

    def restore_settings(self, plugin_settings, instance_settings):
        self._console.restore_settings(plugin_settings, instance_settings)
        self._monitor.restore_settings(plugin_settings, instance_settings)

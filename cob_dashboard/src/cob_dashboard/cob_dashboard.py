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

import roslib
import rospy

from cob_msgs.msg import EmergencyStopState
from cob_msgs.msg import PowerState, DashboardState

from rqt_robot_dashboard.dashboard import Dashboard
from rqt_robot_dashboard.widgets import MonitorDashWidget, ConsoleDashWidget

from python_qt_binding.QtCore import QSize
from python_qt_binding.QtGui import QMessageBox

from cob_battery import COBBattery

from cob_runstops import COBRunstops


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
        self._batteries = [COBBattery(self.context)]

        self._dashboard_agg_sub = rospy.Subscriber("/dashboard_agg", DashboardState, self.db_agg_cb)
        self.em_sub = rospy.Subscriber("/emergency_stop_state", EmergencyStopState, self.emcb)

        self._powerstate_sub = rospy.Subscriber('/power_state', PowerState, self.dashboard_callback)

    def get_widgets(self):
        return [[self._monitor, self._console], [self._runstop], self._batteries]

    
    def db_agg_cb(self, msg):
        pass
    
    def emcb(self, msg):
        self._last_dashboard_message_time = rospy.get_time()
        if(not msg.emergency_button_stop):
            self._runstop.set_button_engaged()
            self._runstop.setToolTip(self.tr("Button Runstop: Pressed"))
        if(not msg.scanner_stop):
            self._runstop.set_scanner_engaged()
            self._runstop.setToolTip(self.tr("Button Runstop: Unknown\nScanner Runstop: Active"))
        if(msg.emergency_button_stop and msg.scanner_stop):
            self._runstop.set_ok()
            self._runstop.setToolTip(self.tr("Button Runstop: OK\nScanner Runstop: OK"))

    def dashboard_callback(self, msg):
        self._last_dashboard_message_time = rospy.get_time()
        self._batteries[0].set_power_state(msg)

    def shutdown_dashboard(self):
        self._dashboard_agg_sub.unregister()

    def save_settings(self, plugin_settings, instance_settings):
        self._console.save_settings(plugin_settings, instance_settings)
        self._monitor.save_settings(plugin_settings, instance_settings)

    def restore_settings(self, plugin_settings, instance_settings):
        self._console.restore_settings(plugin_settings, instance_settings)
        self._monitor.restore_settings(plugin_settings, instance_settings)

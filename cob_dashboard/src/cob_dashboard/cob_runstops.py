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


from python_qt_binding.QtCore import QSize

from rqt_robot_dashboard.widgets import IconToolButton


class COBRunstops(IconToolButton):
    """
    Dashboard widget to display Care-O-bot Runstop state.
    """
    def __init__(self, context):
        """
        :param context: the plugin context
        :type context: qt_gui.plugin.Plugin
        """

        ok_icon = ['bg-green.svg', 'ic-runstop-off.svg']
        button_engaged_icon = ['bg-red.svg', 'ic-runstop-on.svg']
        scanner_engaged_icon = ['bg-red.svg', 'ic-wireless-runstop-on.svg']
        stale_icon = ['bg-grey.svg', 'ic-runstop-off.svg', 'ol-stale-badge.svg']

        icons = [ok_icon, button_engaged_icon, scanner_engaged_icon, stale_icon]
        super(COBRunstops, self).__init__('Runstop', icons, icons)
        self.setToolTip('Runstop')
        self.set_stale()
        self.setFixedSize(self._icons[0].actualSize(QSize(50, 30)))

    def set_ok(self):
        self.update_state(0)

    def set_button_stop(self):
        self.update_state(1)

    def set_scanner_stop(self):
        self.update_state(2)

    def set_stale(self):
        self.update_state(3)

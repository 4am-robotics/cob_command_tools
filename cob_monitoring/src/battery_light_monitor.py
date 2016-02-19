#!/usr/bin/python
#
# \file
#
# \note
#   Copyright (c) 2016 \n
#   Fraunhofer Institute for Manufacturing Engineering
#   and Automation (IPA) \n\n
#
#
#
# \note
#   Project name: care-o-bot
# \note
#   ROS stack name: cob_command_tools
# \note
#   ROS package name: cob_monitoring
#
# \author
#   Author: Benjamin Maidel
# \author
#   Supervised by:
#
# \date Date of creation: JAN 2015
#
# \brief
#   Monitors the battery level and announces warnings and reminders to recharge.
#
#
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     - Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer. \n
#     - Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution. \n
#     - Neither the name of the Fraunhofer Institute for Manufacturing
#       Engineering and Automation (IPA) nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission. \n
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License LGPL as
# published by the Free Software Foundation, either version 3 of the
# License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU Lesser General Public License LGPL for more details.
#
# You should have received a copy of the GNU Lesser General Public
# License LGPL along with this program.
# If not, see <http://www.gnu.org/licenses/>.
#
#

import rospy
import colorsys
import copy

from cob_msgs.msg import *
from cob_light.msg import *
from std_msgs.msg import *
from cob_light.srv import *


class battery_light_monitor():

    def __init__(self):
        self.power_state = PowerState()
        self.relative_remaining_capacity = 0.0
        self.temperature = 0.0
        self.is_charging = False
        self.num_leds = rospy.get_param('num_leds', 1)
        self.service_name = 'set_light'
        self.topic_name = 'power_state'
        self.track_id_light = {}

        if not rospy.has_param("~led_components"):
            rospy.logwarn("parameter led_components does not exist on ROS Parameter Server")
            return
        self.light_components = rospy.get_param("~led_components")
        for component in self.light_components:
            self.track_id_light[component] = None

        self.mode = LightMode()
        self.mode.priority = 2

        self.last_time_warned = rospy.get_time()

        rospy.Subscriber(self.topic_name, PowerState, self.power_callback)

        rospy.Timer(rospy.Duration(1), self.timer_callback)

    def power_callback(self, msg):
        self.power_state = msg

    def set_light(self, mode, track=False):
        for component in self.light_components:
            action_server_name = component + "/set_light"
            client = actionlib.SimpleActionClient(action_server_name, SetLightModeAction)
            # trying to connect to server
            rospy.logdebug("waiting for %s action server to start",action_server_name)
            if not client.wait_for_server(rospy.Duration(5)):
                # error: server did not respond
                rospy.logerr("%s action server not ready within timeout, aborting...", action_server_name)
            else:
                rospy.logdebug("%s action server ready",action_server_name)

                # sending goal
                goal = SetLightModeGoal()
                goal.mode = mode
                client.send_goal(goal)
                client.wait_for_result()
                res = client.getResult()
                if track:
                    self.track_id_light[component] = res.track_id

    def stop_light(self):
        for component in self.light_components:
            if self.track_id_light[component] is not None:
                srv_server_name = component + "/stop_light"
                try:
                    rospy.wait_for_service('srv_server_name', timeout=2)
                    srv_proxy = rospy.ServiceProxy(srv_server_name, StopLightMode)
                    req = StopLightModeRequest()
                    req.track_id = self.track_id_light[component]
                    srv_proxy(req)
                    self.track_id_light[component] = None
                except Exception as e:
                    rospy.logerr("%s service failed: %s",srv_server_name, e)

    def timer_callback(self, event):
        # warn if battery is empty
        if self.is_charging == False:
            # 5%
            if self.power_state.relative_remaining_capacity <= 5 and (rospy.get_time() - self.last_time_warned) > 5:
                self.last_time_warned = rospy.get_time()
                mode = copy.copy(self.mode)
                mode.mode = LightModes.FLASH
                color = ColorRGBA(1, 0, 0, 1)
                mode.colors = []
                mode.colors.append(color)
                mode.frequency = 5
                mode.pulses = 4
                self.set_light(mode)

            # 10%
            elif self.power_state.relative_remaining_capacity <= 10 and (rospy.get_time() - self.last_time_warned) > 15:
                self.last_time_warned = rospy.get_time()
                mode = copy.copy(self.mode)
                mode.mode = LightModes.FLASH
                color = ColorRGBA(1, 0, 0, 1)
                mode.colors = []
                mode.colors.append(color)
                mode.frequency = 2
                mode.pulses = 2
                self.set_light(mode)
            # 20%
            elif self.power_state.relative_remaining_capacity <= 20 and (rospy.get_time() - self.last_time_warned) > 30:
                self.last_time_warned = rospy.get_time()
                mode = copy.copy(self.mode)
                mode.mode = LightModes.FLASH
                color = ColorRGBA(1, 1, 0, 1)
                mode.colors = []
                mode.colors.append(color)
                mode.frequency = 2
                mode.pulses = 2
                self.set_light(mode)

        if self.is_charging == False and self.power_state.charging == True:
            self.is_charging = True

        if self.is_charging == True and self.power_state.charging == False:
            self.is_charging = False
            self.relative_remaining_capacity = 0.0
            self.stop_light()

        elif self.is_charging == True:
            # only change color mode if capacity change is bigger than 2%
            if abs(self.relative_remaining_capacity - self.power_state.relative_remaining_capacity) > 2:
                rospy.logdebug('adjusting leds')
                mode = copy.copy(self.mode)
                if self.num_leds > 1:
                    leds = int(self.num_leds * self.power_state.relative_remaining_capacity / 100.)
                    mode.mode = LightModes.CIRCLE_COLORS
                    mode.frequency = 60.0
                    mode.colors = []
                    color = ColorRGBA(0.0, 1.0, 0.7, 0.4)
                    for i in range(leds):
                        mode.colors.append(self.color)
                else:
                    mode.mode = LightModes.BREATH
                    mode.frequency = 0.4
                    # 0.34 => green in hsv space
                    hue = 0.34 * (self.power_state.relative_remaining_capacity / 100.0)
                    rgb = colorsys.hsv_to_rgb(hue, 1, 1)
                    color = ColorRGBA(rgb[0], rgb[1], rgb[2], 1.0)
                    mode.colors = []
                    mode.colors.append(color)

                self.relative_remaining_capacity = self.power_state.relative_remaining_capacity
                self.set_light(mode, True)


if __name__ == "__main__":
    rospy.init_node("battery_light_monitor")
    battery_light_monitor()
    rospy.spin()

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
import colorsys
import copy
import actionlib

from std_msgs.msg import ColorRGBA
from cob_msgs.msg import PowerState
from cob_light.msg import LightMode, LightModes, SetLightModeAction, SetLightModeGoal
from cob_light.srv import StopLightMode, StopLightModeRequest

from simple_script_server import *
sss = simple_script_server()


class battery_monitor():

    def __init__(self):
        self.power_state = PowerState()
        self.relative_remaining_capacity = 0.0
        self.temperature = 0.0
        self.is_charging = False
        self.topic_name = 'power_state'
        self.mode = LightMode()

        self.threshold_warning = rospy.get_param("~threshold_warning", 20.0) # % of battery level
        self.threshold_error = rospy.get_param("~threshold_error", 10.0)     # % of battery level
        self.threshold_critical = rospy.get_param("~threshold_critical", 5.0)# % of battery level

        self.enable_light = rospy.get_param("~enable_light", True)
        self.num_leds = rospy.get_param("~num_leds", 1)
        self.track_id_light = {}
        if self.enable_light:
            if not rospy.has_param("~light_components"):
                rospy.logwarn("parameter light_components does not exist on ROS Parameter Server")
                return
            self.light_components = rospy.get_param("~light_components")
            for component in self.light_components:
                self.track_id_light[component] = None
            self.mode = LightMode()
            self.mode.priority = 8

        self.enable_sound = rospy.get_param("~enable_sound", True)
        self.sound_components = {}
        if self.enable_sound:
            if not rospy.has_param("~sound_components"):
                rospy.logwarn("parameter sound_components does not exist on ROS Parameter Server")
                return
            self.sound_components = rospy.get_param("~sound_components")

        self.sound_critical = rospy.get_param("~sound_critical", "My battery is empty, please recharge now.")
        self.sound_warning = rospy.get_param("~sound_warning", "My battery is nearly empty, please consider recharging.")

        self.last_time_warned = rospy.get_time()
        self.last_time_power_received = rospy.get_time()
        rospy.Subscriber(self.topic_name, PowerState, self.power_callback)

        rospy.Timer(rospy.Duration(1), self.timer_callback)

    def power_callback(self, msg):
        self.last_time_power_received = rospy.get_time()
        self.power_state = msg

    def set_light(self, mode, track=False):
        if self.enable_light:
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
                    res = client.get_result()
                    if track:
                        self.track_id_light[component] = res.track_id

    def stop_light(self):
        if self.enable_light:
            for component in self.light_components:
                if self.track_id_light[component] is not None:
                    srv_server_name = component + "/stop_mode"
                    try:
                        rospy.wait_for_service(srv_server_name, timeout=2)
                        srv_proxy = rospy.ServiceProxy(srv_server_name, StopLightMode)
                        req = StopLightModeRequest()
                        req.track_id = self.track_id_light[component]
                        srv_proxy(req)
                        self.track_id_light[component] = None
                    except Exception as e:
                        rospy.logerr("%s service failed: %s",srv_server_name, e)

    def say(self, text):
        if self.enable_sound:
            for component in self.sound_components:
                sss.say(component, [text])

    def timer_callback(self, event):
        # warn if battery is empty
        if not self.is_charging:
            # 5%
            if self.power_state.relative_remaining_capacity <= self.threshold_critical and (rospy.get_time() - self.last_time_warned) > 5:
                self.last_time_warned = rospy.get_time()
                mode = copy.copy(self.mode)
                mode.mode = LightModes.FLASH
                color = ColorRGBA(1, 0, 0, 1)
                mode.colors = []
                mode.colors.append(color)
                mode.frequency = 5
                mode.pulses = 4
                self.set_light(mode)

                self.say(self.sound_critical)

            # 10%
            elif self.power_state.relative_remaining_capacity <= self.threshold_error and (rospy.get_time() - self.last_time_warned) > 15:
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
            elif self.power_state.relative_remaining_capacity <= self.threshold_warning and (rospy.get_time() - self.last_time_warned) > 30:
                self.last_time_warned = rospy.get_time()
                mode = copy.copy(self.mode)
                mode.mode = LightModes.FLASH
                color = ColorRGBA(1, 1, 0, 1)
                mode.colors = []
                mode.colors.append(color)
                mode.frequency = 2
                mode.pulses = 2
                self.set_light(mode)

                self.say(self.sound_warning)

        if self.is_charging == False and self.power_state.charging == True:
            self.is_charging = True

        if self.is_charging == True and (self.power_state.charging == False
                                    or (rospy.get_time() - self.last_time_power_received) > 2):
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
                    for _ in range(leds):
                        mode.colors.append(color)
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
    rospy.init_node("battery_monitor")
    battery_monitor()
    rospy.spin()

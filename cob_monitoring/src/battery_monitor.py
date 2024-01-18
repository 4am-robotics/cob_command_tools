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
from cob_msgs.msg import PowerState, HornMode
from cob_srvs.srv import SetHornMode, SetHornModeRequest
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

        self.enable_sound = rospy.get_param("~enable_sound", True)
        self.enable_horn = rospy.get_param("~enable_horn", True)

        ### light parameter ###
        self.enable_light = rospy.get_param("~enable_light", True)
        self.num_leds = rospy.get_param("~num_leds", 1)

        if rospy.has_param("~color_charging"):
            charging_color_param = rospy.get_param("~color_charging")
        else:
            # set to cyan as default
            charging_color_param = [0.0, 1.0, 0.7, 0.4]
        self.color_charging = ColorRGBA(*charging_color_param)

        if rospy.has_param("~color_warning"):
            color_warning_param = rospy.get_param("~color_warning")
        else:
            # set to cyan as default
            color_warning_param = [1.0, 1.0, 0.0, 1.0]
        self.color_warning = ColorRGBA(*color_warning_param)

        if rospy.has_param("~color_error"):
            color_error_param = rospy.get_param("~color_error")
        else:
            # set to cyan as default
            color_error_param = [1.0, 0.0, 0.0, 1.0]
        self.color_error = ColorRGBA(*color_error_param)

        if rospy.has_param("~color_critical"):
            color_critical_param = rospy.get_param("~color_critical")
        else:
            # set to cyan as default
            color_critical_param = [1.0, 0.0, 0.0, 1.0]
        self.color_critical = ColorRGBA(*color_critical_param)


        ### Set warning intervals ###
        self.horn_interval_duration_warning = rospy.get_param("~horn_interval_duration_warning", 1200.0)    # default: 20 minutes
        self.horn_interval_duration_error = rospy.get_param("~horn_interval_duration_error", 600.0)         # default: 10 minutes
        self.horn_interval_duration_critical = rospy.get_param("~horn_interval_duration_critical", 300.0)   # default: 5 minutes

        self.light_interval_duration_warning = rospy.get_param("~light_interval_duration_warning", 30.0)
        self.light_interval_duration_error = rospy.get_param("~light_interval_duration_error", 15.0)
        self.light_interval_duration_critical = rospy.get_param("~light_interval_duration_critical", 5.0)

        ### horn parameters ###
        self.set_horn_service = rospy.get_param("~set_horn_service", "/mojin_horn/set_horn_mode")
        self.horn_frequency = rospy.get_param("~horn_frequency", 1.0)
        self.horn_pulses_warning = rospy.get_param("~horn_pulses_warning", 1)
        self.horn_pulses_error = rospy.get_param("~horn_pulses_error", 2)
        self.horn_pulses_critical = rospy.get_param("~horn_pulses_critical", 3)
        self.horn_timeout = rospy.get_param("~horn_timeout", 5.0)

        self.srv_set_horn = rospy.ServiceProxy(self.set_horn_service, SetHornMode)


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

        self.sound_components = {}
        if self.enable_sound:
            if not rospy.has_param("~sound_components"):
                rospy.logwarn("parameter sound_components does not exist on ROS Parameter Server")
                return
            self.sound_components = rospy.get_param("~sound_components")

        self.sound_critical = rospy.get_param("~sound_critical", "My battery is empty, please recharge now.")
        self.sound_warning = rospy.get_param("~sound_warning", "My battery is nearly empty, please consider recharging.")

        self.last_time_warned_light = rospy.get_time()
        self.last_time_warned_horn = rospy.get_time()
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

    def trigger_horn(self, horn_pulses):

        if self.enable_horn != True:
            rospy.logdebug(f"could not signal horn, not enabled")
            return

        req = SetHornModeRequest()
        req.mode.mode = HornMode.FLASH
        req.mode.frequency = self.horn_frequency
        req.mode.pulses = horn_pulses
        req.mode.timeout = self.horn_timeout
        try:
            self.srv_set_horn.call(req)
        except (TypeError, rospy.ServiceException) as e:
            rospy.logwarn(f"Received exception when calling '{self.set_horn_service}': {e}")
            return

    def timer_callback(self, event):
        # warn if battery is empty
        if not self.is_charging:

            # 5%
            if self.power_state.relative_remaining_capacity <= self.threshold_critical:
                if (rospy.get_time() - self.last_time_warned_light) > self.light_interval_duration_critical:
                    self.last_time_warned_light = rospy.get_time()
                    mode = copy.copy(self.mode)
                    mode.mode = LightModes.FLASH
                    color = self.color_critical
                    mode.colors = []
                    mode.colors.append(color)
                    mode.frequency = 4
                    mode.pulses = 8
                    self.set_light(mode)
                    self.say(self.sound_critical)
                if (rospy.get_time() - self.last_time_warned_horn) > self.horn_interval_duration_critical:
                    self.last_time_warned_horn = rospy.get_time()
                    self.trigger_horn(self.horn_pulses_critical)

            # 10%
            elif self.power_state.relative_remaining_capacity <= self.threshold_error:
                if (rospy.get_time() - self.last_time_warned_light) > self.light_interval_duration_error:
                    self.last_time_warned_light = rospy.get_time()
                    mode = copy.copy(self.mode)
                    mode.mode = LightModes.FLASH
                    color = self.color_error
                    mode.colors = []
                    mode.colors.append(color)
                    mode.frequency = 2
                    mode.pulses = 2
                    self.set_light(mode)
                    self.say(self.sound_critical)
                if (rospy.get_time() - self.last_time_warned_horn > self.horn_interval_duration_error):
                    self.last_time_warned_horn = rospy.get_time()
                    self.trigger_horn(self.horn_pulses_error)

            # 20%
            elif self.power_state.relative_remaining_capacity <= self.threshold_warning:
                if (rospy.get_time() - self.last_time_warned_light) > self.light_interval_duration_warning:
                    self.last_time_warned_light = rospy.get_time()
                    mode = copy.copy(self.mode)
                    mode.mode = LightModes.FLASH
                    color = self.color_warning
                    mode.colors = []
                    mode.colors.append(color)
                    mode.frequency = 2
                    mode.pulses = 2
                    self.set_light(mode)
                    self.say(self.sound_warning)
                if (rospy.get_time() - self.last_time_warned_horn > self.horn_interval_duration_warning):
                    self.last_time_warned_horn = rospy.get_time()
                    self.trigger_horn(self.horn_pulses_warning)

        if self.is_charging == False and self.power_state.charging == True:
            self.is_charging = True

        if self.is_charging == True and (self.power_state.charging == False or (rospy.get_time() - self.last_time_power_received) > 2):
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
                    color = self.color_charging
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

#!/usr/bin/python
#################################################################
##\file
#
# \note
#   Copyright (c) 2016 \n
#   Fraunhofer Institute for Manufacturing Engineering
#   and Automation (IPA) \n\n
#
#################################################################
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
#################################################################
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
#################################################################

import rospy
import colorsys

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

    rospy.logdebug("waiting for service server")
    rospy.wait_for_service(self.service_name)
    rospy.logdebug("found service server")
    try:
      self.sproxy = rospy.ServiceProxy(self.service_name, SetLightMode)
    except rospy.ServiceException, e:
      print "Service proxy failed: %s"%e
      return

    self.goal = SetLightModeGoal()
    self.mode = LightMode()
    self.mode.mode = self.mode.CIRCLE_COLORS
    self.mode.frequency = 60.0
    self.mode.priority = 0
    self.goal.mode = self.mode

    self.color = ColorRGBA()
    self.color.r = 0.0
    self.color.g = 1.0
    self.color.b = 0.7
    self.color.a = 0.4

    self.last_time_warned = rospy.get_time()

    rospy.Subscriber(self.topic_name, PowerState, self.power_callback)

    rospy.Timer(rospy.Duration(1), self.timer_callback)

  def power_callback(self,msg):
    self.power_state = msg

  def timer_callback(self,event):
    #warn if battery is empty
    if self.is_charging == False:
      #below 10%
      if self.power_state.relative_remaining_capacity <= 10 and (rospy.get_time() - self.last_time_warned) > 5:
        self.last_time_warned = rospy.get_time()
        self.mode.mode = self.mode.FLASH
        self.mode.color.r=1
        self.mode.color.g=0
        self.mode.color.b=0
        self.mode.color.a=1
        self.mode.colors = []
        self.mode.frequency = 2
        self.mode.pulses = 2
        #below 5%
        if self.power_state.relative_remaining_capacity <= 5:
          self.mode.frequency = 5
          self.mode.pulses = 4
        self.sproxy(self.mode)

    if self.is_charging == False and self.power_state.charging == True:
      self.is_charging = True

    if self.is_charging == True and self.power_state.charging == False:
      self.is_charging = False
      self.relative_remaining_capacity = 0.0
      self.mode.mode = self.mode.BREATH
      self.mode.color = self.color
      self.mode.colors = []
      self.mode.frequency = 0.25
      self.sproxy(self.mode)
    elif self.is_charging == True:
      #only change color mode if capacity change is bigger than 2%
      if abs(self.relative_remaining_capacity - self.power_state.relative_remaining_capacity) > 2:
        rospy.logdebug('adjusting leds')
        if self.num_leds > 1:
          leds = int(self.num_leds * self.power_state.relative_remaining_capacity / 100.)
          self.mode.mode = self.mode.CIRCLE_COLORS
          self.mode.frequency = 60.0
          self.mode.colors = []
          for i in range(leds):
            self.mode.colors.append(self.color)
        else:
          self.mode.mode = self.mode.BREATH
          self.mode.frequency = 0.4
          #0.34 => green in hsv space
          hue = 0.34 * (self.power_state.relative_remaining_capacity/100.0)
          rgb = colorsys.hsv_to_rgb(hue, 1, 1)
          self.mode.colors = []
          self.mode.color.r = rgb[0]
          self.mode.color.g = rgb[1]
          self.mode.color.b = rgb[2]
          self.mode.color.a = 1.0

        self.relative_remaining_capacity = self.power_state.relative_remaining_capacity
        self.sproxy(self.mode)


if __name__ == "__main__":
  rospy.init_node("battery_light_monitor")
  battery_light_monitor()
  rospy.spin()

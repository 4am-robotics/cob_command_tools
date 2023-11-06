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


import sys
import copy
from enum import IntEnum

import rospy
from std_msgs.msg import Empty
from sensor_msgs.msg import JointState
from diagnostic_msgs.msg import DiagnosticStatus, DiagnosticArray

from cob_msgs.msg import *
from cob_light.msg import LightMode, LightModes, SetLightModeGoal, SetLightModeAction
from cob_light.srv import StopLightMode, StopLightModeRequest

from simple_script_server import *
sss = simple_script_server()

class EMState(IntEnum):
	EM_UNKNOWN	= -1
	EM_FREE		=  0
	EM_BUTTON	=  1
	EM_BRAKE	=  2
	EM_LASER	=  3
	EM_WIRELESS	=  4
	EM_FALL		=  5
	EM_INTERNAL	=  6


class emergency_stop_monitor():
	def __init__(self):
		self.color = "None"
		self.enable_sound = rospy.get_param("~enable_sound", False)
		self.enable_light = rospy.get_param("~enable_light", False)
		self.diagnostics_based = rospy.get_param("~diagnostics_based", False)
		self.motion_based = rospy.get_param("~motion_based", False)
		self.topic_based = rospy.get_param("~topic_based", False)
		self.track_id_light = None

		if(self.enable_light):
			if not rospy.has_param("~light_components"):
				rospy.logwarn("parameter light_components does not exist on ROS Parameter Server, aborting...")
				sys.exit(1)
			self.light_components = rospy.get_param("~light_components")

		# set colors (also neccessary if self.enable_light is false)
		self.color_error = rospy.get_param("~color_error","red")
		self.color_warn = rospy.get_param("~color_warn","yellow")
		self.color_ok = rospy.get_param("~color_ok","green")
		self.color_off = rospy.get_param("~color_off","black")

		if(self.enable_sound):
			if not rospy.has_param("~sound_components"):
				rospy.logwarn("parameter sound_components does not exist on ROS Parameter Server, aborting...")
				sys.exit(1)
			self.sound_components = rospy.get_param("~sound_components")

		self.sound_em_button_released = rospy.get_param("~sound_em_button_released", "button released")
		self.sound_em_button_issued = rospy.get_param("~sound_em_button_issued", "button issued")
		self.sound_em_brake_released = rospy.get_param("~sound_em_brake_released", "brake released")
		self.sound_em_brake_issued = rospy.get_param("~sound_em_brake_issued", "brake issued")
		self.sound_em_laser_released = rospy.get_param("~sound_em_laser_released", "laser released")
		self.sound_em_laser_issued = rospy.get_param("~sound_em_laser_issued", "laser issued")
		self.sound_em_wireless_released = rospy.get_param("~sound_em_wireless_released", "wireless released")
		self.sound_em_wireless_issued = rospy.get_param("~sound_em_wireless_issued", "wireless issued")
		self.sound_em_fall_released = rospy.get_param("~sound_em_fall_released", "fall released")
		self.sound_em_fall_issued = rospy.get_param("~sound_em_fall_issued", "fall issued")
		self.sound_em_internal_released = rospy.get_param("~sound_em_internal_released", "internal released")
		self.sound_em_internal_issued = rospy.get_param("~sound_em_internal_issued", "internal issued")

		self.em_state = EMState.EM_UNKNOWN
		self.last_em_topic_state = EmergencyStopState.EMFREE

		if not self.topic_based:
			self.diag_status = -1
			rospy.Subscriber("/diagnostics_agg", DiagnosticArray, self.diagnostics_agg_callback, queue_size=1)
		else:
			self.diag_status = DiagnosticStatus.OK

		if(self.diagnostics_based):
			rospy.Subscriber("/diagnostics_toplevel_state", DiagnosticStatus, self.diagnostics_callback, queue_size=1)
			self.last_diag = rospy.get_rostime()

		self.motion_status = -1
		if(self.motion_based):
			rospy.Subscriber("/joint_states", JointState, self.jointstate_callback, queue_size=1)
			self.last_vel = rospy.get_rostime()

		if(self.topic_based):
			emergency_stop_topic = rospy.get_param("~emergency_stop_topic", "/emergency_stop_state")
			rospy.Subscriber(emergency_stop_topic, EmergencyStopState, self.emergency_stop_callback, queue_size=1)
			self.last_topic = rospy.get_rostime()

		self.pub_em_button_released   = rospy.Publisher('em_button_released'  , Empty, queue_size=1)
		self.pub_brake_released       = rospy.Publisher('brake_released'      , Empty, queue_size=1)
		self.pub_em_laser_released    = rospy.Publisher('em_laser_released'   , Empty, queue_size=1)
		self.pub_em_wireless_released = rospy.Publisher('em_wireless_released', Empty, queue_size=1)
		self.pub_em_fall_released     = rospy.Publisher('em_fall_released'    , Empty, queue_size=1)
		self.pub_em_internal_released = rospy.Publisher('em_internal_released', Empty, queue_size=1)

	def diagnostics_agg_callback(self, msg):
		diagnostics_errors = []
		diagnostics_errors_detail = []

		#all diagnostics that have warning/error/stale
		for status in msg.status:
			if "Safety" in status.name:
				if status.level > DiagnosticStatus.WARN:
					diagnostics_errors.append(status)

		#reduce list to most detailed description only
		for i, status in enumerate(diagnostics_errors):
			tmp = diagnostics_errors[:i]+diagnostics_errors[i+1:]
			if not next((True for status2 in tmp if status2.name.startswith(status.name)), False):
				diagnostics_errors_detail.append(status)

		if not diagnostics_errors_detail:
			if self.em_state > EMState.EM_FREE:
				self.stop_light()
				if self.em_state == EMState.EM_BUTTON:
					self.pub_em_button_released.publish(Empty())
					self.say(self.sound_em_button_released)
				elif self.em_state == EMState.EM_BRAKE:
					self.pub_brake_released.publish(Empty())
					self.say(self.sound_em_brake_released)
				elif self.em_state == EMState.EM_LASER:
					self.pub_em_laser_released.publish(Empty())
					self.say(self.sound_em_laser_released)
				elif self.em_state == EMState.EM_WIRELESS:
					self.pub_em_wireless_released.publish(Empty())
					self.say(self.sound_em_wireless_released)
				elif self.em_state == EMState.EM_FALL:
					self.pub_em_fall_released.publish(Empty())
					self.say(self.sound_em_fall_released)
				elif self.em_state == EMState.EM_INTERNAL:
					self.pub_em_internal_released.publish(Empty())
					self.say(self.sound_em_internal_released)
			self.em_state = EMState.EM_FREE

		#determine error state
		sound_output = ""
		em_state = EMState.EM_UNKNOWN
		for status in diagnostics_errors_detail:
			if "Safety" in status.name:
				for value in status.values:
					if (value.key == "button_stop_ok"):
						if (value.value == "False"):
							if em_state <= EMState.EM_FREE or em_state > EMState.EM_BUTTON:
								sound_output = self.sound_em_button_issued
								em_state = EMState.EM_BUTTON
					if (value.key == "brake_stop_ok"):
						if (value.value == "False"):
							if em_state <= EMState.EM_FREE or em_state > EMState.EM_BRAKE:
								sound_output = self.sound_em_brake_issued
								em_state = EMState.EM_BRAKE
					if (value.key == "laser_stop_ok"):
						if (value.value == "False"):
							if em_state <= EMState.EM_FREE or em_state > EMState.EM_LASER:
								sound_output = self.sound_em_laser_issued
								em_state = EMState.EM_LASER
					if (value.key == "wireless_stop_ok"):
						if (value.value == "False"):
							if em_state <= EMState.EM_FREE or em_state > EMState.EM_WIRELESS:
								sound_output = self.sound_em_wireless_issued
								em_state = EMState.EM_WIRELESS
					if (value.key == "fall_sensor_front_ok"):
						if (value.value == "False"):
							if em_state <= EMState.EM_FREE or self.em_state > EMState.EM_FALL:
								sound_output = self.sound_em_fall_issued
								em_state = EMState.EM_FALL
					if (value.key == "fall_sensor_left_ok"):
						if (value.value == "False"):
							if em_state <= EMState.EM_FREE or em_state > EMState.EM_FALL:
								sound_output = self.sound_em_fall_issued
								em_state = EMState.EM_FALL
					if (value.key == "fall_sensor_right_ok"):
						if (value.value == "False"):
							if em_state <= EMState.EM_FREE or em_state > EMState.EM_FALL:
								sound_output = self.sound_em_fall_issued
								em_state = EMState.EM_FALL
					if (value.key == "fall_sensors_released"):
						if (value.value == "False"):
							if em_state <= EMState.EM_FREE or em_state > EMState.EM_FALL:
								sound_output = self.sound_em_fall_issued
								em_state = EMState.EM_FALL
					if (value.key == "efi_bus_front_io_error"):
						if (value.value == "True"):
							if em_state <= EMState.EM_FREE or em_state > EMState.EM_INTERNAL:
								sound_output = self.sound_em_internal_issued
								em_state = EMState.EM_INTERNAL
					if (value.key == "efi_bus_left_io_error"):
						if (value.value == "True"):
							if em_state <= EMState.EM_FREE or em_state > EMState.EM_INTERNAL:
								sound_output = self.sound_em_internal_issued
								em_state = EMState.EM_INTERNAL
					if (value.key == "efi_bus_right_io_error"):
						if (value.value == "True"):
							if em_state <= EMState.EM_FREE or em_state > EMState.EM_INTERNAL:
								sound_output = self.sound_em_internal_issued
								em_state = EMState.EM_INTERNAL

		rospy.logdebug("self.em_state: %d, em_state: %d"%(self.em_state, em_state))
		if self.em_state != em_state and em_state > EMState.EM_FREE:
			#emergency_stop_monitoring has higher priority
			self.set_light(self.color_error)
			self.say(sound_output)
		self.em_state = em_state
		rospy.logdebug("self.em_state: %d"%(self.em_state))

	## Diagnostics monitoring
	def diagnostics_callback(self, msg):
		if self.em_state > EMState.EM_FREE:
			#emergency_stop_monitoring has higher priority
			return

		if self.diag_status != msg.level:
			self.diag_status = msg.level
			rospy.loginfo("Diagnostics change to "+ str(self.diag_status))

			if msg.level == DiagnosticStatus.OK:
				self.stop_light()
				self.motion_status = -1
			else:		# warning or error
				self.set_light(self.color_warn)

	## Motion Monitoring
	def jointstate_callback(self, msg):
		if self.em_state > EMState.EM_FREE:
			#emergency_stop_monitoring has higher priority
			return
		if self.diag_status != DiagnosticStatus.OK:
			#diagnostics_monitoring has higher priority
			return

		threshold = 0.1
		moving = 0
		for v in msg.velocity:
			if abs(v) > threshold:
				moving = 1
				break

		if self.motion_status != moving:
			self.motion_status = moving
			rospy.loginfo("Motion change to "+ str(self.motion_status))

			if moving == 0:	# not moving
				self.stop_light()
			else:	        # moving
				self.set_light(self.color_warn, True)

	## Topic Monitoring
	def emergency_stop_callback(self, msg):
		if msg.emergency_state == EmergencyStopState.EMSTOP:
			if not self.last_em_topic_state == msg.emergency_state:
				if msg.emergency_button_stop:
					self.em_state = EMState.EM_BUTTON
					rospy.logwarn("Emergency stop button has been issued!")
				elif msg.scanner_stop:
					self.em_state = EMState.EM_LASER
					rospy.logwarn("Scanner stop has been issued!")
				else:
					self.em_state = EMState.EM_UNKNOWN
				self.set_light(self.color_error)
				self.last_em_topic_state = msg.emergency_state

		if msg.emergency_state == EmergencyStopState.EMFREE:
			if not self.last_em_topic_state == msg.emergency_state:
				rospy.loginfo("Emergency stop released")
				self.em_state = EMState.EM_FREE
				self.stop_light()
				self.last_em_topic_state = msg.emergency_state


	## set light
	def set_light(self, color, flashing=False):
		if self.enable_light:
			for component in self.light_components:
				error_code, color_rgba = sss.compose_color(component, color)

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
					mode = LightMode()
					mode.priority = 10
					mode.colors = []
					mode.colors.append(color_rgba)
					if flashing:
						mode.mode = LightModes.FLASH	#Flashing
						mode.frequency = 2.0			#Hz
					else:
						mode.mode = LightModes.GLOW		#Glow
						mode.frequency = 10.0			#Hz

					goal = SetLightModeGoal()
					goal.mode = mode
					client.send_goal(goal)
					client.wait_for_result()
					result = client.get_result()
					self.track_id_light = result.track_id

				self.color = color
	def stop_light(self):
		if self.enable_light:
			if self.track_id_light is not None:
				for component in self.light_components:
					srv_server_name = component + "/stop_mode"
					try:
						rospy.wait_for_service(srv_server_name, timeout=2)
						srv_proxy = rospy.ServiceProxy(srv_server_name, StopLightMode)
						req = StopLightModeRequest()
						req.track_id = self.track_id_light
						srv_proxy(req)
					except Exception as e:
						rospy.logerr("%s service failed: %s",srv_server_name, e)

	def say(self, text):
		if self.enable_sound and text:
			for component in self.sound_components:
				sss.say(component, [text])

if __name__ == "__main__":
	rospy.init_node("emergency_stop_monitor")
	emergency_stop_monitor()
	rospy.spin()

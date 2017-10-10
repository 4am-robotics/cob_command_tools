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

import rospy
from sensor_msgs.msg import JointState
from diagnostic_msgs.msg import DiagnosticStatus

from cob_msgs.msg import *
from cob_light.msg import LightMode, LightModes, SetLightModeGoal, SetLightModeAction
from cob_light.srv import StopLightMode, StopLightModeRequest

from simple_script_server import *
sss = simple_script_server()


class emergency_stop_monitor():
	def __init__(self):
		self.color = "None"
		self.enable_sound = rospy.get_param("~enable_sound", False)
		self.enable_light = rospy.get_param("~enable_light", False)
		self.diagnostics_based = rospy.get_param("~diagnostics_based", False)
		self.motion_based = rospy.get_param("~motion_based", False)
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

		self.sound_em_released = rospy.get_param("~sound_em_released", "emergency stop released")
		self.sound_em_acknowledged = rospy.get_param("~sound_em_acknowledged", "emergency stop acknowledged")
		self.sound_em_laser_issued = rospy.get_param("~sound_em_laser_issued", "laser emergency stop issued")
		self.sound_em_button_issued = rospy.get_param("~sound_em_button_issued", "emergency stop button pressed")
		self.sound_em_stop_issued = rospy.get_param("~sound_em_stop_issued", "emergency stop issued")
		self.sound_em_unknown_issued = rospy.get_param("~sound_em_unknown_issued", "unknown emergency status issued")

		#emergency_stop_monitoring always enabled
		rospy.Subscriber("/emergency_stop_state", EmergencyStopState, self.emergency_callback, queue_size=1)
		self.em_status = -1
		self.first_time = True

		if(self.diagnostics_based):
			rospy.Subscriber("/diagnostics_toplevel_state", DiagnosticStatus, self.diagnostics_callback, queue_size=1)
			self.diag_status = -1
			self.last_diag = rospy.get_rostime()

		if(self.motion_based):
			rospy.Subscriber("/joint_states", JointState, self.jointstate_callback, queue_size=1)
			self.motion_status = -1
			self.last_vel = rospy.get_rostime()


	## EmergencyStop monitoring
	def emergency_callback(self, msg):
		# skip first message to avoid speach output on startup
		if self.first_time:
			self.first_time = False
			self.em_status = msg.emergency_state
			return

		if self.em_status != msg.emergency_state:
			self.em_status = msg.emergency_state
			rospy.loginfo("Emergency change to "+ str(self.em_status))

			if msg.emergency_state == 0: # ready
				self.stop_light()
				self.say(self.sound_em_released)
				self.diag_status = -1
				self.motion_status = -1
			elif msg.emergency_state == 1: # em stop
				self.set_light(self.color_error)
				if msg.scanner_stop and not msg.emergency_button_stop:
					self.say(self.sound_em_laser_issued)
				elif not msg.scanner_stop and msg.emergency_button_stop:
					self.say(self.sound_em_button_issued)
				else:
					self.say(self.sound_em_stop_issued)
			elif msg.emergency_state == 2: # release
				self.set_light(self.color_warn)
				self.say(self.sound_em_acknowledged)
			else:
				rospy.logerr("Unknown emergency status issued: %s",str(msg.emergency_state))
				self.set_light(self.color_error)
				self.say(self.sound_em_unknown_issued)


	## Diagnostics monitoring
	def diagnostics_callback(self, msg):
		if self.em_status != 0:
			#emergency_stop_monitoring has higher priority
			return

		if self.diag_status != msg.level:
			self.diag_status = msg.level
			rospy.loginfo("Diagnostics change to "+ str(self.diag_status))

			if msg.level == 0:	# ok
				self.stop_light()
				self.motion_status = -1
			else:								# warning or error
				self.set_light(self.color_warn)


	## Motion Monitoring
	def jointstate_callback(self, msg):
		if self.em_status != 0:
			#emergency_stop_monitoring has higher priority
			return
		if self.diag_status != 0:
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
			else:						# moving
				self.set_light(self.color_warn, True)


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
		if self.enable_sound:
			for component in self.sound_components:
				sss.say(component, [text])

if __name__ == "__main__":
	rospy.init_node("emergency_stop_monitor")
	emergency_stop_monitor()
	rospy.spin()

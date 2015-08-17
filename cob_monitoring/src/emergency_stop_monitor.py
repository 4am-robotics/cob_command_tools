#!/usr/bin/python
import sys

import rospy
from sensor_msgs.msg import JointState
from diagnostic_msgs.msg import DiagnosticStatus

from cob_msgs.msg import *
from cob_light.msg import LightMode, SetLightModeGoal, SetLightModeAction

from simple_script_server import *
sss = simple_script_server()


class emergency_stop_monitor():
	def __init__(self):
		self.color = "None"
		self.sound_enabled = rospy.get_param("~sound_enabled", True)
		self.led_enabled = rospy.get_param("~led_enabled", True)
		self.diagnostics_based = rospy.get_param("~diagnostics_based", False)
		self.motion_based = rospy.get_param("~motion_based", False)

		if(self.led_enabled):
			if not rospy.has_param("~led_components"):
				rospy.logwarn("parameter led_components does not exist on ROS Parameter Server, aborting...")
				sys.exit(1)
			self.light_components = rospy.get_param("~led_components")
			self.color_error = rospy.get_param("~color_error","red")
			self.color_warn = rospy.get_param("~color_warn","yellow")
			self.color_ok = rospy.get_param("~color_ok","green")
			self.color_off = rospy.get_param("~color_off","black")

		#emergency_stop_monitoring always enabled
		rospy.Subscriber("/emergency_stop_state", EmergencyStopState, self.emergency_callback)
		self.em_status = -1
		self.first_time = True

		if(self.diagnostics_based):
			rospy.Subscriber("/diagnostics_toplevel_state", DiagnosticStatus, self.diagnostics_callback)
			self.diag_status = -1
			self.last_diag = rospy.get_rostime()

		if(self.motion_based):
			rospy.Subscriber("/joint_states", JointState, self.jointstate_callback)
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
				self.set_light(self.color_ok)
				if(self.sound_enabled):
					sss.say(["emergency stop released"])
				self.diag_status = -1
				self.motion_status = -1
			elif msg.emergency_state == 1: # em stop
				self.set_light(self.color_error)
				if msg.scanner_stop and not msg.emergency_button_stop:
					if(self.sound_enabled):
						sss.say(["laser emergency stop issued"])
				elif not msg.scanner_stop and msg.emergency_button_stop:
					if(self.sound_enabled):
						sss.say(["emergency stop button pressed"])
				else:
					if(self.sound_enabled):
						sss.say(["emergency stop issued"])
			elif msg.emergency_state == 2: # release
				self.set_light(self.color_warn)
				if(self.sound_enabled):
					sss.say(["emergency stop acknowledged"])
			else:
				rospy.logerr("Unknown emergency status issued: %s",str(msg.emergency_state))
				self.set_light(self.color_error)
				if(self.sound_enabled):
					sss.say(["Unknown emergency status issued"])


	## Diagnostics monitoring
	def diagnostics_callback(self, msg):
		if self.em_status != 0:
			#emergency_stop_monitoring has higher priority
			return

		if self.diag_status != msg.level:
			self.diag_status = msg.level
			rospy.loginfo("Diagnostics change to "+ str(self.diag_status))

			if msg.level == 0:	# ok
				self.set_light(self.color_ok)
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
				self.set_light(self.color_ok)
			else:						# moving
				self.set_light(self.color_warn, True)


	## set light
	def set_light(self, color, flashing=False):
		for component in self.light_components:
			color_rgba = sss.compose_color(component, color)

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
				mode.color = color_rgba
				if flashing:
					mode.mode = 2		#Flashing
					mode.frequency = 2.0	#Hz
				else:
					mode.mode = 1		#Static

				goal = SetLightModeGoal()
				goal.mode = mode
				client.send_goal(goal)
				client.wait_for_result()

				self.color = color


if __name__ == "__main__":
	rospy.init_node("emergency_stop_monitor")
	emergency_stop_monitor()
	rospy.spin()

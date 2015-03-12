#!/usr/bin/python

import roslib
roslib.load_manifest('cob_monitoring')
import rospy
import sys

from geometry_msgs.msg import Twist
from diagnostic_msgs.msg import DiagnosticArray

from cob_msgs.msg import *

from simple_script_server import *
sss = simple_script_server()

##################
### TODO: add diagnostics for em_stop (probably better to be implemented in relayboard) --> then create a diagnostics_monitor.py with sets leds and sound from diagnostics information (for arm, base, torso, ...)
### which color and flashing code assign to diagnostics?
##################

class emergency_stop_monitor():
	def __init__(self):
		self.color = "None"
		self.sound_enabled = rospy.get_param("~sound_enabled", True)
		self.led_enabled = rospy.get_param("~led_enabled", True)

		if(self.led_enabled):
			if not rospy.has_param("~led_components"):
				rospy.logwarn("parameter led_components does not exist on ROS Parameter Server, aborting...")
				sys.exit(1)
			self.light_components = rospy.get_param("~led_components")
			self.color_error = rospy.get_param("~color_error","red")
			self.color_warn = rospy.get_param("~color_warn","yellow")
			self.color_ok = rospy.get_param("~color_ok","green")
			self.color_off = rospy.get_param("~color_off","black")

			self.diagnotics_enabled = rospy.get_param("~diagnostics_based", False)
			if(self.diagnotics_enabled):
				rospy.Subscriber("/diagnostics", DiagnosticArray, self.new_diagnostics)
				self.on = False
        			self.diag_err = False
				self.last_led = rospy.get_rostime()
			else:
				rospy.Subscriber("/emergency_stop_state", EmergencyStopState, self.emergency_callback)	
				self.em_status = EmergencyStopState()
				self.first_time = True

			self.motion_sensing = rospy.get_param("~motion_sensing", False)
			if(self.motion_sensing):
				rospy.Subscriber("/base_controller/command_direct", Twist, self.new_velcommand)
				self.last_vel = rospy.get_rostime()

	## Diagnostics monitoring
	def new_diagnostics(self, diag):
        	for status in diag.status:
            		if(status.name == "//base_controller"):
                		if(status.level != 0):## && self.last_base_diag == 0):
                    			self.diag_err = True
                		elif(status.level == 0):## && self.last_base_diag == 1):
                    			self.diag_err = False
		if((rospy.get_rostime() - self.last_led).to_sec() > 0.5):
			self.last_led = rospy.get_rostime()
		        #Trigger LEDS
	    		if(self.diag_err):
				if(self.color != self.color_error):
		    			self.set_light(self.color_error)	
					self.color = self.color_error
	    		else:
        			if ((rospy.get_rostime() - self.last_vel).to_sec() > 1.0):
					if(self.color != self.color_ok):
		            			self.set_light(self.color_ok)
						self.color = self.color_ok
	    	    		else:
        		    		if(self.on):
            		    			self.on = False
						if(self.color != self.color_warn):
	            	    				self.set_light(self.color_warn)
							self.color = self.color_warn
	 		           	else:
        	        			self.on = True
						if(self.color != self.color_off):
				                	self.set_light(self.color_off)
							self.color = self.color_off
		

	## Velocity Monitoring
	def new_velcommand(self, twist):
	        if twist.linear.x != 0 or twist.linear.y != 0 or twist.angular.z != 0:
        		self.last_vel = rospy.get_rostime()

	## Emergency stop monitoring
	def emergency_callback(self,msg):
		# skip first message to avoid speach output on startup
		if self.first_time:
			self.first_time = False
			self.em_status = msg
			return
	
		if self.em_status.emergency_state != msg.emergency_state:
			self.em_status = msg
			rospy.loginfo("Emergency change to "+ str(self.em_status.emergency_state))
		
			if self.em_status.emergency_state == 0: # ready
				self.set_light(self.color_ok)
			elif self.em_status.emergency_state == 1: # em stop
				self.set_light(self.color_error)
				if self.em_status.scanner_stop and not self.em_status.emergency_button_stop:
					if(self.sound_enabled):
						sss.say(["laser emergency stop issued"])
				elif not self.em_status.scanner_stop and self.em_status.emergency_button_stop:
					if(self.sound_enabled):
						sss.say(["emergency stop button pressed"])
				else:
					if(self.sound_enabled):
						sss.say(["emergency stop issued"])
			elif self.em_status.emergency_state == 2: # release
				self.set_light(self.color_warn)
				if(self.sound_enabled):
					sss.say(["emergency stop released"])

	## set light
	def set_light(self,color):
		for component in self.light_components:
			sss.set_light(component,color)

if __name__ == "__main__":
	rospy.init_node("emergency_stop_monitor")
	emergency_stop_monitor()
	rospy.spin()

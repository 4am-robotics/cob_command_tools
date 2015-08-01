#!/usr/bin/python
#################################################################
##\file
#
# \note
#	 Copyright (c) 2010 \n
#	 Fraunhofer Institute for Manufacturing Engineering
#	 and Automation (IPA) \n\n
#
#################################################################
#
# \note
#	 Project name: care-o-bot
# \note
#	 ROS stack name: cob_command_tools
# \note
#	 ROS package name: cob_script_server
#
# \author
#	 Author: Florian Weisshardt, email:florian.weisshardt@ipa.fhg.de
# \author
#	 Supervised by: Florian Weisshardt, email:florian.weisshardt@ipa.fhg.de
#
# \date Date of creation: Aug 2010
#
# \brief
#	 Implements script server functionalities.
#
#################################################################
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#		 - Redistributions of source code must retain the above copyright
#			 notice, this list of conditions and the following disclaimer. \n
#		 - Redistributions in binary form must reproduce the above copyright
#			 notice, this list of conditions and the following disclaimer in the
#			 documentation and/or other materials provided with the distribution. \n
#		 - Neither the name of the Fraunhofer Institute for Manufacturing
#			 Engineering and Automation (IPA) nor the names of its
#			 contributors may be used to endorse or promote products derived from
#			 this software without specific prior written permission. \n
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

import time
import os
import sys
import types
import thread
import commands
import math
import threading

# graph includes
import pygraphviz as pgv

# ROS imports
import rospy
import actionlib

# msg imports
from std_msgs.msg import String,ColorRGBA
from std_srvs.srv import Trigger
from geometry_msgs.msg import *
from trajectory_msgs.msg import *
from move_base_msgs.msg import *
from control_msgs.msg import *
from tf.transformations import *

# care-o-bot includes
from cob_sound.msg import *
from cob_script_server.msg import *
from cob_light.msg import LightMode, SetLightModeGoal, SetLightModeAction
from cob_mimic.msg import SetMimicGoal, SetMimicAction

graph=""
graph_wait_list=[]
function_counter = 0
ah_counter = 0
graph = pgv.AGraph()
graph.node_attr['shape']='box'
last_node = "Start"

## Script class from which all script inherit.
#
# Implements basic functionalities for all scripts.
class script():
	def __init__(self):
		# use filename as nodename
		filename = os.path.basename(sys.argv[0])
		self.basename, extension = os.path.splitext(filename)
		rospy.init_node(self.basename)
		self.graph_pub = rospy.Publisher("/script_server/graph", String, queue_size=1)

	## Dummy function for initialization
	def Initialize(self):
		pass

	## Dummy function for main run function
	def Run(self):
		pass

	## Function to start the script
	#
	# First does a simulated turn and then calls Initialize() and Run().
	def Start(self):
		self.Parse()
		global ah_counter
		ah_counter = 0
		self.sss = simple_script_server()
		rospy.loginfo("Starting <<%s>> script...",self.basename)
		self.Initialize()
		self.Run()
		# wait until last threaded action finishes
		rospy.loginfo("Wait for script to finish...")
		while ah_counter != 0:
			rospy.sleep(1)
		rospy.loginfo("...script finished.")
	
	## Function to generate graph view of script.
	#
	# Starts the script in simulation mode and calls Initialize() and Run().
	def Parse(self):
		rospy.loginfo("Start parsing...")
		global graph
		global function_counter
		function_counter = 0
		# run script in simulation mode
		self.sss = simple_script_server(parse=True)
		self.Initialize()
		self.Run()
		
		# save graph on parameter server for further processing
#		self.graph = graph
		rospy.set_param("/script_server/graph", graph.string())
		self.graph_pub.publish(graph.string())
		rospy.loginfo("...parsing finished")
		function_counter = 0
		return graph.string()

## Simple script server class.
#
# Implements the python interface for the script server.
class simple_script_server:
	## Initializes simple_script_server class.
	#
	# \param parse Defines wether to run script in simulation for graph generation or not
	def __init__(self, parse=False):
		global graph
		self.ns_global_prefix = "/script_server"
		self.wav_path = ""
		self.parse = parse
		
		rospy.sleep(1) # we have to wait here until publishers are ready, don't ask why

	#------------------- Init section -------------------#
	## Initializes different components.
	#
	# Based on the component, the corresponding init service will be called.
	#
	# \param component_name Name of the component.
	def init(self,component_name,blocking=True):
		return self.trigger(component_name,"init",blocking=blocking)

	## Stops different components.
	#
	# Based on the component, the corresponding stop service will be called.
	#
	# \param component_name Name of the component.
	def stop(self,component_name,mode="omni",blocking=True):
		ah = action_handle("stop", component_name, "", False, self.parse)
		if(self.parse):
			return ah
		else:
			ah.set_active(mode="service")

		rospy.loginfo("<<stop>> <<%s>>", component_name)
		if component_name == "base":
			if(mode == None or mode == ""):
				action_server_name = "/move_base"
			elif(mode == "omni"):
				action_server_name = "/move_base"
			elif(mode == "diff"):
				action_server_name = "/move_base_diff"
			elif(mode == "linear"):
				action_server_name = "/move_base_linear"
			else:
				rospy.logerr("no valid navigation mode given for %s, aborting...",component_name)
				print "navigation mode is:",mode
				ah.set_failed(33)
				return ah
			client = actionlib.SimpleActionClient(action_server_name, MoveBaseAction)
		else:
			parameter_name = self.ns_global_prefix + "/" + component_name + "/action_name"
			if not rospy.has_param(parameter_name):
					rospy.logerr("parameter %s does not exist on ROS Parameter Server, aborting...",parameter_name)
					return 2
			action_server_name = rospy.get_param(parameter_name)
			client = actionlib.SimpleActionClient(action_server_name, FollowJointTrajectoryAction)

		# call action server
		rospy.logdebug("calling %s action server",action_server_name)
		
		if blocking:
			# trying to connect to server
			rospy.logdebug("waiting for %s action server to start",action_server_name)
			if not client.wait_for_server(rospy.Duration(5)):
				# error: server did not respond
				rospy.logerr("%s action server not ready within timeout, aborting...", action_server_name)
				ah.set_failed(4)
				return ah
			else:
				rospy.logdebug("%s action server ready",action_server_name)

		# cancel all goals
		client.cancel_all_goals()
		
		ah.set_succeeded() # full success
		return ah	

	## Recovers different components.
	#
	# Based on the component, the corresponding recover service will be called.
	#
	# \param component_name Name of the component.
	def recover(self,component_name,blocking=True):
		return self.trigger(component_name,"recover",blocking=blocking)

	## Halts different components.
	#
	# Based on the component, the corresponding halt service will be called.
	#
	# \param component_name Name of the component.
	def halt(self,component_name,blocking=True):
		return self.trigger(component_name,"halt",blocking=blocking)

	## Deals with all kind of trigger services for different components.
	#
	# Based on the component and service name, the corresponding trigger service will be called.
	#
	# \param component_name Name of the component.
	# \param service_name Name of the trigger service.
	# \param blocking Service calls are always blocking. The parameter is only provided for compatibility with other functions.
	def trigger(self,component_name,service_name,blocking=True, planning=False):
		ah = action_handle(service_name, component_name, "", blocking, self.parse)
		if(self.parse):
			return ah
		else:
			ah.set_active(mode="service")

		rospy.loginfo("<<%s>> <<%s>>", service_name, component_name)
		parameter_name = self.ns_global_prefix + "/" + component_name + "/service_ns"
		if not rospy.has_param(parameter_name):
				rospy.logerr("parameter %s does not exist on ROS Parameter Server, aborting...",parameter_name)
				return 2
		service_ns_name = rospy.get_param(parameter_name)
		service_full_name = service_ns_name + "/" + service_name
		
		if blocking:
			# check if service is available
			try:
				rospy.wait_for_service(service_full_name,5)
			except rospy.ROSException, e:
				error_message = "%s"%e
				rospy.logerr("...<<%s>> service of <<%s>> not available, error: %s",service_name, component_name, error_message)
				ah.set_failed(4)
				return ah

		# check if service is callable
		try:
			trigger = rospy.ServiceProxy(service_full_name,Trigger)
			if blocking:
				rospy.loginfo("Wait for <<%s>> to <<%s>>...", component_name, service_name)
				resp = trigger()
			else:
				thread.start_new_thread(trigger,())
		except rospy.ServiceException, e:
			error_message = "%s"%e
			rospy.logerr("...calling <<%s>> service of <<%s>> not successfull, error: %s",service_name, component_name, error_message)
			ah.set_failed(10)
			return ah

		if blocking:
			# evaluate sevice response
			if not resp.success:
				rospy.logerr("...<<%s>> <<%s>> not successfull, error: %s",service_name, component_name, resp.message) 
				ah.set_failed(10)
				return ah
		
			# full success
			rospy.loginfo("...<<%s>> is <<%s>>", component_name, service_name)
			ah.set_succeeded() # full success
		return ah

#------------------- Move section -------------------#
	## Deals with all kind of movements for different components.
	#
	# Based on the component, the corresponding move functions will be called.
	#
	# \param component_name Name of the component.
	# \param parameter_name Name of the parameter on the ROS parameter server.
	# \param blocking Bool value to specify blocking behaviour.
	def move(self,component_name,parameter_name,blocking=True, mode=None):
		if component_name == "base":
			return self.move_base(component_name,parameter_name,blocking, mode)
		elif component_name == "arm" and mode=="planned":
			return self.move_planned(component_name,parameter_name,blocking)
		else:
			return self.move_traj(component_name,parameter_name,blocking)

	## Deals with movements of the base.
	#
	# A target will be sent to the actionlib interface of the move_base node.
	#
	# \param component_name Name of the component.
	# \param parameter_name Name of the parameter on the ROS parameter server.
	# \param blocking Bool value to specify blocking behaviour.
	def move_base(self,component_name,parameter_name,blocking, mode):
		ah = action_handle("move", component_name, parameter_name, blocking, self.parse)
		if(self.parse):
			return ah
		else:
			ah.set_active()
		
		if(mode == None or mode == ""):
			rospy.loginfo("Move <<%s>> to <<%s>>",component_name,parameter_name)
		else:
			rospy.loginfo("Move <<%s>> to <<%s>> using <<%s>> mode",component_name,parameter_name,mode)
		
		# get joint values from parameter server
		if type(parameter_name) is str:
			if not rospy.has_param(self.ns_global_prefix + "/" + component_name + "/" + parameter_name):
				rospy.logerr("parameter %s does not exist on ROS Parameter Server, aborting...",self.ns_global_prefix + "/" + component_name + "/" + parameter_name)
				ah.set_failed(2)
				return ah
			param = rospy.get_param(self.ns_global_prefix + "/" + component_name + "/" + parameter_name)
		else:
			param = parameter_name
		
		# check trajectory parameters
		if not type(param) is list: # check outer list
				rospy.logerr("no valid parameter for %s: not a list, aborting...",component_name)
				print "parameter is:",param
				ah.set_failed(3)
				return ah
		else:
			#print i,"type1 = ", type(i)
			DOF = 3
			if not len(param) == DOF: # check dimension
				rospy.logerr("no valid parameter for %s: dimension should be %d and is %d, aborting...",component_name,DOF,len(param))
				print "parameter is:",param
				ah.set_failed(3)
				return ah
			else:
				for i in param:
					#print i,"type2 = ", type(i)
					if not ((type(i) is float) or (type(i) is int)): # check type
						#print type(i)
						rospy.logerr("no valid parameter for %s: not a list of float or int, aborting...",component_name)
						print "parameter is:",param
						ah.set_failed(3)
						return ah
					else:
						rospy.logdebug("accepted parameter %f for %s",i,component_name)

		# convert to pose message
		pose = PoseStamped()
		pose.header.stamp = rospy.Time.now()
		pose.header.frame_id = "/map"
		pose.pose.position.x = param[0]
		pose.pose.position.y = param[1]
		pose.pose.position.z = 0.0
		q = quaternion_from_euler(0, 0, param[2])
		pose.pose.orientation.x = q[0]
		pose.pose.orientation.y = q[1]
		pose.pose.orientation.z = q[2]
		pose.pose.orientation.w = q[3]
		
		# call action server
		if(mode == None or mode == ""):
			action_server_name = "/move_base"
		elif(mode == "omni"):
			action_server_name = "/move_base"
		elif(mode == "diff"):
			action_server_name = "/move_base_diff"
		elif(mode == "linear"):
			action_server_name = "/move_base_linear"
		else:
			rospy.logerr("no valid navigation mode given for %s, aborting...",component_name)
			print "navigation mode is:",mode
			ah.set_failed(33)
			return ah
		
		rospy.logdebug("calling %s action server",action_server_name)
		client = actionlib.SimpleActionClient(action_server_name, MoveBaseAction)
		# trying to connect to server
		rospy.logdebug("waiting for %s action server to start",action_server_name)
		if not client.wait_for_server(rospy.Duration(5)):
			# error: server did not respond
			rospy.logerr("%s action server not ready within timeout, aborting...", action_server_name)
			ah.set_failed(4)
			return ah
		else:
			rospy.logdebug("%s action server ready",action_server_name)

		# sending goal
		client_goal = MoveBaseGoal()
		client_goal.target_pose = pose
		#print client_goal
		client.send_goal(client_goal)
		ah.set_client(client)

		ah.wait_inside()

		return ah

	## Parse and compose trajectory message
	def compose_trajectory(self, component_name, parameter_name):
		# get joint_names from parameter server
		param_string = self.ns_global_prefix + "/" + component_name + "/joint_names"
		if not rospy.has_param(param_string):
				rospy.logerr("parameter %s does not exist on ROS Parameter Server, aborting...",param_string)
				return (JointTrajectory(), 2)
		joint_names = rospy.get_param(param_string)
		
		# check joint_names parameter
		if not type(joint_names) is list: # check list
				rospy.logerr("no valid joint_names for %s: not a list, aborting...",component_name)
				print "joint_names are:",joint_names
				return (JointTrajectory(), 3)
		else:
			for i in joint_names:
				#print i,"type1 = ", type(i)
				if not type(i) is str: # check string
					rospy.logerr("no valid joint_names for %s: not a list of strings, aborting...",component_name)
					print "joint_names are:",param
					return (JointTrajectory(), 3)
				else:
					rospy.logdebug("accepted joint_names for component %s",component_name)
		
		# get joint values from parameter server
		if type(parameter_name) is str:
			if not rospy.has_param(self.ns_global_prefix + "/" + component_name + "/" + parameter_name):
				rospy.logerr("parameter %s does not exist on ROS Parameter Server, aborting...",self.ns_global_prefix + "/" + component_name + "/" + parameter_name)
				return (JointTrajectory(), 2)
			param = rospy.get_param(self.ns_global_prefix + "/" + component_name + "/" + parameter_name)
		else:
			param = parameter_name

		# check trajectory parameters
		if not type(param) is list: # check outer list
				rospy.logerr("no valid parameter for %s: not a list, aborting...",component_name)
				print "parameter is:",param
				return (JointTrajectory(), 3)

		traj = []

		for point in param:
			#print point,"type1 = ", type(point)
			if type(point) is str:
				if not rospy.has_param(self.ns_global_prefix + "/" + component_name + "/" + point):
					rospy.logerr("parameter %s does not exist on ROS Parameter Server, aborting...",self.ns_global_prefix + "/" + component_name + "/" + point)
					return (JointTrajectory(), 2)
				point = rospy.get_param(self.ns_global_prefix + "/" + component_name + "/" + point)
				point = point[0] # \todo TODO: hack because only first point is used, no support for trajectories inside trajectories
				#print point
			elif type(point) is list:
				rospy.logdebug("point is a list")
			else:
				rospy.logerr("no valid parameter for %s: not a list of lists or strings, aborting...",component_name)
				print "parameter is:",param
				return (JointTrajectory(), 3)

			# here: point should be list of floats/ints
			#print point
			if not len(point) == len(joint_names): # check dimension
				rospy.logerr("no valid parameter for %s: dimension should be %d and is %d, aborting...",component_name,len(joint_names),len(point))
				print "parameter is:",param
				return (JointTrajectory(), 3)

			for value in point:
				#print value,"type2 = ", type(value)
				if not ((type(value) is float) or (type(value) is int)): # check type
					#print type(value)
					rospy.logerr("no valid parameter for %s: not a list of float or int, aborting...",component_name)
					print "parameter is:",param
					return (JointTrajectory(), 3)
			
				rospy.logdebug("accepted value %f for %s",value,component_name)
			traj.append(point)

		rospy.logdebug("accepted trajectory for %s",component_name)
		
		# get current pos
		timeout = 3.0
		try:
			start_pos = rospy.wait_for_message("/" + component_name + "/joint_trajectory_controller/state", JointTrajectoryControllerState, timeout = timeout).actual.positions
		except rospy.ROSException as e:
			rospy.logwarn("no joint states received from %s within timeout of %ssec. using default point time of 8sec.", component_name, str(timeout))
			start_pos = []

		# convert to ROS trajectory message
		traj_msg = JointTrajectory()
		# if no timestamp is set in header, this means that the trajectory starts "now"
		traj_msg.joint_names = joint_names
		point_nr = 0
		traj_time = 0
		
		param_string = self.ns_global_prefix + "/" + component_name + "/default_vel"
		if not rospy.has_param(param_string):
			rospy.logwarn("parameter %s does not exist on ROS Parameter Server, using default of 0.1 [rad/sec].",param_string)
			default_vel = 0.1 # rad/s
		else:
			default_vel = rospy.get_param(param_string)

		for point in traj:
			point_nr = point_nr + 1
			point_msg = JointTrajectoryPoint()
			point_msg.positions = point

			# set zero velocities for last trajectory point only
			if point_nr == len(traj):
				point_msg.velocities = [0]*len(joint_names)

			# use hardcoded point_time if no start_pos available
			if start_pos != []:
				point_time = self.calculate_point_time(component_name, start_pos, point, default_vel)
			else:
				point_time = 8*point_nr

			start_pos = point
			point_msg.time_from_start=rospy.Duration(point_time + traj_time)
			traj_time += point_time
			traj_msg.points.append(point_msg)
		return (traj_msg, 0)

	def calculate_point_time(self, component_name, start_pos, end_pos, default_vel):
		d_max = max(list(abs(numpy.array(start_pos) - numpy.array(end_pos))))
		point_time = d_max / default_vel
		return point_time

	## Deals with all kind of trajectory movements for different components.
	#
	# A trajectory will be sent to the actionlib interface of the corresponding component.
	#
	# \param component_name Name of the component.
	# \param parameter_name Name of the parameter on the ROS parameter server.
	# \param blocking Bool value to specify blocking behaviour.
	def move_traj(self,component_name,parameter_name,blocking):
		ah = action_handle("move", component_name, parameter_name, blocking, self.parse)
		if(self.parse):
			return ah
		else:
			ah.set_active()
		
		rospy.loginfo("Move <<%s>> to <<%s>>",component_name,parameter_name)
		(traj_msg, error_code) = self.compose_trajectory(component_name, parameter_name)
		if error_code != 0:
			ah.set_failed(error_code)
			return ah
		

		# call action server
		parameter_name = self.ns_global_prefix + "/" + component_name + "/action_name"
		if not rospy.has_param(parameter_name):
				rospy.logerr("parameter %s does not exist on ROS Parameter Server, aborting...",parameter_name)
				return 2
		action_server_name = rospy.get_param(parameter_name)
		rospy.logdebug("calling %s action server",action_server_name)
		client = actionlib.SimpleActionClient(action_server_name, FollowJointTrajectoryAction)
		# trying to connect to server
		rospy.logdebug("waiting for %s action server to start",action_server_name)
		if not client.wait_for_server(rospy.Duration(5)):
			# error: server did not respond
			rospy.logerr("%s action server not ready within timeout, aborting...", action_server_name)
			ah.set_failed(4)
			return ah
		else:
			rospy.logdebug("%s action server ready",action_server_name)		

		# sending goal
		client_goal = FollowJointTrajectoryGoal()
		client_goal.trajectory = traj_msg
		#print client_goal
		client.send_goal(client_goal)
		ah.set_client(client)

		ah.wait_inside()
		return ah
		
	## Relative movement of the base
	#
	# \param component_name Name of component; here always "base".
	# \param parameter_name List of length 3: (item 1 & 2) relative x and y translation [m]; (item 3) relative rotation about z axis [rad].
	# \param blocking Bool value to specify blocking behaviour.
	# 
	# # throws error code 3 in case of invalid parameter_name vector 
	def move_base_rel(self, component_name, parameter_name=[0,0,0], blocking=True):	
		ah = action_handle("move_base_rel", component_name, parameter_name, blocking, self.parse)
		if(self.parse):
			return ah
		else:
			ah.set_active(mode="topic")
			
		parameter_topic_name = self.ns_global_prefix + "/" + component_name + "/topic_name"
		if not rospy.has_param(parameter_topic_name):
			rospy.logerr("parameter %s does not exist on ROS Parameter Server, aborting...",parameter_topic_name)
			ah.set_failed(3)
			return ah
		topic_name = rospy.get_param(parameter_topic_name)

		rospy.loginfo("Move base relatively by <<%s>>", parameter_name)

		# step 0: check validity of parameters:
		if not len(parameter_name) == 3 or not isinstance(parameter_name[0], (int, float)) or not isinstance(parameter_name[1], (int, float)) or not isinstance(parameter_name[2], (int, float)):
			rospy.logerr("Non-numeric parameter_name list, aborting move_base_rel")
			print("parameter_name must be numeric list of length 3; (relative x and y transl [m], relative rotation [rad])")
			ah.set_failed(3)
			return ah
		if math.sqrt(parameter_name[0]**2 + parameter_name[1]**2) >= 1.0: # [m]
			rospy.logerr("Maximal relative translation step exceeded, aborting move_base_rel")
			print("Maximal relative translation step is 0.1 m")
			ah.set_failed(3)
			return(ah)
		if abs(parameter_name[2]) >= math.pi/2: # [rad]
			rospy.logerr("Maximal relative rotation step exceeded, aborting move_base_rel")
			print("Maximal relative rotation step is pi/2")
			ah.set_failed(3)
			return(ah)

		# step 1: determine duration of motion so that upper thresholds for both translational as well as rotational velocity are not exceeded
		max_trans_vel = 0.05 # [m/s]
		max_rot_vel = 0.2 # [rad/s]
		duration_trans_sec = math.sqrt(parameter_name[0]**2 + parameter_name[1]**2) / max_trans_vel
		duration_rot_sec = abs(parameter_name[2] / max_rot_vel)
		duration_sec = max(duration_trans_sec, duration_rot_sec)
		duration_ros = rospy.Duration.from_sec(duration_sec) # duration of motion in ROS time

		# step 2: determine actual velocities based on calculated duration
		x_vel = parameter_name[0] / duration_sec
		y_vel = parameter_name[1] / duration_sec
		rot_vel = parameter_name[2] / duration_sec

		# step 3: send constant velocity command to base_controller for the calculated duration of motion
		pub = rospy.Publisher(topic_name, Twist, queue_size=1)
		twist = Twist()
		twist.linear.x = x_vel
		twist.linear.y = y_vel
		twist.angular.z = rot_vel
		end_time = rospy.Time.now() + duration_ros
		
		if blocking:
			rospy.loginfo("Wait for <<%s>> to finish move_base_rel...", component_name)
			self.publish_twist(pub, twist, end_time)
		else:
			thread.start_new_thread(self.publish_twist,(pub, twist, end_time))

		ah.set_succeeded()
		return ah
		
	def publish_twist(self, pub, twist, end_time):
		r = rospy.Rate(10) # send velocity commands at 10 Hz
		while not rospy.is_shutdown() and rospy.Time.now() < end_time:
			pub.publish(twist)
			r.sleep()
		
#------------------- LED section -------------------#
	## Set the color of the cob_light component.
	#
	# The color is given by a parameter on the parameter server.
	#
	# \param parameter_name Name of the parameter on the parameter server which holds the rgb values.

	def compose_color(self,component_name,parameter_name):
		# get joint values from parameter server
		if type(parameter_name) is str:
			full_parameter_name = self.ns_global_prefix + "/" + component_name + "/" + parameter_name
			if not rospy.has_param(full_parameter_name):
				rospy.logerr("parameter %s does not exist on ROS Parameter Server, aborting...",full_parameter_name)
				return 2
			param = rospy.get_param(full_parameter_name)
		else:
			param = parameter_name
			
		# check color parameters
		if not type(param) is list: # check outer list
			rospy.logerr("no valid parameter for light: not a list, aborting...")
			print "parameter is:",param
			ah.error_code = 3
			return ah
		else:
			if not len(param) == 4: # check dimension
				rospy.logerr("no valid parameter for light: dimension should be 4 (r,g,b,a) and is %d, aborting...",len(param))
				print "parameter is:",param
				ah.error_code = 3
				return ah
			else:
				for i in param:
					#print i,"type1 = ", type(i)
					if not ((type(i) is float) or (type(i) is int)): # check type
						#print type(i)
						rospy.logerr("no valid parameter for light: not a list of float or int, aborting...")
						print "parameter is:",param
						ah.error_code = 3
						return ah
					else:
						rospy.logdebug("accepted parameter %f for light",i)
		
		# convert to ColorRGBA message
		color = ColorRGBA()
		color.r = param[0]
		color.g = param[1]
		color.b = param[2]
		color.a = param[3] # Transparency
		return color	

	def set_light(self,component_name,parameter_name,blocking=False):
		ah = action_handle("set_light", component_name, parameter_name, blocking, self.parse)
		if(self.parse):
			return ah
		else:
			ah.set_active()

		rospy.loginfo("Set <<%s>> to <<%s>>", component_name, parameter_name)

		mode = LightMode()
		mode.mode = 1
		mode.color = self.compose_color(component_name, parameter_name)

		# call action server
		action_server_name = component_name + "/set_light"
		rospy.logdebug("calling %s action server",action_server_name)
		client = actionlib.SimpleActionClient(action_server_name, SetLightModeAction)
		# trying to connect to server
		rospy.logdebug("waiting for %s action server to start",action_server_name)
		if not client.wait_for_server(rospy.Duration(5)):
			# error: server did not respond
			rospy.logerr("%s action server not ready within timeout, aborting...", action_server_name)
			ah.set_failed(4)
			return ah
		else:
			rospy.logdebug("%s action server ready",action_server_name)
		
		# sending goal
		goal = SetLightModeGoal()
		goal.mode = mode
		client.send_goal(goal)
		ah.set_client(client)

		ah.wait_inside()
		return ah

#------------------- Mimic section -------------------#
	## Set the mimic of the cob_mimic component.
	#
	# The mode is given by a parameter on the parameter server.
	#
	# \param parameter_name Name of the parameter on the parameter server which holds the mimic mode (string).

	def set_mimic(self,component_name,parameter_name,blocking=False):
		ah = action_handle("set_mimic", component_name, parameter_name, blocking, self.parse)
		if(self.parse):
			return ah
		else:
			ah.set_active()

		rospy.loginfo("Set <<%s>> to <<%s>>", component_name, parameter_name)
			
		# check mimic parameters
		mimic = SetMimicGoal()
		
		if not (type(parameter_name) is str or type(parameter_name) is list): # check outer list
			rospy.logerr("no valid parameter for mimic: not a string or list, aborting...")
			print "parameter is:",parameter_name
			ah.error_code = 3
			return ah
		
		if type(parameter_name) is str:
			mimic.mimic = parameter_name
		elif type(parameter_name) is list:
			if len(parameter_name) != 3:
				rospy.logerr("no valid parameter for mimic: not a list with size 3, aborting...")
				print "parameter is:",parameter_name
				ah.error_code = 3
				return ah
			if ((type(parameter_name[0]) is str) and (type(parameter_name[1]) is float or type(parameter_name[1]) is int) and (type(parameter_name[2]) is float or type(parameter_name[2]) is int)):
				mimic.mimic = parameter_name[0]
				mimic.speed = parameter_name[1]
				mimic.repeat = parameter_name[2]
			else:
				rospy.logerr("no valid parameter for mimic: not a list with [mode, speed, repeat], aborting...")
				print "parameter is:",parameter_name
				ah.error_code = 3
				return ah
		else:
			rospy.logerr("you should never be here")
				
		rospy.logdebug("accepted parameter %s for mimic",parameter_name)

		# call action server
		action_server_name = component_name + "/set_mimic"
		rospy.logdebug("calling %s action server",action_server_name)
		client = actionlib.SimpleActionClient(action_server_name, SetMimicAction)
		# trying to connect to server
		rospy.logdebug("waiting for %s action server to start",action_server_name)
		if not client.wait_for_server(rospy.Duration(5)):
			# error: server did not respond
			rospy.logerr("%s action server not ready within timeout, aborting...", action_server_name)
			ah.set_failed(4)
			return ah
		else:
			rospy.logdebug("%s action server ready",action_server_name)

		
		# sending goal
		client.send_goal(mimic)
		ah.set_client(client)

		ah.wait_inside()
		return ah

#-------------------- Sound section --------------------#
	## Say some text.
	#
	# The text to say may be given by a list of strings or a single string which points to a parameter on the ROS parameter server.
	#
	# \param parameter_name Name of the parameter
	# \param language Language to use for the TTS system
	def say(self,parameter_name,blocking=True):
		component_name = "sound"
		ah = action_handle("say", component_name, parameter_name, blocking, self.parse)
		if(self.parse):
			return ah
		else:
			ah.set_active()
			
		text = ""
		
		# get values from parameter server
		language = rospy.get_param(self.ns_global_prefix + "/" + component_name + "/language","en")
		if type(parameter_name) is str:
			if not rospy.has_param(self.ns_global_prefix + "/" + component_name + "/" + language + "/" + parameter_name):
				rospy.logerr("parameter %s does not exist on ROS Parameter Server, aborting...",self.ns_global_prefix + "/" + component_name + "/" + language + "/" + parameter_name)
				ah.set_failed(2)
				return ah
			param = rospy.get_param(self.ns_global_prefix + "/" + component_name + "/" + language + "/" + parameter_name)
		else:
			param = parameter_name
		
		# check parameters
		if not type(param) is list: # check list
				rospy.logerr("no valid parameter for %s: not a list, aborting...",component_name)
				print "parameter is:",param
				ah.set_failed(3)
				return ah
		else:
			for i in param:
				#print i,"type1 = ", type(i)
				if not type(i) is str:
					rospy.logerr("no valid parameter for %s: not a list of strings, aborting...",component_name)
					print "parameter is:",param
					ah.set_failed(3)
					return ah
				else:
					text = text + i + " "
					rospy.logdebug("accepted parameter <<%s>> for <<%s>>",i,component_name)

		rospy.loginfo("Saying <<%s>>",text)
		
		# call action server
		action_server_name = component_name + "/say"
		rospy.logdebug("calling %s action server",action_server_name)
		client = actionlib.SimpleActionClient(action_server_name, SayAction)
		# trying to connect to server
		rospy.logdebug("waiting for %s action server to start",action_server_name)
		if not client.wait_for_server(rospy.Duration(5)):
			# error: server did not respond
			rospy.logerr("%s action server not ready within timeout, aborting...", action_server_name)
			ah.set_failed(4)
			return ah
		else:
			rospy.logdebug("%s action server ready",action_server_name)

		# sending goal
		client_goal = SayGoal()
		client_goal.text.data = text
		#print client_goal
		client.send_goal(client_goal)
		ah.set_client(client)

		ah.wait_inside()
		return ah

	## Play a sound file.
	#
	# \param parameter_name Name of the parameter
	# \param language Language to use
	def play(self,parameter_name,blocking=True):
		component_name = "sound"
		ah = action_handle("play", component_name, parameter_name, False, self.parse)
		if(self.parse):
			return ah
		else:
			ah.set_active(mode="system")
		
		language = rospy.get_param(self.ns_global_prefix + "/" + component_name + "/language","en")
		if self.wav_path == "":
			wav_path = commands.getoutput("rospack find cob_script_server")
		else:
			wav_path = self.wav_path
		filename = wav_path + "/common/files/" + language + "/" + parameter_name + ".wav"
		
		rospy.loginfo("Playing <<%s>>",filename)
		#self.soundhandle.playWave(filename)
		
		#\todo TODO: check if file exists
		# if filename exists:
		#	do ...
		# else 
		#	ah.set_fail(3)
		#	return ah
		
		if blocking:
			ret = os.system("aplay -q " + filename)
			if ret != 0:
				ah.set_failed(99)
				return ah
		else:
			os.system("aplay -q " + filename + "&") # TODO how to check if execution failed (e.g. file could be found)?
		ah.set_succeeded()
		return ah
		
	def set_wav_path(self,parameter_name,blocking=True):
		if type(parameter_name) is str:
			self.wav_path = parameter_name
		else:
			rospy.logerr("invalid wav_path parameter specified, aborting...")
			print "parameter is:", parameter_name
			ah.set_failed(2)
			return ah		

#------------------- General section -------------------#
	## Sleep for a certain time.
	#
	# \param duration Duration in seconds to sleep.
	#
	def sleep(self,duration):
		ah = action_handle("sleep", "", str(duration), True, self.parse)
		if(self.parse):
			return ah
		else:
			ah.set_active()
		rospy.loginfo("Wait for %f sec",duration)
		rospy.sleep(duration)
		
		ah.set_succeeded()
		return ah

	## Waits for user input.
	#
	# Waits either for a user input or until timeout is reached.
	#
	# \param duration Duration in seconds for timeout.
	# 
	# \todo TODO: implement waiting for timeout
	def wait_for_input(self,duration=0):
		ah = action_handle("wait", "input", str(duration), True, self.parse)
		if(self.parse):
			return ah
		else:
			ah.set_active()
		
		if (duration != 0):
			rospy.logerr("Wait with duration not implemented yet") # \todo TODO: implement waiting with duration
		
		rospy.loginfo("Wait for user input...")
		retVal = raw_input()
		rospy.loginfo("...got string <<%s>>",retVal)
		ah.set_succeeded()
		return retVal

#------------------- action_handle section -------------------#	
## Action handle class.
#
# The action handle is used to implement asynchronous behaviour within the script.
class action_handle:
	## Initializes the action handle.
	def __init__(self, function_name, component_name, parameter_name, blocking, parse):
		global graph
		global function_counter
		self.parent_node = ""
		self.error_code = -1
		self.wait_log = False
		self.function_counter = function_counter
		self.function_name = function_name
		self.component_name = component_name
		self.parameter_name = parameter_name
		self.state = ScriptState.UNKNOWN
		self.blocking = blocking
		self.parse = parse
		self.level = int(rospy.get_param("/script_server/level",100))
		self.state_pub = rospy.Publisher("/script_server/state", ScriptState, queue_size=1)
		self.AppendNode(blocking)
		self.client = actionlib.SimpleActionClient("dummy",ScriptAction)
		self.client_state = 9
		self.client_mode = ""

	## Sets the actionlib client.
	def set_client(self,client):
		self.client = client

	## Sets the execution state to active, if not paused
	def set_active(self,mode=""):
		if mode != "": # not processing an actionlib client
			self.client_mode = mode
			self.client_state = 1
		self.check_pause()
		self.state = ScriptState.ACTIVE
		self.error_code = -1
		self.PublishState()
		
		global ah_counter
		ah_counter += 1
		
	## Checks for pause
	def check_pause(self):
		param_string = "/script_server/pause"
		while bool(rospy.get_param(param_string,False)):
			rospy.logwarn("Script is paused...")
			self.state = ScriptState.PAUSED
			self.PublishState()
			rospy.sleep(1)
		if self.state == ScriptState.PAUSED:
			rospy.loginfo("...continuing script")
		
	## Sets the execution state to succeeded.
	def set_succeeded(self):
		if self.client_mode != "": # not processing an actionlib client
			self.client_state = 3
		self.state = ScriptState.SUCCEEDED
		self.error_code = 0
		self.PublishState()
		
		global ah_counter
		ah_counter -= 1
		
	## Sets the execution state to failed.
	def set_failed(self,error_code):
		if self.client_mode != "": # not processing an actionlib client
			self.client_state = 4
		self.state = ScriptState.FAILED
		self.error_code = error_code
		self.PublishState()

		global ah_counter
		ah_counter -= 1
		
	## Gets the state of an action handle.
	def get_state(self):
		if self.client_mode != "": # not processing an actionlib client
			return self.client_state
		else:
			return self.client.get_state()

	## Gets the error code of an action handle.
	def get_error_code(self):
		return self.error_code
	
	## Returns the graphstring.
	def GetGraphstring(self):
		if type(self.parameter_name) is types.StringType:
			graphstring = str(self.function_counter)+"_"+self.function_name+"_"+self.component_name+"_"+self.parameter_name
		else:
			graphstring = str(self.function_counter)+"_"+self.function_name+"_"+self.component_name
		return graphstring

	## Gets level of function name.
	def GetLevel(self,function_name):
		if (function_name == "move"):
			level = 0
		elif (function_name == "init"):
			level = 1
		elif (function_name == "stop"):
			level = 1
		elif (function_name == "sleep"):
			level = 2
		else:
			level = 100
		return level
		
	## Appends a registered function to the graph.
	def AppendNode(self, blocking=True):
		global graph
		global graph_wait_list
		global function_counter
		global last_node
		graphstring = self.GetGraphstring()
		if self.parse:
			if ( self.level >= self.GetLevel(self.function_name)):
				#print "adding " + graphstring + " to graph"
				graph.add_edge(last_node, graphstring)
				for waiter in graph_wait_list:
					graph.add_edge(waiter, graphstring)
				graph_wait_list=[]
				if blocking:
					last_node = graphstring
				else:
					self.parent_node = graphstring
			#else:
				#print "not adding " + graphstring + " to graph"
		#else:
			#self.PublishState()
		function_counter += 1
		
	## Publishs the state of the action handle
	def PublishState(self):
		script_state = ScriptState()
		script_state.header.stamp = rospy.Time.now()
		script_state.number = self.function_counter
		script_state.function_name = self.function_name
		script_state.component_name = self.component_name
		script_state.full_graph_name = self.GetGraphstring()
		if ( type(self.parameter_name) is str ):
			script_state.parameter_name = self.parameter_name
		else:
			script_state.parameter_name = ""
		script_state.state = self.state
		script_state.error_code = self.error_code
		self.state_pub.publish(script_state)
		
	## Handles wait.
	#
	# This function is meant to be uses directly in the script.
	#
	# \param duration Duration for timeout.
	def wait(self, duration=None):
		global ah_counter
		ah_counter += 1
		self.blocking = True
		self.wait_for_finished(duration,True)

	## Handles inside wait.
	#
	# This function is meant to be uses inside the simple_script_server.
	#
	# \param duration Duration for timeout.
	def wait_inside(self, duration=None):
		if self.blocking:
			self.wait_for_finished(duration,True)
		else:
			thread.start_new_thread(self.wait_for_finished,(duration,False,))
		return self.error_code
	
	## Waits for the action to be finished.
	#
	# If duration is specified, waits until action is finished or timeout is reached.
	#
	# \param duration Duration for timeout.
	# \param logging Enables or disables logging for this wait.
	def wait_for_finished(self, duration, logging):
		global graph_wait_list
		if(self.parse):
			if(self.parent_node != ""):
				graph_wait_list.append(self.parent_node)
			return

		if self.error_code <= 0:			
			if duration is None:
				if logging:
					rospy.loginfo("Wait for <<%s>> reaching <<%s>>...",self.component_name, self.parameter_name)
				self.client.wait_for_result()
			else:
				if logging:
					rospy.loginfo("Wait for <<%s>> reached <<%s>> (max %f secs)...",self.component_name, self.parameter_name,duration)
				if not self.client.wait_for_result(rospy.Duration(duration)):
					if logging:
						rospy.logerr("Timeout while waiting for <<%s>> to reach <<%s>>. Continuing...",self.component_name, self.parameter_name)
					self.set_failed(10)
					return
			# check state of action server
			#print self.client.get_state()
			if self.client.get_state() != 3:
				if logging:
					rospy.logerr("...<<%s>> could not reach <<%s>>, aborting...",self.component_name, self.parameter_name)
				self.set_failed(11)
				return

			if logging:
				rospy.loginfo("...<<%s>> reached <<%s>>",self.component_name, self.parameter_name)
		else:
			rospy.logwarn("Execution of <<%s>> to <<%s>> was aborted, wait not possible. Continuing...",self.component_name, self.parameter_name)
			self.set_failed(self.error_code)
			return
			
		self.set_succeeded() # full success
	
	## Cancel action
	#
	# Cancels action goal(s).
	def cancel(self):
		self.client.cancel_all_goals()

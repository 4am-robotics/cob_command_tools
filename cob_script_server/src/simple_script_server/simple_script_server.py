#!/usr/bin/env python
#
# Copyright 2017 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#	http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import time
import datetime
import os
import sys
import types
import thread
import commands
import math
import threading
import numpy
import itertools

# graph includes
import pygraphviz as pgv

# ROS imports
import rospy
import actionlib

from urdf_parser_py.urdf import URDF

# msg imports
from std_msgs.msg import String,ColorRGBA
from std_srvs.srv import Trigger
from sensor_msgs.msg import JointState
from geometry_msgs.msg import *
from trajectory_msgs.msg import *
from move_base_msgs.msg import *
from control_msgs.msg import *
from tf.transformations import *

# care-o-bot includes
from cob_actions.msg import SetStringAction, SetStringGoal
from cob_sound.msg import *
from cob_script_server.msg import *
from cob_light.msg import LightMode, LightModes, SetLightModeGoal, SetLightModeAction
from cob_mimic.msg import SetMimicGoal, SetMimicAction
from actionlib.msg import TestAction, TestGoal
from actionlib import GoalStatus

graph=""
graph_wait_list=[]
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
		# run script in simulation mode
		self.sss = simple_script_server(parse=True)
		self.Initialize()
		self.Run()

		# save graph on parameter server for further processing
		#self.graph = graph
		rospy.set_param("/script_server/graph", graph.string())
		self.graph_pub.publish(graph.string())
		rospy.loginfo("...parsing finished")
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
				message = "Given navigation mode " + mode + " for " + component_name + " is not valid, aborting..."
				rospy.logerr(message)
				ah.set_failed(33, message)
				return ah
			client = actionlib.SimpleActionClient(action_server_name, MoveBaseAction)
		else:
			parameter_name = self.ns_global_prefix + "/" + component_name + "/action_name"
			if not rospy.has_param(parameter_name):
				message = "Parameter " + parameter_name + " does not exist on ROS Parameter Server, aborting..."
				rospy.logerr(message)
				ah.set_failed(2, message)
				return ah
			action_server_name = rospy.get_param(parameter_name)
			client = actionlib.SimpleActionClient(action_server_name, FollowJointTrajectoryAction)

		# call action server
		rospy.logdebug("calling %s action server",action_server_name)

		if blocking:
			# trying to connect to server
			rospy.logdebug("waiting for %s action server to start",action_server_name)
			if not client.wait_for_server(rospy.Duration(1)):
				# error: server did not respond
				message = "ActionServer " + action_server_name + " not ready within timeout, aborting..."
				rospy.logerr(message)
				ah.set_failed(4, message)
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
	def trigger(self,component_name,service_name,blocking=True):
		ah = action_handle(service_name, component_name, "", blocking, self.parse)
		if(self.parse):
			return ah
		else:
			ah.set_active(mode="service")

		rospy.loginfo("<<%s>> <<%s>>", service_name, component_name)
		parameter_name = self.ns_global_prefix + "/" + component_name + "/service_ns"
		if not rospy.has_param(parameter_name):
			message = "Parameter " + parameter_name + " does not exist on ROS Parameter Server, aborting..."
			rospy.logerr(message)
			ah.set_failed(2, message)
			return ah
		service_ns_name = rospy.get_param(parameter_name)
		service_full_name = service_ns_name + "/" + service_name

		if blocking:
			# check if service is available
			try:
				rospy.wait_for_service(service_full_name,1)
			except rospy.ROSException, e:
				error_message = "%s"%e
				message = "...service <<" + service_name + ">> of <<" + component_name + ">> not available,\n error: " + error_message
				rospy.logerr(message)
				ah.set_failed(4, message)
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
			message = "...calling <<" + service_name + ">> of <<" + component_name + ">> not successfull,\n error: " + error_message
			rospy.logerr(message)
			ah.set_failed(10, message)
			return ah

		if blocking:
			# evaluate sevice response
			if not resp.success:
				message = "...response of <<" + service_name + ">> of <<" + component_name + ">> not successfull,\n error: " + resp.message
				rospy.logerr(message)
				ah.set_failed(10, message)
				return ah

			# full success
			rospy.loginfo("...<<%s>> is <<%s>>", component_name, service_name)
			ah.set_succeeded() # full success
		return ah

	## Allows to trigger actions of the type actionlib/TestAction
	#
	# Based on the component and action name, the corresponding ActionServer will be called.
	#
	# \param component_name Name of the component.
	# \param action_name Name of the action.
	# \param blocking Whether to wait for the Action to return.
	def trigger_action(self,component_name,action_name,blocking=True):
		ah = action_handle(action_name, component_name, "", blocking, self.parse)
		if(self.parse):
			return ah
		else:
			ah.set_active(mode="action")

		rospy.loginfo("<<%s>> <<%s>>", action_name, component_name)
		#parameter_name = self.ns_global_prefix + "/" + component_name + "/service_ns"
		#if not rospy.has_param(parameter_name):
			#message = "Parameter " + parameter_name + " does not exist on ROS Parameter Server, aborting..."
			#rospy.logerr(message)
			#ah.set_failed(2, message)
			#return ah
		#service_ns_name = rospy.get_param(parameter_name)
		action_full_name = "/" + component_name + "/" + action_name
		action_client = actionlib.SimpleActionClient(action_full_name, TestAction)

		if blocking:
			# check if action is available
			if not action_client.wait_for_server(rospy.Duration(1)):
				message = "ActionServer %s is not running"%action_full_name
				rospy.logerr(message)
				ah.set_failed(4, message)
				return ah

		# call the action
		if blocking:
			rospy.loginfo("Wait for <<%s>> to <<%s>>...", component_name, action_name)
			goal_state = action_client.send_goal_and_wait(TestGoal())
			if not (action_client.get_state() == GoalStatus.SUCCEEDED):
				message = "...state of <<" + action_name + ">> of <<" + component_name + ">> : " + GoalStatus.to_string(action_client.get_state())
				rospy.logerr(message)
				ah.set_failed(10, message)
				return ah
		else:
			action_client.send_goal(TestGoal())

		# full success
		rospy.loginfo("...<<%s>> is <<%s>>", component_name, action_name)
		ah.set_succeeded() # full success
		return ah


	## Allows to trigger actions of the type cob_actions/SetString
	#
	# Based on the component and action name, the corresponding ActionServer will be called.
	#
	# \param component_name Name of the component (namespace of the action).
	# \param data The data to be send in the ActionGoal.
	# \param blocking Whether to wait for the Action to return.
	def string_action(self,component_name, data, blocking):
		action_name = "set_string"
		ah = action_handle(action_name, component_name, "", blocking, self.parse)
		if(self.parse):
			return ah
		else:
			ah.set_active(mode="action")

		rospy.loginfo("<<%s>> <<%s>>", action_name, component_name)
		action_full_name = "/" + component_name + "/" + action_name
		action_client = actionlib.SimpleActionClient(action_full_name, SetStringAction)

		if blocking:
			# check if action is available
			if not action_client.wait_for_server(rospy.Duration(1)):
				message = "ActionServer %s is not running"%action_full_name
				rospy.logerr(message)
				ah.set_failed(4, message)
				return ah

		# call the action
		goal = SetStringGoal()
		goal.data = data
		if blocking:
			rospy.loginfo("Wait for <<%s>> to <<%s>>...", component_name, action_name)
			goal_state = action_client.send_goal_and_wait(goal)
			if not (action_client.get_state() == GoalStatus.SUCCEEDED):
				message = "...state of <<" + action_name + ">> of <<" + component_name + ">> : " + GoalStatus.to_string(action_client.get_state())
				rospy.logerr(message)
				ah.set_failed(10, message)
				return ah
		else:
			action_client.send_goal(goal)

		# full success
		rospy.loginfo("...<<%s>> is <<%s>>", component_name, action_name)
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
	def move(self,component_name,parameter_name,blocking=True, mode=None, speed_factor=1.0, urdf_vel=False, default_vel=None):
		if component_name == "base":
			return self.move_base(component_name,parameter_name,blocking, mode)
		else:
			return self.move_traj(component_name,parameter_name,blocking, speed_factor=speed_factor, urdf_vel=urdf_vel, default_vel=default_vel)

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
			full_parameter_name = self.ns_global_prefix + "/" + component_name + "/" + parameter_name
			if not rospy.has_param(full_parameter_name):
				message = "Parameter " + full_parameter_name + " does not exist on ROS Parameter Server, aborting..."
				rospy.logerr(message)
				ah.set_failed(2, message)
				return ah
			param = rospy.get_param(self.ns_global_prefix + "/" + component_name + "/" + parameter_name)
		else:
			param = parameter_name

		# check trajectory parameters
		if not type(param) is list: # check outer list
			message = "No valid parameter for " + component_name + ": not a list, aborting..."
			rospy.logerr(message)
			print "parameter is:",param
			ah.set_failed(3, message)
			return ah
		else:
			#print i,"type1 = ", type(i)
			DOF = 3
			if not len(param) == DOF: # check dimension
				message = "No valid parameter for " + component_name + ": dimension should be " + str(DOF) + " but is " + str(len(param)) + ", aborting..."
				rospy.logerr(message)
				print "parameter is:",param
				ah.set_failed(3, message)
				return ah
			else:
				for i in param:
					#print i,"type2 = ", type(i)
					if not ((type(i) is float) or (type(i) is int)): # check type
						#print type(i)
						message = "No valid parameter for " + component_name + ": not a list of float or int, aborting..."
						rospy.logerr(message)
						print "parameter is:",param
						ah.set_failed(3, message)
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
			message = "Given navigation mode " + mode + " for " + component_name + " is not valid, aborting..."
			rospy.logerr(message)
			ah.set_failed(33, message)
			return ah

		rospy.logdebug("calling %s action server",action_server_name)
		client = actionlib.SimpleActionClient(action_server_name, MoveBaseAction)
		# trying to connect to server
		rospy.logdebug("waiting for %s action server to start",action_server_name)
		if not client.wait_for_server(rospy.Duration(1)):
			# error: server did not respond
			message = "ActionServer " + action_server_name + " not ready within timeout, aborting..."
			rospy.logerr(message)
			ah.set_failed(4, message)
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

	def _determine_desired_velocity(self, default_vel, start_pos, component_name, joint_names, speed_factor, urdf_vel):
		if default_vel:  # passed via argument
			rospy.logdebug("using default_vel from argument")
			if (type(default_vel) is float) or (type(default_vel) is int):
				default_vel = numpy.array([default_vel for _ in start_pos])
			elif (type(default_vel) is list) and (len(default_vel) == len(start_pos)) and all(
					((type(item) is float) or (type(item) is int)) for item in default_vel):
				default_vel = default_vel
			else:
				raise ValueError("argument 'default_vel' {} has wrong format (must be float/int or list of float/int) with proper dimensions.".format(default_vel))
		else:  # get from parameter server
			rospy.logdebug("using default_vel parameter server")
			param_string = self.ns_global_prefix + "/" + component_name + "/default_vel"
			if not rospy.has_param(param_string):
				default_vel = numpy.array([0.1 for _ in start_pos])  # rad/s
				rospy.logwarn(
					"parameter '{}' does not exist on ROS Parameter Server, using default_vel {} [rad/sec].".format(
						param_string, default_vel))
			else:
				param_vel = rospy.get_param(param_string)
				if (type(param_vel) is float) or (type(param_vel) is int):
					default_vel = numpy.array([param_vel for _ in start_pos])
				elif (type(param_vel) is list) and (len(param_vel) == len(start_pos)) and all(
						((type(item) is float) or (type(item) is int)) for item in param_vel):
					default_vel = numpy.array(param_vel)
				else:
					default_vel = numpy.array([0.1 for _ in start_pos])  # rad/s
					rospy.logwarn(
						"parameter '{}' {} has wrong format (must be float/int or list of float/int), using default_vel {} [rad/sec].".format(
							param_string, param_vel, default_vel))
		rospy.logdebug("default_vel: {}".format(default_vel))

		try:
			robot_description = rospy.search_param('robot_description') if rospy.search_param('robot_description') else '/robot_description'
			rospy.loginfo("Initialize urdf structure from '{}'".format(robot_description))
			robot_urdf = URDF.from_parameter_server(key=robot_description)
		except KeyError as key_err:
			message = "Unable to initialize urdf structure: {}".format(key_err.message)
			rospy.logerr(message)
			raise ValueError(message)

		limit_vel = []
		for idx, joint_name in enumerate(joint_names):
			try:
				limit_vel.append(robot_urdf.joint_map[joint_name].limit.velocity)
			except KeyError:
				limit_vel.append(default_vel[idx])

		# limit_vel from urdf or default_vel (from argument or parameter server)
		if urdf_vel:
			rospy.logdebug("using default_vel from urdf_limits")
			velocities = limit_vel
		else:
			rospy.logdebug("using default_vel from argument or parameter server")
			velocities = list(default_vel)

		# check velocity limits
		desired_vel = numpy.array(velocities)*speed_factor
		if (numpy.any(desired_vel > numpy.array(limit_vel))):
			raise ValueError("desired velocities {} exceed velocity limits {},...aborting".format(desired_vel, numpy.array(limit_vel)))

		if (numpy.any(desired_vel <= numpy.zeros_like(desired_vel))):
			raise ValueError("desired velocities {} cannot be zero or negative,...aborting".format(desired_vel))
		rospy.loginfo("Velocities are: {}".format(desired_vel))
		return desired_vel

	## Parse and compose trajectory message
	def compose_trajectory(self, component_name, parameter_name, speed_factor=1.0, urdf_vel=False, default_vel=None):
		if urdf_vel and default_vel:
			rospy.logerr("arguments not valid - cannot set 'urdf_vel' and 'default_vel' at the same time, aborting...")
			return (JointTrajectory(), 3)

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
					print "joint_names are:", joint_names
					return (JointTrajectory(), 3)
				else:
					rospy.logdebug("accepted joint_names for component %s",component_name)

		# get joint values from parameter server
		if type(parameter_name) is str:
			full_parameter_name = self.ns_global_prefix + "/" + component_name + "/" + parameter_name
			if not rospy.has_param(full_parameter_name):
				rospy.logerr("parameter %s does not exist on ROS Parameter Server, aborting...",full_parameter_name)
				return (JointTrajectory(), 2)
			param = rospy.get_param(full_parameter_name)
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
				full_parameter_name = self.ns_global_prefix + "/" + component_name + "/" + point
				if not rospy.has_param(full_parameter_name):
					rospy.logerr("parameter %s does not exist on ROS Parameter Server, aborting...",full_parameter_name)
					return (JointTrajectory(), 2)
				point = rospy.get_param(full_parameter_name)
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
		rospy.logdebug("traj after param: {}".format(traj))

		# get current pos
		timeout = 3.0
		try:
			joint_state = rospy.wait_for_message("/" + component_name + "/joint_states", JointState, timeout=timeout)  # type: JointState
			# make sure we have the same joint order
			start_pos = []
			for name in joint_names:
				idx = joint_state.name.index(name)
				start_pos.append(joint_state.position[idx])
		except rospy.ROSException as e:
			rospy.logwarn("no joint states received from %s within timeout of %ssec. using default point time of 8sec.", component_name, str(timeout))
			start_pos = []

		# insert start_pos to trajectory
		if start_pos:
			traj.insert(0,start_pos)
			rospy.logdebug("traj after add: {}".format(traj))

		# convert to ROS trajectory message
		traj_msg = JointTrajectory()
		# if no timestamp is set in header, this means that the trajectory starts "now"
		traj_msg.joint_names = joint_names
		point_nr = 0
		traj_time = 0

		# get desired_vel
		try:
			desired_vel = self._determine_desired_velocity(default_vel, start_pos, component_name, joint_names, speed_factor, urdf_vel)
		except ValueError as val_err:
			rospy.logerr(val_err.message)
			return (JointTrajectory(), 3)

		# get default_acc
		param_string = self.ns_global_prefix + "/" + component_name + "/default_acc"
		if not rospy.has_param(param_string):
			default_acc = numpy.array([1.0 for _ in start_pos]) # rad^2/s
			rospy.logwarn("parameter '{}' does not exist on ROS Parameter Server, using default_acc {} [rad^2/sec].".format(param_string,default_acc))
		else:
			param_acc = rospy.get_param(param_string)
			if (type(param_acc) is float) or (type(param_acc) is int):
				default_acc = numpy.array([param_acc for _ in start_pos])
			elif (type(param_acc) is list) and (len(param_acc) == len(start_pos)) and all(
					((type(item) is float) or (type(item) is int)) for item in param_acc):
				default_acc = numpy.array(param_acc)
			else:
				default_acc = numpy.array([1.0 for _ in start_pos]) # rad^2/s
				rospy.logwarn("parameter '{}' {} has wrong format (must be float/int or list of float/int), using default_acc {} [rad^2/sec].".format(param_string,param_acc,default_acc))

		# filter duplicates
		def is_close(a,b):
			return numpy.all(numpy.isclose(numpy.array(a), numpy.array(b), atol=0.001))

		def unique_next(elems):
			for i, (curr, nxt) in enumerate(zip(elems, elems[1:]+[None])):
				if not nxt:
					yield curr
				else:
					if not is_close(curr, nxt):
						yield curr
					else:
						rospy.logdebug("dropping trajectory point {} due to is close: (curr {}, nxt: {})".format(i, curr, nxt))

		try:
			traj = list(unique_next(traj))
		except Exception as e:
			rospy.logerr(e.message)
			return (JointTrajectory(), 3)
		rospy.logdebug("traj after unique_nxt: {}".format(traj))

		# calculate time_from_start
		for point in traj:
			point_nr = point_nr + 1
			point_msg = JointTrajectoryPoint()
			point_msg.positions = point

			# use hardcoded point_time if no start_pos available
			if start_pos != []:
				point_time = self.calculate_point_time(start_pos, point, desired_vel, default_acc)
			else:
				point_time = 8*point_nr

			start_pos = point
			point_msg.time_from_start=rospy.Duration(point_time + traj_time)
			traj_time += point_time
			traj_msg.points.append(point_msg)

		# calculate traj_point velocities and accelerations
		prevs, items, nexts = itertools.tee(traj_msg.points, 3)
		prevs = itertools.chain([None], prevs)
		nexts = itertools.chain(itertools.islice(nexts, 1, None), [None])
		for idx, (pre, curr, post) in enumerate(itertools.izip(prevs, items, nexts)):
			traj_msg.points[idx].velocities = self.calculate_point_velocities(pre, curr, post)
			traj_msg.points[idx].accelerations = self.calculate_point_accelerations(pre, curr, post)

		# drop first trajectory point because it is equal to start_pos
		traj_msg.points = traj_msg.points[1:]

		return (traj_msg, 0)

	def calculate_point_time(self, start_pos, end_pos, default_vel, default_acc):
		if isinstance(default_vel, float):
			default_vel = numpy.array([default_vel for _ in start_pos])

		if isinstance(default_acc, float):
			default_acc = numpy.array([default_acc for _ in start_pos])

		try:
			# dist: Distance traveled in the move by each joint
			dist = numpy.abs(numpy.array(start_pos) - numpy.array(end_pos))

			t1 = default_vel / default_acc  # Time needed for joints to accelerate to desired velocity
			s1 = default_acc / 2.0 * t1**2  # Distance traveled during this time

			t = numpy.zeros_like(dist)

			for i in range(len(start_pos)):
				if 2 * s1[i] < dist[i]:
					# If we can accelerate and decelerate in less than the total distance, then:
					# accelerate up to speed, travel at that speed for a bit and then decelerate, a three phase trajectory:
					# with constant velocity phase (acc, const vel, dec)
					# 1st phase: accelerate from v=0 to v=default_vel with a=default_acc in t=t1. Need s1 distance for this
					# 2nd phase: constant velocity with v=default_vel and t=t2
					# 3rd phase: deceleration (analog to 1st phase). Need s1 distance for this
					#  ^
					#  |    __2__
					#  |   /     \
					# v|1 /       \ 3
					#  | /         \
					#  o--------------->
					#         t     d_max
					s2 = dist[i] - 2 * s1[i]
					t2 = s2 / default_vel[i]
					t[i] = 2 * t1[i] + t2
				else:
					# If we don't have enough distance to get to full speed, then we do
					# without constant velocity phase (only acc and dec, so a two phase trajectory)
					# 1st phase: accelerate from v=0 to v=default_vel with a=default_acc in t=t1
					# 2nd phase: missing because distance is to short (we already reached the distance with the acc and dec phase)
					# 3rd phase: deceleration (analog to 1st phase)
					t[i] = numpy.sqrt(dist[i] / default_acc[i])

			# Instead of deciding per joint if we can do a three or two-phase trajectory,
			# we can simply take the slowest joint of them all and select that.
			#point_time = max(numpy.max(t), 0.4)	 # use minimal point_time
			point_time = numpy.max(t)
		except ValueError as e:
			print("Value Error: {}".format(e))
			print("Likely due to mimic joints. Using default point_time: 3.0 [sec]")
			point_time = 3.0  # use default point_time
		return point_time

	def calculate_point_velocities(self, prev, curr, post):
		rospy.logdebug("calculate_point_velocities")
		rospy.logdebug("prev: {}".format(prev))
		rospy.logdebug("curr: {}".format(curr))
		rospy.logdebug("post: {}".format(post))

		# set zero velocities for last trajectory point only
		if not post:
			rospy.logdebug("not has post")
			point_velocities = numpy.zeros(len(curr.positions))
		elif not prev:
			rospy.logdebug("not has prev")
			point_velocities = numpy.zeros(len(curr.positions))
		else:
			rospy.logdebug("has prev, has post")
			# calculate based on difference quotient post-curr
			point_velocities = numpy.divide(numpy.subtract(numpy.array(post.positions), numpy.array(prev.positions)), numpy.array([(post.time_from_start-prev.time_from_start).to_sec()]*len(curr.positions)))
			rospy.logdebug("point_velocities diff quot: {}".format(point_velocities))

			# check sign change or consecutive points too close
			rospy.logdebug("has prev")
			curr_prev_diff = numpy.subtract(numpy.array(curr.positions), numpy.array(prev.positions))
			post_curr_diff = numpy.subtract(numpy.array(post.positions), numpy.array(curr.positions))
			rospy.logdebug("curr_prev_diff: {}".format(curr_prev_diff))
			rospy.logdebug("post_curr_diff: {}".format(post_curr_diff))
			same_sign = numpy.equal(numpy.sign(curr_prev_diff), numpy.sign(post_curr_diff))
			prev_close = numpy.isclose(curr_prev_diff, numpy.zeros_like(curr_prev_diff), atol=0.01)
			post_close = numpy.isclose(post_curr_diff, numpy.zeros_like(post_curr_diff), atol=0.01)
			rospy.logdebug("same_sign: {}".format(same_sign))
			rospy.logdebug("prev_close: {}".format(prev_close))
			rospy.logdebug("post_close: {}".format(post_close))

			for idx, vel in enumerate(point_velocities):
				if prev_close[idx]:
					rospy.logdebug("prev close for joint {} - setting vel to 0.0".format(idx))
					point_velocities[idx] = 0.0
				if post_close[idx]:
					rospy.logdebug("post close for joint {} - setting vel to 0.0".format(idx))
					point_velocities[idx] = 0.0
				if not same_sign[idx]:
					rospy.logdebug("sign change for joint {} - setting vel to 0.0".format(idx))
					point_velocities[idx] = 0.0

		rospy.logdebug("point_velocities: {}".format(point_velocities))
		return list(point_velocities)

	def calculate_point_accelerations(self, prev, curr, post):
		return [0]*len(curr.positions)

	## Deals with all kind of trajectory movements for different components.
	#
	# A trajectory will be sent to the actionlib interface of the corresponding component.
	#
	# \param component_name Name of the component.
	# \param parameter_name Name of the parameter on the ROS parameter server.
	# \param blocking Bool value to specify blocking behaviour.
	def move_traj(self,component_name,parameter_name,blocking, speed_factor=1.0, urdf_vel=False, default_vel=None):
		ah = action_handle("move", component_name, parameter_name, blocking, self.parse)
		if(self.parse):
			return ah
		else:
			ah.set_active()

		rospy.loginfo("Move <<%s>> to <<%s>>",component_name,parameter_name)
		(traj_msg, error_code) = self.compose_trajectory(component_name, parameter_name, speed_factor=speed_factor, urdf_vel=urdf_vel, default_vel=default_vel)
		if error_code != 0:
			message = "Composing the trajectory failed with error: " + str(error_code)
			ah.set_failed(error_code, message)
			return ah

		# call action server
		parameter_name = self.ns_global_prefix + "/" + component_name + "/action_name"
		if not rospy.has_param(parameter_name):
			message = "Parameter " + parameter_name + " does not exist on ROS Parameter Server, aborting..."
			rospy.logerr(message)
			ah.set_failed(2, message)
			return ah
		action_server_name = rospy.get_param(parameter_name)
		rospy.logdebug("calling %s action server",action_server_name)
		client = actionlib.SimpleActionClient(action_server_name, FollowJointTrajectoryAction)
		# trying to connect to server
		rospy.logdebug("waiting for %s action server to start",action_server_name)
		if not client.wait_for_server(rospy.Duration(1)):
			# error: server did not respond
			message = "ActionServer " + action_server_name + " not ready within timeout, aborting..."
			rospy.logerr(message)
			ah.set_failed(4, message)
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
			message = "Parameter " + parameter_topic_name + " does not exist on ROS Parameter Server, aborting..."
			rospy.logerr(message)
			ah.set_failed(3, message)
			return ah
		topic_name = rospy.get_param(parameter_topic_name)
		rospy.loginfo("Move base relatively by <<%s>>", parameter_name)

		# step 0: check validity of parameters:
		if not len(parameter_name) == 3 or not isinstance(parameter_name[0], (int, float)) or not isinstance(parameter_name[1], (int, float)) or not isinstance(parameter_name[2], (int, float)):
			message = "Parameter " + parameter_name + " not formated correctly (non-numeric list), aborting move_base_rel"
			rospy.logerr(message)
			print("parameter_name must be numeric list of length 3; (relative x and y transl [m], relative rotation [rad])")
			ah.set_failed(3, message)
			return ah
		max_rel_trans_step = 1.0 # [m]
		max_rel_rot_step = math.pi/2 # [rad]
		if math.sqrt(parameter_name[0]**2 + parameter_name[1]**2) > max_rel_trans_step:
			message = "Parameter " + str(parameter_name) + " exceeds maximal relative translation step (" + str(max_rel_trans_step) + "), aborting move_base_rel"
			rospy.logerr(message)
			ah.set_failed(3, message)
			return(ah)
		if abs(parameter_name[2]) > max_rel_rot_step:
			message = "Parameter " + str(parameter_name) + " exceeds maximal relative rotation step (" + str(max_rel_rot_step) + "), aborting move_base_rel"
			rospy.logerr(message)
			ah.set_failed(3, message)
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

	## Relative movement
	## Deals with all kind of relative movements for different components.
	#
	# Based on the component, the corresponding move functions will be called.
	#
	# \param component_name Name of the component.
	# \param parameter_name List of movements for each joint of the component
	# \param blocking Bool value to specify blocking behaviour.
	#
	# # throws error code 3 in case of invalid parameter_name vector
	def move_rel(self, component_name, parameter_name, blocking=True, speed_factor=1.0, urdf_vel=False, default_vel=None):
		ah = action_handle("move_rel", component_name, parameter_name, blocking, self.parse)
		if(self.parse):
			return ah
		else:
			ah.set_active(mode="topic")

		rospy.loginfo("Move <<%s>> relatively by <<%s>>", component_name, parameter_name)

		# step 0: check validity of parameters:
		if not isinstance(parameter_name, list):
			message = "Parameter " + str(parameter_name) + " not formated correctly (not a list), aborting move_rel"
			rospy.logerr(message)
			ah.set_failed(3, message)
			return ah
		for parameter in parameter_name:
			if not isinstance(parameter, list):
				message = "Parameter " + str(parameter_name) + " not formated correctly (not a list of lists), aborting move_rel"
				rospy.logerr(message)
				ah.set_failed(3, message)
				return ah
			for param in parameter:
				if not isinstance(param, (int, float)):
					message = "Parameter " + str(parameter) + " not formated correctly (not a list of numeric lists), aborting move_rel"
					rospy.logerr(message)
					ah.set_failed(3, message)
					return ah

		# step 1: get current position
		timeout = 1.0
		try:
			joint_state_message = rospy.wait_for_message("/" + component_name + "/joint_states", JointState, timeout = timeout)
			joint_names = list(joint_state_message.name)
			start_pos = list(joint_state_message.position)
		except rospy.ROSException as e:
			message = "no joint states received from %s within timeout of %ssec."%(component_name, str(timeout))
			rospy.logerr(message)
			ah.set_failed(3, message)
			return ah

		# step 2: get joint limits from urdf
		try:
			robot_description = rospy.search_param('robot_description') if rospy.search_param('robot_description') else '/robot_description'
			rospy.loginfo("Initialize urdf structure from '{}'".format(robot_description))
			robot_urdf = URDF.from_parameter_server(key=robot_description)
		except KeyError as key_err:
			message = "Unable to initialize urdf structure: {}".format(key_err.message)
			rospy.logerr(message)
			ah.set_failed(3, message)
			return ah
		limits = {}
		for joint in robot_urdf.joints:
			limits.update({joint.name : joint.limit})
		
		# step 3 calculate and send goal
		end_poses = []
		for move_rel_param in parameter_name:
			end_pos = []
			if len(start_pos) == len(move_rel_param) and len(joint_names) == len(move_rel_param):
				for i in range(len(joint_names)):
					if (end_poses == []):
						pos_val = start_pos[i] + move_rel_param[i]
					# for multiple movements, append move_rel_param to previous calculated pose
					else:
						pos_val = end_poses[-1][i] + move_rel_param[i]
					# check if joint limits are exceeded
					if limits[joint_names[i]].upper > pos_val and limits[joint_names[i]].lower < pos_val:
						end_pos.append(pos_val)
					else:
						message = "Relative motion wil exceed absolute limits! %s would go to %f but limits are: %f (upper) and %f (lower)"%(joint_names[i], pos_val, limits[joint_names[i]].upper, limits[joint_names[i]].lower)
						rospy.logerr(message)
						ah.set_failed(3, message)
						return ah
			else:
				message = "Parameters %s don't fit with joint states!"%str(move_rel_param)
				rospy.logerr(message)
				ah.set_failed(3, message)
				return ah
			end_poses.append(end_pos)

		return self.move_traj(component_name, end_poses, blocking, speed_factor=speed_factor, urdf_vel=urdf_vel, default_vel=default_vel)

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
				return 2, None
			param = rospy.get_param(full_parameter_name)
		else:
			param = parameter_name

		# check color parameters
		if not type(param) is list: # check outer list
			rospy.logerr("no valid parameter for light: not a list, aborting...")
			print "parameter is:",param
			return 3, None
		else:
			if not len(param) == 4: # check dimension
				rospy.logerr("no valid parameter for light: dimension should be 4 (r,g,b,a) and is %d, aborting...",len(param))
				print "parameter is:",param
				return 3, None
			else:
				for i in param:
					#print i,"type1 = ", type(i)
					if not ((type(i) is float) or (type(i) is int)): # check type
						#print type(i)
						rospy.logerr("no valid parameter for light: not a list of float or int, aborting...")
						print "parameter is:",param
						return 3, None
					else:
						rospy.logdebug("accepted parameter %f for light",i)

		# convert to ColorRGBA message
		color = ColorRGBA()
		color.r = param[0]
		color.g = param[1]
		color.b = param[2]
		color.a = param[3] # Transparency
		return 0, color

	def set_light(self,component_name,parameter_name,blocking=False):
		ah = action_handle("set_light", component_name, parameter_name, blocking, self.parse)
		if(self.parse):
			return ah
		else:
			ah.set_active()
		rospy.loginfo("Set <<%s>> to <<%s>>", component_name, parameter_name)

		mode = LightMode()
		mode.mode = LightModes.STATIC
		color = ColorRGBA()
		(error,color) = self.compose_color(component_name, parameter_name)
		if error != 0:
			message = "Composing the color failed with error: " + str(error)
			ah.set_failed(error, message)
			return ah
		mode.colors = []
		mode.colors.append(color)

		# call action server
		action_server_name = component_name + "/set_light"
		rospy.logdebug("calling %s action server",action_server_name)
		client = actionlib.SimpleActionClient(action_server_name, SetLightModeAction)
		# trying to connect to server
		rospy.logdebug("waiting for %s action server to start",action_server_name)
		if not client.wait_for_server(rospy.Duration(1)):
			# error: server did not respond
			message = "ActionServer " + action_server_name + " not ready within timeout, aborting..."
			rospy.logerr(message)
			ah.set_failed(4, message)
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
			message = "No valid parameter for mimic: not a string or list, aborting..."
			rospy.logerr(message)
			print "parameter is:",parameter_name
			ah.set_failed(3, message)
			return ah

		if type(parameter_name) is str:
			mimic.mimic = parameter_name
		elif type(parameter_name) is list:
			if len(parameter_name) != 3:
				message = "No valid parameter for mimic: not a list with size 3, aborting..."
				rospy.logerr(message)
				print "parameter is:",parameter_name
				ah.set_failed(3, message)
				return ah
			if ((type(parameter_name[0]) is str) and (type(parameter_name[1]) is float or type(parameter_name[1]) is int) and (type(parameter_name[2]) is float or type(parameter_name[2]) is int)):
				mimic.mimic = parameter_name[0]
				mimic.speed = parameter_name[1]
				mimic.repeat = parameter_name[2]
			else:
				message = "No valid parameter for mimic: not a list with [mode, speed, repeat], aborting..."
				rospy.logerr(message)
				print "parameter is:",parameter_name
				ah.set_failed(3, message)
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
		if not client.wait_for_server(rospy.Duration(1)):
			# error: server did not respond
			message = "ActionServer " + action_server_name + " not ready within timeout, aborting..."
			rospy.logerr(message)
			ah.set_failed(4, message)
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
	def say(self,component_name,parameter_name,blocking=True):
		ah = action_handle("say", component_name, parameter_name, blocking, self.parse)
		if(self.parse):
			return ah
		else:
			ah.set_active()

		text = ""
		# get values from parameter server
		language = rospy.get_param(self.ns_global_prefix + "/" + component_name + "/language","en")
		if type(parameter_name) is str:
			full_parameter_name = self.ns_global_prefix + "/" + component_name + "/" + language + "/" + parameter_name
			if not rospy.has_param(full_parameter_name):
				message = "Parameter " + full_parameter_name + " does not exist on ROS Parameter Server, aborting..."
				rospy.logerr(message)
				ah.set_failed(2, message)
				return ah
			param = rospy.get_param(full_parameter_name)
		else:
			param = parameter_name

		# check parameters
		if not type(param) is list: # check list
			message = "No valid parameter for " + component_name + ": not a list, aborting..."
			rospy.logerr(message)
			print "parameter is:",param
			ah.set_failed(3, message)
			return ah
		else:
			for i in param:
				#print i,"type1 = ", type(i)
				if not type(i) is str:
					message = "No valid parameter for " + component_name + ": not a list of strings, aborting..."
					rospy.logerr(message)
					print "parameter is:",param
					ah.set_failed(3, message)
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
		if not client.wait_for_server(rospy.Duration(1)):
			# error: server did not respond
			message = "ActionServer " + action_server_name + " not ready within timeout, aborting..."
			rospy.logerr(message)
			ah.set_failed(4, message)
			return ah
		else:
			rospy.logdebug("%s action server ready",action_server_name)

		# sending goal
		client_goal = SayGoal()
		client_goal.text = text
		#print client_goal
		client.send_goal(client_goal)
		ah.set_client(client)
		ah.wait_inside()
		return ah

	## Play a sound file.
	#
	# \param parameter_name Name of the parameter
	# \param language Language to use
	def play(self,component_name, parameter_name,blocking=True):
		ah = action_handle("play", component_name, parameter_name, False, self.parse)
		if(self.parse):
			return ah
		else:
			ah.set_active()

		if not (type(parameter_name) is str or type(parameter_name) is list): # check outer list
			message = "No valid parameter for play: not a string or list, aborting..."
			rospy.logerr(message)
			print "parameter is:",parameter_name
			ah.set_failed(3, message)
			return ah

		if type(parameter_name) is str:
			full_parameter_name = self.ns_global_prefix + "/" + component_name + "/" + "audio_file_path"
			if not rospy.has_param(full_parameter_name):
				message = "Parameter " + full_parameter_name + " does not exist on ROS Parameter Server, aborting..."
				rospy.logerr(message)
				ah.set_failed(2, message)
				return ah
			filename = rospy.get_param(full_parameter_name) + "/" + parameter_name + ".wav"
		elif type(parameter_name) is list:
			if len(parameter_name) != 3:
				message = "No valid parameter for play: not a list with size 3, aborting..."
				rospy.logerr(message)
				print "parameter is:",parameter_name
				ah.set_failed(3, message)
				return ah
			if ((type(parameter_name[0]) is str) and (type(parameter_name[1]) is str) and (type(parameter_name[2]) is str)):
				filename = parameter_name[1] + "/" + parameter_name[0] + "." + parameter_name[2]
			else:
				message = "No valid parameter for play: not a list with [filename, file_path, file_suffix], aborting..."
				rospy.logerr(message)
				print "parameter is:",parameter_name
				ah.set_failed(3, message)
				return ah
		else:
			rospy.logerr("you should never be here")
		rospy.logdebug("accepted parameter %s for play",parameter_name)

		action_server_name = component_name + "/play"
		rospy.logdebug("calling %s action server",action_server_name)
		client = actionlib.SimpleActionClient(action_server_name, PlayAction)
		# trying to connect to server
		rospy.logdebug("waiting for %s action server to start",action_server_name)
		if not client.wait_for_server(rospy.Duration(1)):
			# error: server did not respond
			message = "ActionServer " + action_server_name + " not ready  within timeout, aborting..."
			rospy.logerr(message)
			ah.set_failed(4, message)
			return ah
		else:
			rospy.logdebug("%s action server ready",action_server_name)

		# sending goal
		rospy.loginfo("Playing <<%s>>",filename)
		client_goal = PlayGoal()
		client_goal.filename = filename
		#print client_goal
		client.send_goal(client_goal)
		ah.set_client(client)
		ah.wait_inside()
		ah.set_succeeded()
		return ah

	def set_wav_path(self,parameter_name,blocking=True):
		if type(parameter_name) is str:
			self.wav_path = parameter_name
		else:
			message = "Invalid wav_path parameter specified, aborting..."
			rospy.logerr(message)
			print "parameter is:", parameter_name
			ah.set_failed(2, message)
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
		self.parent_node = ""
		self.error_code = -1
		self.success = False
		self.message = ""
		self.function_name = function_name
		self.component_name = component_name
		self.parameter_name = parameter_name
		self.state = ScriptState.UNKNOWN
		self.blocking = blocking
		self.parse = parse
		self.level = int(rospy.get_param("/script_server/level",100))
		self.state_pub = rospy.Publisher("/script_server/state", ScriptState, queue_size=1)
		self.AppendNode(blocking)
		self.client = None
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
	def set_succeeded(self,message=""):
		if self.client_mode != "": # not processing an actionlib client
			self.client_state = 3
		self.state = ScriptState.SUCCEEDED
		self.error_code = 0
		self.success = True
		self.message = message
		self.PublishState()

		global ah_counter
		ah_counter -= 1

	## Sets the execution state to failed.
	def set_failed(self,error_code,message=""):
		if self.client_mode != "": # not processing an actionlib client
			self.client_state = 4
		self.state = ScriptState.FAILED
		self.error_code = error_code
		self.success = False
		self.message = message
		self.PublishState()

		global ah_counter
		ah_counter -= 1

	## Gets the state of an action handle.
	def get_state(self):
		if self.client_mode != "": # not processing an actionlib client
			return self.client_state
		elif self.client == None:
			return None
		else:
			return self.client.get_state()

	## Gets the error code of an action handle.
	def get_error_code(self):
		return self.error_code

	## Returns the graphstring.
	def GetGraphstring(self):
		if type(self.parameter_name) is types.StringType:
			graphstring = str(datetime.datetime.utcnow())+"_"+self.function_name+"_"+self.component_name+"_"+self.parameter_name
		else:
			graphstring = str(datetime.datetime.utcnow())+"_"+self.function_name+"_"+self.component_name
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

	## Publishs the state of the action handle
	def PublishState(self):
		script_state = ScriptState()
		script_state.header.stamp = rospy.Time.now()
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
					message = "Timeout while waiting for <<%s>> to reach <<%s>>. Continuing..."%(self.component_name, self.parameter_name)
					if logging:
						rospy.logerr(message)
					self.set_failed(10, message)
					return
			# check state of action server
			#print self.client.get_state()
			if self.client.get_state() != 3:
				message = "...<<%s>> could not reach <<%s>>, aborting..."%(self.component_name, self.parameter_name)
				if logging:
					rospy.logerr(message)
				self.set_failed(11, message)
				return
			if logging:
				rospy.loginfo("...<<%s>> reached <<%s>>",self.component_name, self.parameter_name)
		else:
			message = "Execution of <<%s>> to <<%s>> was aborted, wait not possible. Continuing..."%(self.component_name, self.parameter_name)
			rospy.logwarn(message)
			self.set_failed(self.error_code, message)
			return

		self.set_succeeded() # full success

	## Cancel action
	#
	# Cancels action goal(s).
	def cancel(self):
		self.client.cancel_all_goals()

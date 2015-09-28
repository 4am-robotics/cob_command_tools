#!/usr/bin/python
#################################################################
##\file
#
# \note
#   Copyright (c) 2010 \n
#   Fraunhofer Institute for Manufacturing Engineering
#   and Automation (IPA) \n\n
#
#################################################################
#
# \note
#   Project name: care-o-bot
# \note
#   ROS stack name: cob_apps
# \note
#   ROS package name: cob_script_server
#
# \author
#   Author: Florian Weisshardt, email:florian.weisshardt@ipa.fhg.de
# \author
#   Supervised by: Florian Weisshardt, email:florian.weisshardt@ipa.fhg.de
#
# \date Date of creation: Aug 2010
#
# \brief
#   Implementation of ROS node for script_server.
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

import time
import inspect

import rospy
import actionlib

from cob_script_server.msg import *
from cob_script_server.srv import *
from simple_script_server import *

sss = simple_script_server()

## Script server class which inherits from script class.
#
# Implements actionlib interface for the script server.
#
class script_server():
	## Initializes the actionlib interface of the script server.
	#
	def __init__(self):
		self.ns_global_prefix = "/script_server"
		self.script_action_server = actionlib.SimpleActionServer(self.ns_global_prefix, ScriptAction, self.execute_cb, False)
		self.script_action_server.start()
		self.compose_trajectory_service = rospy.Service('~compose_trajectory', ComposeTrajectory, self.handle_compose_trajectory)

#------------------- Service section -------------------#
	def handle_compose_trajectory(self, req):
		print "compose trajectory", req.component_name, req.parameter_name
		traj_msg, error_code = sss.compose_trajectory(req.component_name, req.parameter_name)
		if error_code != 0:
			return None
		res = ComposeTrajectoryResponse()
		res.trajectory = traj_msg
		return res

#------------------- Actionlib section -------------------#
	## Executes actionlib callbacks.
	#
	# \param server_goal ScriptActionGoal
	#
	def execute_cb(self, server_goal):
		server_result = ScriptActionResult().result
		if server_goal.function_name == None or server_goal.function_name.strip() == "":
			rospy.logerr("function name cannot be blank")
			return

		if server_goal.function_name in dir(sss):
			func = getattr(sss, server_goal.function_name)
			argspec = inspect.getargspec(func)
			args = {}
			for arg in argspec.args:
				if arg in dir(server_goal):
					serverArg = getattr(server_goal, arg)
					if type(serverArg) == str:
						try:
							serverArg = eval(serverArg)
						except:
							pass
					args[arg] = serverArg

			handle01 = func(*(), **args)
		else:
			rospy.logerr("function <<%s>> not supported", server_goal.function_name)
			self.script_action_server.set_aborted(server_result)
			return

		if 'get_error_code' in dir(handle01):
			server_result.error_code = handle01.get_error_code()
		else:
			rospy.logwarn("unexpected action result type <<%s>> for function %s", type(handle01), server_goal.function_name)
		if server_result.error_code == 0:
			rospy.logdebug("action result success")
			self.script_action_server.set_succeeded(server_result)
		else:
			rospy.logerr("action result error, error_code: " + str(server_result.error_code))
			self.script_action_server.set_aborted(server_result)

## Main routine for running the script server
#
if __name__ == '__main__':
	rospy.init_node('script_server')
	script_server()
	rospy.loginfo("script_server is running")
	rospy.spin()

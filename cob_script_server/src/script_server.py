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

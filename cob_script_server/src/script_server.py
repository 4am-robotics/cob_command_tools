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
		if server_goal.function_name == None or server_goal.function_name.strip() == "":
			msg = "function name cannot be blank"
			rospy.logerr(msg)
			self.script_action_server.set_aborted(ScriptResult(message=msg))
			return

		if server_goal.function_name in dir(sss):
			func = getattr(sss, server_goal.function_name)
			argspec = inspect.getargspec(func)
			function_param_names = [item.key for item in server_goal.function_params]
			function_param_values = [item.value for item in server_goal.function_params]
			rospy.logdebug("argspec: {}".format(argspec))
			rospy.logdebug("dir(server_goal): {}".format(dir(server_goal)))
			rospy.logdebug("function_param_names: {}".format(function_param_names))
			rospy.logdebug("function_param_values: {}".format(function_param_values))
			args = {}
			for arg in argspec.args:
				if arg in dir(server_goal):
					serverArg = getattr(server_goal, arg)
				elif arg in function_param_names:
					serverArg = function_param_values[function_param_names.index(arg)]
				else:
					continue

				if type(serverArg) == str:
					try:
						serverArg = eval(serverArg)
					except:
						pass
				args[arg] = serverArg

			rospy.logdebug("args: {}".format(args))
			handle01 = func(*(), **args)
		else:
			msg = "function <<{}>> not supported".format(server_goal.function_name)
			rospy.logerr(msg)
			self.script_action_server.set_aborted(ScriptResult(message=msg))
			return

		if 'get_error_code' in dir(handle01):
			error_code = handle01.get_error_code()
		else:
			rospy.logwarn("unexpected action result type <<%s>> for function %s", type(handle01), server_goal.function_name)
			error_code = -1

		if "blocking" in function_param_names and not eval(function_param_values[function_param_names.index("blocking")]):
			msg = "action is non-blocking - not tracking result"
			rospy.logdebug(msg)
			self.script_action_server.set_succeeded(ScriptResult(message=msg, error_code=error_code))
		elif error_code == 0:
			msg = "action result success"
			rospy.logdebug(msg)
			self.script_action_server.set_succeeded(ScriptResult(message=msg, error_code=error_code))
		else:
			msg = "action result error, error_code: {}".format(str(error_code))
			rospy.logerr(msg)
			self.script_action_server.set_aborted(ScriptResult(message=msg, error_code=error_code))

## Main routine for running the script server
#
if __name__ == '__main__':
	rospy.init_node('script_server')
	script_server()
	rospy.loginfo("script_server is running")
	rospy.spin()

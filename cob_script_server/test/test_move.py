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
import unittest

import rospy
from simple_script_server import *
sss = simple_script_server()

## This test checks the correct call to commands from the cob_script_server. This does not cover the execution of the commands (this shoud be checked in the package where the calls refer to).
class TestMove(unittest.TestCase):
	def __init__(self, *args):
		super(TestMove, self).__init__(*args)
		rospy.init_node('test_move')
		self.cb_executed = False

	# test move base commands
	def test_move_base(self):
		self.move_base()

	def test_move_base_omni(self):
		self.move_base(mode="omni")

	def test_move_base_diff(self):
		self.move_base(mode="diff")

	def test_move_base_linear(self):
		self.move_base(mode="linear")

	def move_base(self,mode=None):
		if mode == None or mode == "" or mode == "omni":
			as_name = "/move_base"
		else:
			as_name = "/move_base_" + mode
		self.as_server = actionlib.SimpleActionServer(as_name, MoveBaseAction, execute_cb=self.base_cb, auto_start=False)
		self.as_server.start()
		self.cb_executed = False
		handle = sss.move("base",[0,0,0],mode=mode)
		if not self.cb_executed:
			self.fail('Action Server not called. script server error_code: ' + str(handle.get_error_code()))

	def base_cb(self, goal):
		self.cb_executed = True
		result = MoveBaseResult()
		self.as_server.set_succeeded(result)

	# test move trajectory commands
	def test_move_traj(self):
		component_name = "arm" # testing for component arm
		as_name = "/" + component_name + "/joint_trajectory_controller/follow_joint_trajectory"
		self.as_server = actionlib.SimpleActionServer(as_name, FollowJointTrajectoryAction, execute_cb=self.traj_cb, auto_start=False)
		self.as_server.start()
		self.cb_executed = False
		handle = sss.move(component_name,[[0,0,0,0,0,0,0]])
		if not self.cb_executed:
			self.fail('Action Server not called. script server error_code: ' + str(handle.get_error_code()))

	def traj_cb(self, goal):
		self.cb_executed = True
		result = FollowJointTrajectoryResult()
		self.as_server.set_succeeded(result)

if __name__ == '__main__':
	import rostest
	rostest.rosrun('cob_script_server', 'move', TestMove)

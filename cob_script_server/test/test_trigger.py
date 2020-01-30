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
import actionlib
from std_srvs.srv import Trigger, TriggerResponse
from move_base_msgs.msg import MoveBaseAction
from control_msgs.msg import FollowJointTrajectoryAction
from simple_script_server import *
sss = simple_script_server()

## This test checks the correct call to commands from the cob_script_server. This does not cover the execution of the commands (this shoud be checked in the package where the calls refer to).
class TestTrigger(unittest.TestCase):
	def __init__(self, *args):
		super(TestTrigger, self).__init__(*args)
		rospy.init_node('test_trigger')
		self.cb_executed = False
		self.component_name = "arm"
		self.service_ns = "/" + self.component_name + "/driver"
		rospy.set_param("/script_server/"+ self.component_name + "/service_ns", self.service_ns)
		self.action_name = "/"+ self.component_name + "/joint_trajectory_controller/follow_joint_trajectory"
		rospy.set_param("/script_server/"+ self.component_name + "/action_name", self.action_name)

	def test_init(self):
		rospy.Service(self.service_ns + "/init", Trigger, self.srv_cb)
		self.cb_executed = False
		handle = sss.init(self.component_name)
		if not self.cb_executed:
			self.fail('sss.init() failed. script server error_code: ' + str(handle.get_error_code()))

	def test_stop(self):
		as_name = "/move_base"
		self.as_server = actionlib.SimpleActionServer(as_name, MoveBaseAction, execute_cb=self.action_cb)
		handle = sss.stop("base")
		if not handle.get_error_code() == 0:
			self.fail('sss.stop("base") failed. script server error_code: ' + str(handle.get_error_code()))
		handle = sss.stop("base", mode="omni")
		if not handle.get_error_code() == 0:
			self.fail('sss.stop("base", mode="omni") failed. script server error_code: ' + str(handle.get_error_code()))

		as_name = "/move_base_diff"
		self.as_server = actionlib.SimpleActionServer(as_name, MoveBaseAction, execute_cb=self.action_cb)
		handle = sss.stop("base", mode="diff")
		if not handle.get_error_code() == 0:
			self.fail('sss.stop("base", mode="diff") failed. script server error_code: ' + str(handle.get_error_code()))

		as_name = "/move_base_linear"
		self.as_server = actionlib.SimpleActionServer(as_name, MoveBaseAction, execute_cb=self.action_cb)
		handle = sss.stop("base", mode="linear")
		if not handle.get_error_code() == 0:
			self.fail('sss.stop("base", mode="linear") failed. script server error_code: ' + str(handle.get_error_code()))

		self.as_server = actionlib.SimpleActionServer(self.action_name, FollowJointTrajectoryAction, execute_cb=self.action_cb)
		handle = sss.stop(self.component_name)
		if not handle.get_error_code() == 0:
			self.fail('sss.stop("arm") failed. script server error_code: ' + str(handle.get_error_code()))

	def test_recover(self):
		rospy.Service(self.service_ns + "/recover", Trigger, self.srv_cb)
		self.cb_executed = False
		handle = sss.recover(self.component_name)
		if not self.cb_executed:
			self.fail('sss.recover() failed. script server error_code: ' + str(handle.get_error_code()))

	def test_halt(self):
		rospy.Service(self.service_ns + "/halt", Trigger, self.srv_cb)
		self.cb_executed = False
		handle = sss.halt(self.component_name)
		if not self.cb_executed:
			self.fail('sss.halt() failed. script server error_code: ' + str(handle.get_error_code()))

	def srv_cb(self,req):
		self.cb_executed = True
		res = TriggerResponse()
		res.success = True
		return res

	def action_cb(self, goal):
		while not self.as_server.is_preempt_requested():
			rospy.Rate(1).sleep()
		self.as_server.set_preempted()

if __name__ == '__main__':
	import rostest
	rostest.rosrun('cob_script_server', 'trigger', TestTrigger)

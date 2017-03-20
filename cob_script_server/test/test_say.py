#!/usr/bin/env python

import sys
import unittest

import rospy
import rostest

from simple_script_server import *
sss = simple_script_server()

class SayTest(unittest.TestCase):
	def __init__(self, *args):
		super(SayTest, self).__init__(*args)
		rospy.init_node('test_say_test')
		self.cb_executed = False

	def test_say(self):
		as_name = "/sound/say"
		self.as_server = actionlib.SimpleActionServer(as_name, SayAction, execute_cb=self.say_cb, auto_start=False)
		self.as_server.start()
		self.cb_executed = False
		handle = sss.say("sound", ["hello"])
		if not self.cb_executed:
			self.fail('Service Server not called. script server error_code: ' + str(handle.get_error_code()))

	def say_cb(self, goal):
		self.cb_executed = True
		result = SayResult()
		self.as_server.set_succeeded(result)

if __name__ == '__main__':
	try:
		rostest.run('rostest', 'test_say_test', SayTest, sys.argv)
	except KeyboardInterrupt, e:
		pass
	print "exiting"


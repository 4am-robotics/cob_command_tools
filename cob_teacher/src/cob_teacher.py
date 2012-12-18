#!/usr/bin/env python
import roslib; roslib.load_manifest('cob_teacher')
import rospy

import yaml
import sys
import time
import tf

from sensor_msgs.msg import JointState

class ros_helpers:
	def __init__(self):
		rospy.init_node('cob_teacher', anonymous=True)
		rospy.Subscriber("/joint_states", JointState, self.js_cb)
		self.listener = tf.TransformListener()
		time.sleep(0.1)
		self.arm_config = []
		self.torso_config = []
		self.head_config = []
		self.gripper_config = []

	def js_cb(self):
		pass

	def getCurrentConfig(self, robot_part):
		if(robot_part == "arm"):
			if len(self.arm_config) != 0:
				return self.arm_config
		if(robot_part == "torso"):
			if len(self.torso_config) != 0:
				return self.torso_config
		if(robot_part == "head"):
			if len(self.head_config) != 0:
				return self.head_config
		if(robot_part == "gripper"):
			if len(self.gripper_config) != 0:
				return self.gripper_config
		return []

	def getCurrentPose(self, robot_part):
		target_frame = "/base_link"
		if(robot_part == "base"):
			target_frame = "/base_link"
		try:
            (trans,rot) = self.listener.lookupTransform('/map', target_frame, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        #tranform rot to rpy
        roll, pitch, yaw = tf.transformations.euler_from_quaternion(rot)
        return [trans[0], trans[1], yaw]

class yaml_manager:
	def __init__(self, script_package):
		self.stream = open(script_package, "rw")

	def get_poses(self, robot_part):
		positions = yaml.safe_load(self.stream)
		for a in  positions:
			#TODO: check for joint names and trajectories
			print a, " = ", positions[a]

	def add_pose(self, robot_part, name):
		positions = yaml.safe_load(self.stream)
		positions[name] = [[0,0,0,0,0,0,0]]
		outstream = yaml.dump(positions)
		print outstream
		self.stream.write(str(outstream))


def usage():
	print "cob_teacher [script_package] safe_config [arm, torso] name"
	print "cob_teacher [script_package] list_config [arm, torso]"
	print ""
	print "cob_teacher [script_package] safe_pose [base] name"
	print "cob_teacher [script_package] list_pose [base]"
	print ""
	print "cob_teacher [script_package] update"

if __name__ == "__main__":
	pass

def test():
	if len(sys.argv) <= 2:
		usage()
	else:
		script_package = sys.argv[1]
		command = sys.argv[2]
		p = yaml_manager(script_package)
		
		print script_package, command
		if(command == "list_config"):
			if(len(sys.argv) != 4):
				usage()
			else:
				p.get_poses(sys.argv[3])
		if(command == "safe_config"):
			if(len(sys.argv) != 5):
				usage()
			else:
				p.add_pose(sys.argv[3], sys.argv[4])


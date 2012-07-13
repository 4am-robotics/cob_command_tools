#!/usr/bin/python

import roslib
roslib.load_manifest('cob_script_server')
import rospy

from simple_script_server import *
from kinematics_msgs.srv import *
from tf.transformations import *

class TestScript(script):
		
	def Initialize(self):
		self.sss.init("arm")
		
	def Run(self):
		
		#optional: start position of arm
		pos_start_joint = [-0.0134, 0.49, -0.01, -1.88, 0.0, 0.0, 0.0] #joint space 
		
		#end position of arm
		pos_end = [-0.494, -0.772, 1.414] 
		quat_end = [0.291, -0.539, 0.593, 0.522]
		
		
		err_code = self.sss.check_plan('arm',[[list(pos_end),list(quat_end)], pos_start_joint])
		##alternative function call using the actual position to start plan
		#err_code = self.sss.check_plan('arm',[[list(pos_end),list(quat_end)]])
		
		
		#print if valid motion plan available
		if err_code[1].val == err_code[1].SUCCESS:
			rospy.loginfo("Valid plan found.")
		else:
			rospy.logerr("No valid plan for given states.")
	
if __name__ == "__main__":
	SCRIPT = TestScript()
	SCRIPT.Start()


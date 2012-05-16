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
		pos_start = [-0.51, 0.14, 0.97]
		quat_start= [0.32, -0.64, 0.29, 0.63]  
		  
		#end position of arm
		pos_end = [-0.494, -0.772, 1.414] #[-0.494, -0.772, 1.414]
		quat_end = [0.291, -0.539, 0.593, 0.522]
		
		#ik of positions
		start_pos = self.sss.calculate_ik(['base_footprint',pos_start, euler_from_quaternion(quat_start)])
		end_pos = self.sss.calculate_ik(['base_footprint',pos_end, euler_from_quaternion(quat_end)])
		
		#check if ik was successful
		if (start_pos[1].val==1 and end_pos[1].val==1):
			err_code = self.sss.check_plan('arm',[list(end_pos),list(start_pos)])
			
			#print if valid motion plan available
			if err_code[1].val == 1:
				rospy.loginfo("Valid plan found.")
			else:
				rospy.logerr("No valid plan for given states.")
		else:
			rospy.logerr("IK was not successful. Check start or end position!")
		
	
if __name__ == "__main__":
	SCRIPT = TestScript()
	SCRIPT.Start()


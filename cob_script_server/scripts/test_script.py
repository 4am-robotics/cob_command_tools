#!/usr/bin/python

import roslib
roslib.load_manifest('cob_script_server')
import rospy

from simple_script_server import *
sss = simple_script_server()
		
if __name__ == "__main__":
	rospy.init_node("asd")
	
	sss.move("torso","home")

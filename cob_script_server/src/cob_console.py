#!/usr/bin/python

import roslib
roslib.load_manifest('cob_script_server')
import rospy

from simple_script_server import *
sss = simple_script_server()

### start this with
# rosrun cob_script_server cob_console
	
if __name__ == "__main__":
	rospy.init_node("cob_console")

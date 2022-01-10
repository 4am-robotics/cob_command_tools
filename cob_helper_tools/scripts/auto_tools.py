#!/usr/bin/env python

import rospy
from cob_auto_tools.auto_init import AutoInit
from cob_auto_tools.auto_recover import AutoRecover

if __name__ == "__main__":
  rospy.init_node("auto_tools")
  rospy.loginfo("auto_tools running")
  
  if rospy.get_param('~enable_auto_init', True):
    AI = AutoInit()
  
  if rospy.get_param('~enable_auto_recover', True):
    AR = AutoRecover()
  
  rospy.spin()

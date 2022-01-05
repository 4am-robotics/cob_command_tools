#!/usr/bin/env python

import rospy
from cob_helper_tools.auto_init import AutoInit
from cob_helper_tools.auto_recover import AutoRecover

if __name__ == "__main__":
  rospy.init_node("auto_tools")
  rospy.loginfo("auto_tools running")
  AI = AutoInit()
  AR = AutoRecover()
  rospy.spin()

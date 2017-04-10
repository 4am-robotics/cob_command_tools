#!/usr/bin/python
#################################################################
##\file
#
# \note
#   Copyright (c) Felix Messmer \n
#   Fraunhofer Institute for Manufacturing Engineering
#   and Automation (IPA) \n
#
#   All rights reserved. \n\n
#
#################################################################
#
# \note
#   Repository name: cob_command_tools
# \note
#   ROS package name: cob_helper_tools
#
# \author
#   Author: Felix Messmer
#
# \date Date of creation: January 2017
#
# \brief
#   A script to automatically initialize hardware on startup
#
#################################################################

import rospy
import tf

from cob_msgs.msg import EmergencyStopState

from simple_script_server import *
sss = simple_script_server()

class AutoInit():

  def __init__(self):
    self.components = rospy.get_param('~components', [])
    self.em_state = 1  # assume EMSTOP
    rospy.Subscriber("/emergency_stop_state", EmergencyStopState, self.em_cb)

    # wait for all components to start
    for component in self.components:
      rospy.loginfo("[auto_init]: Waiting for %s to start...", component)
      rospy.wait_for_service("/" + component + "/driver/init")

    # wait for emergency_stop to be released
    while not rospy.is_shutdown():
      if self.em_state == 1: # EMSTOP
        rospy.loginfo("[auto_init]: Waiting for emergency stop to be released...")
        try:
          rospy.sleep(1)
        except rospy.exceptions.ROSInterruptException as e:
          pass
      else: # EMFREE or EMCONFIRMED
        # call init for all components
        rospy.loginfo("[auto_init]: Initializing components")
        for component in self.components:
          while not rospy.is_shutdown():
            handle = sss.init(component)
            if not (handle.get_error_code() == 0):
              rospy.logerr("[auto_init]: Could not initialize %s. Retrying...", component)
            else:
              rospy.loginfo("[auto_init]: Component %s initialized successfully", component)
              break
        break # done

  def em_cb(self, msg):
    self.em_state = msg.emergency_state

if __name__ == "__main__":
  rospy.init_node("auto_init")
  rospy.loginfo("auto init running")
  AI = AutoInit()
  rospy.loginfo("auto init finished")

#!/usr/bin/python

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
      rospy.loginfo("waiting for %s to start", component)
      rospy.wait_for_service("/" + component + "/driver/init")
    
    # wait for emergency_stop to be released
    while not rospy.is_shutdown():
      if self.em_state == 1: # EMSTOP
        rospy.loginfo("auto_init: waiting for emergency stop to be released")
        try:
          rospy.sleep(1)
        except rospy.exceptions.ROSInterruptException as e:
          pass
      else: # EMFREE or EMCONFIRMED
        # call init for all components
        rospy.loginfo("auto_init: initializing components")
        for component in self.components:
          sss.init(component)
        break # done

  def em_cb(self, msg):
    self.em_state = msg.emergency_state

if __name__ == "__main__":
  rospy.init_node("auto_init")
  rospy.loginfo("auto init running")
  AI = AutoInit()
  rospy.loginfo("auto init finished")

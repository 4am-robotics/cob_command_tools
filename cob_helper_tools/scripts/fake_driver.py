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
#   A script providing services to fake a (ros_canopen) driver
#
#################################################################

import rospy
from std_srvs.srv import *

class FakeDriver():

    def __init__(self):
        self.init_srv = rospy.Service('driver/init', Trigger, self.srv_cb)
        self.recover_srv = rospy.Service('driver/recover', Trigger, self.srv_cb)
        self.halt_srv = rospy.Service('driver/halt', Trigger, self.srv_cb)
        self.shutdown_srv = rospy.Service('driver/shutdown', Trigger, self.srv_cb)

    def srv_cb(self, req):
        resp = TriggerResponse()
        resp.success = True
        return resp


if __name__ == "__main__":
   rospy.init_node('fake_driver')
   FakeDriver()
   rospy.loginfo("fake_driver running")
   rospy.spin()


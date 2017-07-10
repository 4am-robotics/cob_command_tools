#!/usr/bin/python

import rospy
from generic_throttle.generic_throttle import GenericThrottle

if __name__ == "__main__":
    rospy.init_node('generic_throttle_node', anonymous=True)
    gt = GenericThrottle()
    rospy.spin()

#! /usr/bin/env python

import rospy

from scenario_test_tools.scriptable_move_base import ScriptableMoveBase
from move_base_msgs.msg import MoveBaseAction, MoveBaseResult

if __name__ == "__main__":
    rospy.init_node('always_succeeding_move_base')

    # By default, wait 5 secs for the robot to arrive
    result_delay = rospy.get_param("result_delay", 5)

    succeeded = MoveBaseResult()
    move_base = ScriptableMoveBase('~move_base',
                                   MoveBaseAction,
                                   default_result_delay=result_delay,
                                   default_result=succeeded)

    move_base.start()
    rospy.loginfo("always_succeeding_move_base running")

    rospy.spin()
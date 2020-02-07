#!/usr/bin/env python
import rospy

from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseResult

from scenario_test_tools.scriptable_move_base import ScriptableMoveBase

if __name__ == "__main__":
    rospy.init_node('move_base_fake')
    result_delay = rospy.get_param("result_delay")
    move_base = ScriptableMoveBase('/move_base_linear', MoveBaseAction,
                                        result_delay=result_delay,
                                        default_result=MoveBaseResult())
    move_base.start()
    rospy.loginfo("fake move base running")
    rospy.spin()

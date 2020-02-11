#! /usr/bin/env python

import rospy

from scenario_test_tools.scriptable_move_base import ScriptableMoveBase

from move_base_msgs.msg import MoveBaseAction, MoveBaseResult


if __name__ == "__main__":
    rospy.init_node('move_base_fake')

    succeeded = MoveBaseResult()

    # By default, wait 5 secs for the robot to arrive
    result_delay = rospy.get_param("result_delay", 5)

    move_base = ScriptableMoveBase(rospy.get_name(), MoveBaseAction, default_result_delay=result_delay)
    move_base.start()
    rospy.loginfo("fake move base running")

    try:
        move_base.reply(succeeded, marker='1: succeeded', reply_delay=3, timeout=60)

        # To check that the client will perhaps retry the navigation or otherwise handle an abort
        move_base.reply(move_base.ABORT_GOAL, marker='2: Aborted')

        # The client will have to decide it's taking too long and cancel
        move_base.reply(move_base.IGNORE_GOAL, marker='3: Ignored', reply_delay=0)
    except AssertionError as assertion_err:
        rospy.logerr(assertion_err)
        raise assertion_err
    rospy.spin()
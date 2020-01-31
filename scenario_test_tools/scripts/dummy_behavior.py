#!/usr/bin/python

import random
import rospy
import actionlib

from std_msgs.msg import String, Header
from cob_sound.msg import SayAction, SayGoal
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal, FollowJointTrajectoryResult
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from cob_srvs.srv import SetString, SetStringRequest
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion

charger_pose = PoseStamped(header=Header(frame_id='map'),
                           pose=Pose(position=Point(10, 10, 0),
                                     orientation=Quaternion(1, 0, 0, 0)))

charger_arm_traj = FollowJointTrajectoryGoal(trajectory=JointTrajectory(points=[JointTrajectoryPoint(positions=[0])]))

if __name__ == "__main__":
    rospy.init_node("dummy_behavior")

    move_base_ac = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    say_ac = actionlib.SimpleActionClient('say', SayAction)
    move_arm_ac = actionlib.SimpleActionClient('/arm/joint_trajectory_controller/follow_joint_trajectory',
                                            FollowJointTrajectoryAction)
    dock_srv = rospy.ServiceProxy('/dock', SetString)

    def command_callback(msg):
        if 'charge' in msg.data:
            rospy.loginfo("I'm told to go charge, lets go")
            move_base_ac.send_goal_and_wait(MoveBaseGoal(target_pose=charger_pose))

            rospy.loginfo("I'm going to dock")
            dock_result = dock_srv(SetStringRequest('charger'))

            if not dock_result.success:
                rospy.logwarn("I borked docking, let's try again")
                say_ac.send_goal_and_wait(SayGoal('Docking has failed, lets try again'))

                dock_srv(SetStringRequest('charger'))
                #  And now let's hope it does succeed

            rospy.loginfo("Let's use my arm to plug myself in")
            move_arm_ac.send_goal_and_wait(charger_arm_traj)
            arm_successful = move_arm_ac.get_result().error_code == FollowJointTrajectoryResult.SUCCESSFUL
            if not arm_successful:
                rospy.logwarn("I borked using my arm, let's try again")
                say_ac.send_goal_and_wait(SayGoal('My arm has failed, lets try again'))
                move_arm_ac.send_goal_and_wait(charger_arm_traj)
                # Again, lets assume it works this time

            rospy.loginfo("Ah, finally charging, lets that juice flow in")
            sentence = random.choice(["Jummy, fresh juice!", "Ah, some juice!"])
            say_ac.send_goal_and_wait(SayGoal(sentence))

            rospy.loginfo("Test succeeded, I'm done")

    command_subscriber = rospy.Subscriber('/command', String, command_callback)

    rospy.loginfo("I'm waiting for your command, e.g. to go charge")

    rospy.spin()
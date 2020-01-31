#!/usr/bin/python

import rospy
import traceback
import pdb

import tf
from std_msgs.msg import String
from cob_sound.msg import SayAction, SayResult
from move_base_msgs.msg import MoveBaseAction, MoveBaseResult, MoveBaseGoal
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryResult, FollowJointTrajectoryGoal
from cob_srvs.srv import SetString, SetStringRequest, SetStringResponse

from scenario_test_tools.scriptable_move_base import ScriptableMoveBase
from scenario_test_tools.scriptable_action_server import ScriptableActionServer
from scenario_test_tools.scriptable_service_server import ScriptableServiceServer
from scenario_test_tools.util import countdown_sleep, round_tuple


class bcolors:
    HEADER = '\033[95m'

    BLACK = '\033[30m'
    RED = '\033[31m'
    GREEN = '\033[32m'
    YELLOW = '\033[33m'
    BLUE = '\033[34m'
    MAGENTA = '\033[35m'
    CYAN = '\033[36m'
    WHITE = '\033[37m'

    BLACK_BG = '\033[40m'
    RED_BG = '\033[41m'
    GREEN_BG = '\033[42m'
    YELLOW_BG = '\033[43m'
    BLUE_BG = '\033[44m'
    MAGENTA_BG = '\033[45m'
    CYAN_BG = '\033[46m'
    WHITE_BG = '\033[47m'

    ENDC = '\033[0m'
    UNDERLINE = '\033[4m'


def format_move_base_goal(mbg):
    roll, pitch, yaw = tf.transformations.euler_from_quaternion(
        [mbg.target_pose.pose.orientation.x, mbg.target_pose.pose.orientation.y, mbg.target_pose.pose.orientation.z,
         mbg.target_pose.pose.orientation.w])
    return bcolors.MAGENTA + bcolors.UNDERLINE + "MoveBase to: (x={x}, y={y}, yaw={yaw})".format(
        x=mbg.target_pose.pose.position.x,
        y=mbg.target_pose.pose.position.y,
        yaw=yaw) + bcolors.ENDC


class TestScenario(object):
    def __init__(self, debug_exceptions=False):
        self.debug_exceptions = debug_exceptions

        self.human_to_robot_speech = rospy.Publisher('/command', String, queue_size=1)

        self.say = ScriptableActionServer('say', SayAction,
                                          # The default reply allows to not bother with every wishy-washy things
                                          # the robot needs to say, eg. in cases where success is expected (like TTS)
                                          default_result=SayResult(success=True),
                                          goal_formatter=lambda goal: bcolors.YELLOW + bcolors.UNDERLINE +
                                                                      "text: '{}'".format(goal.text) +
                                                                      bcolors.ENDC,
                                          result_formatter=lambda result: bcolors.YELLOW +
                                                                          "success: '{}'".format(result.success) +
                                                                          bcolors.ENDC)

        self.move_base = ScriptableMoveBase('/move_base', MoveBaseAction,
                                          result_delay=5,
                                          goal_formatter=format_move_base_goal,
                                          result_formatter=lambda result: bcolors.MAGENTA +
                                                                          "MoveBase: done".format(result) +
                                                                          bcolors.ENDC
                                          )

        self.arm = ScriptableActionServer('/arm/joint_trajectory_controller/follow_joint_trajectory',
                                          FollowJointTrajectoryAction,
                                          result_delay=1,
                                          goal_formatter=lambda goal: bcolors.CYAN + bcolors.UNDERLINE +
                                                                      "Moving to {}".format(
                                                                          goal.trajectory.points[-1].positions) +
                                                                      bcolors.ENDC,
                                          result_formatter=lambda result: bcolors.CYAN +
                                                                          "Arm: done".format(result.error_code) +
                                                                          bcolors.ENDC)

        self.dock = ScriptableServiceServer('/dock', SetString,
                                            response_delay=5,
                                            request_formatter=lambda
                                          req: bcolors.YELLOW_BG + bcolors.BLACK + bcolors.UNDERLINE +
                                               "Docking: {}".format(req.data) +
                                               bcolors.ENDC,
                                            response_formatter=lambda res: bcolors.YELLOW_BG + bcolors.BLACK +
                                                                     "Dock: {}".format(
                                                                         "Succeeded " if res.success else "Failed") +
                                                                     bcolors.ENDC)

        self.say.start()
        self.move_base.start()
        self.arm.start()

        self._start_scenario_srv = rospy.Service('start_scenario', SetString, self._start_scenario)

        print("The following tests are available: " + ', '.join(self._available_test_methods))
        rospy.sleep(1)

    @property
    def _available_test_methods(self):
        return [member for member in dir(self) if member.startswith('test_')]

    def _start_scenario(self, request):
        res = SetStringResponse()
        res.success, res.message = self.start_scenario(request.data)
        return res

    def start_scenario(self, scenario):
        if scenario not in self._available_test_methods:
            return False, "Not a valid test scenario"
        else:
            func = getattr(self, scenario)
            try:
                print("Starting scenario: {}".format(scenario))
                func()
                return True, "Scenario '{}' passed successfully".format(scenario)
            except Exception as e:
                print(bcolors.RED + "----- error: ------")
                print(e)
                traceback.format_exc()
                if self.debug_exceptions:
                    pdb.post_mortem()
                return False, str(e)

    def stop(self):
        print("Stopping all test helpers")
        self.move_base.stop()
        self.arm.stop()
        self.say.stop()
        self.dock.stop()

    def test_happy_flow(self):
        """
        The robot is told to go charge.
        It should then navigate to a position close to the charger,
        call the dock-service then uses it's arm to plug itself in and say something when
        finally getting some juice from the charger.
        """
        self.human_to_robot_speech.publish('Go charge')

        nav_goal = (10, 10, 0)  # Just a random goal in x, y and orientation

        move_base_goal = self.move_base.await_goal(marker='start_move_to_charger')
        navigated_to_goal = any(self.move_base.match_in_received_goals([nav_goal], key=self.move_base.goal_to_xy_theta))
        already_at_goal = self.move_base.pose_bl == nav_goal

        assert navigated_to_goal or already_at_goal, "Robot is not at goal for service"

        self.move_base.direct_reply(MoveBaseResult(), marker='end_move_to_charger')

        self.dock.reply(SetStringResponse(success=True), marker='dock_into_charger')
        self.arm.reply(FollowJointTrajectoryResult(error_code=FollowJointTrajectoryResult.SUCCESSFUL), marker='arm_plug')

        while not self.say.match_in_received_goals(["Ah, some juice!", "Jummy, fresh juice!"],
                                                   key=lambda req: req.text) and not rospy.is_shutdown():
            rospy.sleep(0.5)

        rospy.loginfo("test_happy_flow succeeded")

    def test_unhappy_flow(self):
        """
        The robot is told to go charge.
        It should then navigate to a position close to the charger.
        call the dock-service then uses it's arm to plug itself in and say something when
        finally getting some juice from the charger.

        However, the docking fails first so the robot retries and then the arm fails at first,
        so the robot says something about it's arm and then retries
        """
        self.human_to_robot_speech.publish('Go charge')

        nav_goal = (10, 10, 0)  # Just a random goal in x, y and orientation

        move_base_goal = self.move_base.await_goal()
        navigated_to_goal = any(self.move_base.match_in_received_goals([nav_goal], key=self.move_base.goal_to_xy_theta))
        already_at_goal = self.move_base.pose_bl == nav_goal

        assert navigated_to_goal or already_at_goal, "Robot is not at goal for service"

        self.move_base.direct_reply(MoveBaseResult())

        # Using a custom reply, we can override the default reply set on the dock service temporarily
        with self.dock.custom_reply():
            self.dock.reply(SetStringResponse(success=False))

            # the robot notices docking has failed, so it should say something about that
            say_goal = self.say.await_goal()
            assert 'fail' in say_goal.text
            self.say.direct_reply(SayResult(success=True))

            # Then retry docking, now successfully
            self.dock.reply(SetStringResponse(success=True))

        with self.arm.custom_reply():
            self.arm.reply(FollowJointTrajectoryResult(error_code=FollowJointTrajectoryResult.PATH_TOLERANCE_VIOLATED))

            # the robot notices docking has failed, so it should say something about that
            say_goal = self.say.await_goal()
            assert 'fail' in say_goal.text
            self.say.direct_reply(SayResult(success=True))

            self.arm.reply(FollowJointTrajectoryResult(error_code=FollowJointTrajectoryResult.SUCCESSFUL))

        while not self.say.match_in_received_goals(["Ah, some juice!", "Jummy, fresh juice!"],
                                                   key=lambda req: req.text) and not rospy.is_shutdown():
            rospy.sleep(0.5)

        rospy.loginfo("test_unhappy_flow succeeded")

    def test_all(self):
        if not rospy.is_shutdown():
            with self.say.remember_goals():
                self.test_happy_flow()

        if not rospy.is_shutdown():
            with self.say.remember_goals():
                self.test_unhappy_flow()


if __name__ == "__main__":
    rospy.init_node("example_test")

    ts = TestScenario()

    rospy.on_shutdown(ts.stop)

    import sys
    import argparse

    rospy.myargv(argv=sys.argv)  # Remove ROS remappings
    parser = argparse.ArgumentParser()
    parser.add_argument("--start_scenario", help="Which test scenario should be started?")
    parser.add_argument("--keep_alive", action='store_true', help="Let servers run after test finished?")
    parser.add_argument("--debug", action='store_true', help="Run pdb.post_mortem when a test hits an exception")
    args = parser.parse_args()

    if args.debug:
        ts.debug_exceptions = True

    if args.start_scenario:
        start_scenario = args.start_scenario
        success, message = ts.start_scenario(args.start_scenario)

    if not args.keep_alive:
        rospy.signal_shutdown("End of test")

        ts.stop()
        exit()
    else:
        rospy.logwarn(
            "Test has finished but node keeps running because --keep_alive was passed")
        rospy.spin()

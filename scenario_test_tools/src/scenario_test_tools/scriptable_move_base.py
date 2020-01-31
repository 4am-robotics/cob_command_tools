#!/usr/bin/env python
#
# Copyright 2020 Mojin Robotics GmbH
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import math

import rospy
import tf

from scenario_test_tools.scriptable_action_server import ScriptableActionServer
from scenario_test_tools.util import countdown_sleep, round_tuple


class ScriptableMoveBase(ScriptableActionServer):
    def __init__(self, name, action_type, goal_formatter=format, result_formatter=format, default_result=None, result_delay=0):
        ScriptableActionServer.__init__(self,
                                        name=name,
                                        action_type=action_type,
                                        goal_formatter=goal_formatter,
                                        result_formatter=result_formatter,
                                        default_result=default_result,
                                        result_delay=result_delay)

        self.br = tf.TransformBroadcaster()
        self.pose_bl = [0, 0, 0]
        self._base_link_timer = rospy.Timer(rospy.Duration(0.1), self._pub_transform_bl)

    def stop(self):
        self._base_link_timer.shutdown()
        super(ScriptableMoveBase, self).stop()

    def _execute_cb(self):
        """
        Called when the underlying action server receives a goal.
        If the default_result is None, it will wait for a custom result to be set via reply* otherwise
        return the default_result after the given result_delay

        In the reply-case,it then notifies self.reply* (which should be called by a test script outside this class),
        after which self.reply* determines the result to the goal.
        Then it notifies _execute_cb that the result has been determined so that _execute_cb can send it
        """
        print('\n########  {}.execute  ###########'.format(self._name))

        self._current_goal = self._as.accept_new_goal()
        self._received_goals.append(self._current_goal)

        try:
            goal_str = self.goal_formatter(self._current_goal)
        except Exception as e:
            rospy.logerr("goal_formatter of {} raised an exception: {}".format(self._name, e))
            goal_str = self._current_goal
        print('{}.execute: From {} to Goal: {}'.format(self._name, self.pose_bl, goal_str))

        if self.default_reply is not None:
            countdown_sleep(self._reply_delay, text="{}.execute: Wait for {}s. ".format(self._name, self._reply_delay) + "Remaining {}s...")
            self._call_pre_reply_callbacks(self._current_goal, self.default_reply)
            self._send_result(self.default_reply)
        else:
            self._request.set()
            # Now, wait for  to be called, which sets the _reply event AND the _next_reply
            self._reply.wait()
            self._reply.clear()

            self._call_pre_reply_callbacks(self._current_goal, self._next_reply)
            self._send_result(self._next_reply)

            self._next_reply = None
            self._current_goal = None
            self._sent.set()

    def _send_result(self, result):
        if result is not self.IGNORE_GOAL and result is not self.ABORT_GOAL:
            try:
                result_str = self.result_formatter(result)
            except Exception as e:
                result_str = result
            print("{}.execute: Result: {}".format(self._name, result_str))
            self.pose_bl = self.goal_to_xy_theta(self.current_goal)
            self._as.set_succeeded(result)
        elif result == self.ABORT_GOAL:
            print("{}.execute: Result: {}".format(self._name, result))
            self._as.set_aborted()
        elif result == self.IGNORE_GOAL:
            print("{}.execute: Result: {}".format(self._name, result))
            # Do not send a reply at all, to see how the client deals with it
            pass

    def _pub_transform_bl(self, event):
        """
        Publish the transform from /map to /base_link
        :param event: timer event
        """
        self.br.sendTransform((self.pose_bl[0], self.pose_bl[1], 0),
                              tf.transformations.quaternion_from_euler(0, 0, self.pose_bl[2]),
                              rospy.Time.now(),
                              "/base_link",
                              "/map")

    def distance(self, pose1, pose2):
        """
        Calculate the distance between two x, y, theta arrays

        :param pose1: Position to calculate distance from
        :type pose1: [x, y, theta]
        :param pose2: Position to calculate distance to
        :type pose2: [x, y, theta]
        :return: cartesian distance between the positions
        :rtype float
        """
        if (len(pose1) == len(pose2) and len(pose1) == 3):
            dist = math.sqrt(pow(pose2[0] - pose1[0], 2) + pow(pose2[1] - pose1[1], 2))
        else:
            dist = 0.0
        return dist

    @staticmethod
    def goal_to_xy_theta(goal):
        """
        Convert a MoveBaseGoal to a much condensed representation: a list of [x, y, theta]

        :param goal: A moveBaseGoal
        :return: a compact representation of the goal
        :rtype List[float]
        """
        roll, pitch, yaw = tf.transformations.euler_from_quaternion(
            [goal.target_pose.pose.orientation.x,
             goal.target_pose.pose.orientation.y,
             goal.target_pose.pose.orientation.z,
             goal.target_pose.pose.orientation.w])
        return round_tuple((goal.target_pose.pose.position.x,
                goal.target_pose.pose.position.y,
                yaw), 1)


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

import actionlib
import rospy

from scenario_test_tools.scriptable_base import ScriptableBase
from scenario_test_tools.util import countdown_sleep


class ScriptableActionServer(ScriptableBase):
    """
    ScriptableActionServer allows its users to determine the ActionResult to an ActionGoal.

    The result-type for e.g. FooAction should be FooResult, not FooActionResult!

    Goals can be aborted by setting `ABORT_GOAL` as the result.

    Goals can be ignored by setting `IGNORE_GOAL` as the result. This makes the action client never get a result

    Note that Actionlib uses the term 'result' for its, well, results, whereas ROS services use 'reply'.
    The base class `ScriptableBase` uses the 'reply' terminology.
    `ScriptableActionServer` should be consistent with the action terminology,
    so it's constructor does some translation between terms and passes 'default_reply=default_result'
    to the constructor of `ScriptableBase`.
    """

    IGNORE_GOAL = "IGNORE_GOAL"
    ABORT_GOAL = "ABORT_GOAL"

    def __init__(self, name, action_type, goal_formatter=format, result_formatter=format, default_result=None, default_result_delay=0):
        """
        Set up a ScriptableActionServer based on the name and the type of Action it should implement

        :param action_type: action type (e.g. MoveBaseAction)
        """
        ScriptableBase.__init__(self, name,
                                goal_formatter=goal_formatter,
                                reply_formatter=result_formatter,
                                default_reply=default_result,
                                default_reply_delay=default_result_delay)

        self._as = actionlib.SimpleActionServer(name, action_type, auto_start=False)
        self._as.register_goal_callback(self._execute_cb)
        self._as.register_preempt_callback(self._preempt_cb)

    def start(self):
        """
        Start the action server and thus listen to goals
        """
        self._as.start()

    def stop(self):
        self._as.action_server.started = False
        super(ScriptableActionServer, self).stop()

    def _execute_cb(self):
        """
        Called when the underlying action server receives a goal.
        If the default_result is None, it will wait for a custom result to be set via reply* otherwise
        return the default_result after the given default_result_delay

        In the reply-case,it then notifies self.reply* (which should be called by a test script outside this class),
        after which self.reply* determines the result to the goal.
        Then it notifies _execute_cb that the result has been determined so that _execute_cb can send it
        """

        self._current_goal = self._as.accept_new_goal()
        self._received_goals += [self._current_goal]
        try:
            goal_str = self.goal_formatter(self._current_goal)
        except Exception as e:
            rospy.logerr("goal_formatter of {} raised an exception: {}".format(self._name, e))
            goal_str = self._current_goal
        print('{}.execute: Goal: {}'.format(self._name, goal_str))

        if self.default_reply is not None:
            countdown_sleep(self._default_reply_delay, text="{}.execute: Wait for {}s. ".format(self._name, self._default_reply_delay) + "Remaining {}s...")
            self._call_pre_reply_callbacks(self._current_goal, self.default_reply)
            self._send_result(self.default_reply)
        else:
            self._request.set()
            # Now, wait for action to be called, which sets the _reply event AND the _next_reply
            self._reply.wait()
            self._reply.clear()

            self._call_pre_reply_callbacks(self._current_goal, self._next_reply)
            self._send_result(self._next_reply)

            self._next_reply = None
            self._current_goal = None
            self._sent.set()

    def _send_result(self, result):
        """
        Send the result and deal with ignored and aborted goals

        :param result: a Result associated with the Action-type of this Server
        """
        if result is not self.IGNORE_GOAL and result is not self.ABORT_GOAL:
            try:
                result_str = self.result_formatter(result)
            except Exception as e:
                rospy.logerr("result_formatter of {} raised an exception: {}".format(self._name, e))
                result_str = self._current_goal
            print("{}.execute: Result: {}".format(self._name, result_str))
            self._as.set_succeeded(result)
        elif result == self.ABORT_GOAL:
            print("{}.execute: Result: {}".format(self._name, result))
            self._as.set_aborted()
        elif result == self.IGNORE_GOAL:
            print("{}.execute: Result: {}".format(self._name, result))
            # Do not send a reply at all, to see how the client deals with it
            pass

    def _preempt_cb(self):
        """
        Handle action preemption by the action client
        """
        if self._as.is_active():
            self.preemption = True
        self._as.set_preempted()

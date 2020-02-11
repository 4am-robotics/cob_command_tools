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

from threading import Event
import contextlib

from scenario_test_tools.util import countdown_sleep
import collections


class ScriptableBase(object):
    """
    ScriptableBase is a superclass that has common functionality for Scriptable... callbacks.

    It allows to set a default reply that can be overridden if need be, so custom replies can be defined
    """

    def __init__(self, name, goal_formatter=format, reply_formatter=format, default_reply=None, default_reply_delay=0):
        """
        Set up a ScriptableBase. Must be subclassed.

        :param name: the action namespace the server should operate on (e.g. `move_base`)
        :param goal_formatter: a function accepting a goal and returning a nice-looking summary str of it
        :param reply_formatter: a function accepting a result and returning a nice-looking summary str of it
        :param default_reply: optional. If set, this will be returned after default_result_delay,
            otherwise a .reply* call is needed to send a reply. Overriding the default reply and
            doing something custom is possible with the `custom_reply`-context manager
        :param default_reply_delay: If default_result is defined, the Scriptable... waits for this delay before returning.
            This delay is also used when no more specific reply_delay is specified in a reply_*-call
        """
        self._name = name

        self._current_goal = None

        # What to reply in the next goal?
        self._next_reply = None

        # Set when _as receives a goal (a question)
        self._request = Event()

        # Set when self.reply determines the result (the answer)
        self._reply = Event()

        # Set when _as has actually sent the result.
        # Only so printed output appears in order
        self._sent = Event()

        self._waiting_for = None

        self.goal_formatter = goal_formatter
        self.result_formatter = reply_formatter

        self._default_reply = default_reply
        self._default_reply_delay = default_reply_delay

        self._received_goals = []

        self._pre_reply_callbacks = []

    def stop(self):
        """
        If the process is blocked by waiting for some Events to be set, stop sets those Events.
        """
        self._sent.set()
        self._reply.set()

    def reply(self, result, timeout=None, reply_delay=None, marker=None):
        """
        Reply to the next goal with the given result, after `reply_delay` amount of seconds.
        An AssertionError is raised when a goal is not received within the given timeout.

        :param result: an ...ActionResult of the type associated with the Action-type of this server
        :param timeout: how long to wait for the goal? Defaults to None to wait indefinitely
        :param reply_delay: how to to reply_delay/calculate on this goal before sending the reply
        :param marker: A str that is printed in the output for easy reference between different replies
        :return: None
        """
        if reply_delay is None:
            reply_delay = self.default_reply_delay

        print('\n########  {}.reply{}  ###########'.format(self._name, '({})'.format(marker) if marker else ''))

        assert self._waiting_for is None, "reply{} cannot follow an 'await_goal', use reply_directly".format('({})'.format(marker) if marker else '')
        self.default_reply = None

        print("{}.reply{}: Waiting {}for goal...".format(self._name, '({})'.format(marker) if marker else '',
                                                         str(timeout)+'s ' if timeout is not None else ''))

        assert self._request.wait(timeout), \
            "{}.reply{} did not get a goal in time".format(self._name, '({})'.format(marker) if marker else '')

        self._request.clear()
        print("{}.reply{}: Got goal: {}"
            .format(self._name, '({})'.format(marker) if marker else '', self.goal_formatter(self._current_goal)))

        self._next_reply = result

        # The second {} will be filled by countdown_sleep
        countdown_sleep(reply_delay, text="{}.reply{}: Think for {}s. ".format(self._name, '({})'.format(marker) if marker else '', reply_delay) + "Remaining {}s...")

        self._reply.set()
        self._sent.wait()
        self._sent.clear()

        print("{}.reply{}: Finished replying"
            .format(self._name, '({})'.format(marker) if marker else ''))

    def reply_conditionally(self, condition, true_result, false_result, timeout=None, reply_delay=None, marker=None):
        """
        Reply one of two possibilities, based on a condition. This is a callable that, given a Goal, returns a bool
        If True, then reply with the true_reply and vice versa.
        An AssertionError is raised when a goal is not received within the given timeout.

        :param condition: callable(...Goal) -> bool
        :param true_result: a ...Result
        :param false_result: a ...Result
        :param timeout: seconds to wait for the goal. Defaults to None to wait indefinitely
        :param reply_delay: Delay the reply by this amount of seconds
        :param marker: A str that is printed in the output for easy reference between different replies
        :return: bool
        """
        if reply_delay is None:
            reply_delay = self.default_reply_delay

        print('\n########  {}.reply_conditionally{}  ###########'
            .format(self._name, '({})'.format(marker) if marker else ''))

        assert self._waiting_for is None, "reply_conditionally{} cannot follow an 'await_goal', use reply_directly".format('({})'.format(marker) if marker else '')
        self.default_reply = None

        print("{}.reply_conditionally{}: Waiting {}for goal..."
            .format(self._name, '({})'.format(marker) if marker else '',
                    str(timeout)+'s ' if timeout is not None else ''))
        assert self._request.wait(timeout), "{}.reply_conditionally{} did not get a goal in time"\
            .format(self._name, '({})'.format(marker) if marker else '')

        self._request.clear()
        print("{}.reply_conditionally{}: Got goal: {}"
            .format(self._name, '({})'.format(marker) if marker else '', self.goal_formatter(self._current_goal)))

        print("{}: Think for {}s...".format(self._name, reply_delay))
        match = condition(self._current_goal)
        if match:
            self._next_reply = true_result
        else:
            self._next_reply = false_result
        countdown_sleep(reply_delay, text="{}.reply_conditionally{}: Think for {}s. "
                        .format(self._name, '({})'.format(marker) if marker else '', reply_delay) + "Remaining {}s...")
        # raw_input("Press the any-key to continue: ")

        self._reply.set()
        self._sent.wait()
        self._sent.clear()

        print("{}.reply_conditionally{}: Finished replying"
            .format(self._name, '({})'.format(marker) if marker else ''))

        return match

    def await_goal(self, timeout=None, marker=None):
        """
        Await a goal to be sent to this Scriptable... and return that goal for close inspection.
        Based on that, send a reply via `direct_reply`
        An AssertionError is raised when a goal is not received within the given timeout.

        :param timeout: how long to wait for the goal? Defaults to None to wait indefinitely
        :param marker: A str that is printed in the output for easy reference between different replies
        :return: the received goal
        """
        print('\n########  {}.await_goal{}  ###########'
            .format(self._name, '({})'.format(marker) if marker else ''))
        self.default_reply = None

        print("{}.await_goal{}: Waiting {}for goal..."
            .format(self._name, '({})'.format(marker) if marker else '',
                    str(timeout)+'s ' if timeout is not None else ''))
        assert self._request.wait(timeout), "{}.await_goal{} did not get a goal in time".format(self._name, '({})'.format(marker) if marker else '')
        self._request.clear()
        print("{}.await_goal{}: Got goal: {}"
            .format(self._name, '({})'.format(marker) if marker else '', self.goal_formatter(self._current_goal)))

        self._waiting_for = 'direct_reply'

        return self._current_goal

    def direct_reply(self, result, reply_delay=None, marker=None):
        """
        Reply to the current goal with the given result, after `reply_delay` amount of seconds.

        :param result: a ...Result of the type associated with the type of this server
        :param reply_delay: how long to 'reply_delay/calculate' on this goal before sending the reply
        :param marker: A str that is printed in the output for easy reference between different replies
        """
        assert self._waiting_for == 'direct_reply', "reply cannot follow an 'await_goal', use reply_directly"

        if reply_delay is None:
            reply_delay = self.default_reply_delay

        self._next_reply = result

        # The second {} will be filled by countdown_sleep
        countdown_sleep(reply_delay, text="{}.direct_reply{}: Think for {}s. "
                        .format(self._name, '({})'.format(marker) if marker else '', reply_delay) + "Remaining {}s...")

        self._reply.set()
        self._sent.wait()
        self._sent.clear()

        self._waiting_for = None

        print("{}.direct_reply{}: Finished replying".format(self._name, '({})'.format(marker) if marker else ''))

    @property
    def default_reply(self):
        """
        The Result that is currently set to be returned by default
        """
        return self._default_reply

    @default_reply.setter
    def default_reply(self, result):
        """
        Set the current default reply.
        If this is None, there will not be a default reply, then a reply must be defined via a reply*-call

        :param result: a ...Result of the type associated with the type of this server
        """
        self._default_reply = result

    @property
    def default_reply_delay(self):
        """
        Wait this amount of time before returning the default reply
        """
        return self._default_reply_delay

    @default_reply_delay.setter
    def default_reply_delay(self, delay):
        """Set the delay after which the `default_result` is sent

        :param delay number of seconds to wait before sending the `default_result`"""
        self._default_reply_delay = delay

    @contextlib.contextmanager
    def custom_reply(self):
        """
        Use this context manager to temporarily define your own replies and go back to defaulting outside the context

        Usage:

        >>> server = ScriptableBase()
        >>> server.default_result = True
        >>> # Do other stuff that does not require special attention of server
        >>> with server.custom_reply():
        >>>    server.reply(False)
        >>> # Continue to do other stuff that does not require special attention of server

        """
        default = self.default_reply
        self.default_reply = None
        yield
        self.default_reply = default

    def clear_goals(self):
        """
        Clear the log of goals that we've received
        """
        self._received_goals = []

    @property
    def current_goal(self):
        """
        The goal we've received last
        """
        return self._received_goals[-1]

    @property
    def received_goals(self):
        """
        all goals received (since the last `clear_goals()` call
        """
        return self._received_goals

    def add_pre_reply_callback(self, callback):
        """
        Add a callback that is called just before a result is being sent back

        :param callback: callable that will receive the goal and result. The return value of the callable is discarded
        """
        assert isinstance(callback, collections.Callable)
        self._pre_reply_callbacks += [callback]

    def _call_pre_reply_callbacks(self, goal, reply):
        """
        Trigger all callback in the order they were added, with the current goal and the reply to it

        :param goal: current goal
        :param reply: reply to that goal
        """
        for func in self._pre_reply_callbacks:
            func(goal, reply)

    def match_in_received_goals(self, match_against, key=lambda x: x):
        """
        Find out if this server has any goal in it's history that is also in `match_against`

        :param match_against: We're looking for any of the items in `match_against`
        :param key: optionally transform the received goals with this callable into the same type as `match_against`'s elements
        :return: The matching elements
        """
        assert isinstance(key, collections.Callable), "key must be a callable"
        processed_previous_goals = list(map(key, self.received_goals))
        # Is there any overlap between the (processed) previous goals and what we're looking for?
        return set(match_against).intersection(set(processed_previous_goals))

    @contextlib.contextmanager
    def remember_goals(self):
        """
        Remember goals only instance of this context.
        Goals before this context are forgotten;
        Goals received during/in the context only are remembered;
        after the context everything is forgotten.
        """
        self.clear_goals()
        yield
        self.clear_goals()

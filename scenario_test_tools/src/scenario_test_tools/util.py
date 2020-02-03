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

import sys

import rospy


def countdown_sleep(duration, stepsize=1, text="{}"):
    """
    Print the numbers of a countdown on the terminal in-place, without using a new line for each number

    :param duration: How long to count down
    :param stepsize: Time between steps
    :param text: What text to display each tick. Must include a format string, eg. 'launch in {}...'
    """
    step, substep = divmod(duration, stepsize)
    for i in range(int(step), 0, -stepsize):
        if not rospy.is_shutdown():
            sys.stdout.write("\r" + text.format(i))
            sys.stdout.flush()
            rospy.sleep(stepsize)
    rospy.sleep(substep)

    if step > 0: sys.stdout.write('\n')
    sys.stdout.flush()

def round_tuple(tup, decimals):
    """
    Rounding operation broadcast over a tuple

    :param tup: tuple to be rounded
    :param decimals: how many decimals to round to
    :return: tuple of same dimension but with lower precision
    """
    return tuple([round(elem, decimals) for elem in tup])
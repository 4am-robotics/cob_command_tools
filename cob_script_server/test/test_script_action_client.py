#!/usr/bin/env python
from __future__ import print_function

# Copyright 2017 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
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
#! /usr/bin/env python

import rospy

import actionlib
from cob_script_server.msg import *

def script_action_client():
    client = actionlib.SimpleActionClient('/script_server', cob_script_server.msg.ScriptAction)
    client.wait_for_server()

    goal = cob_script_server.msg.ScriptGoal()
    goal.function_name = "move"
    goal.component_name = "base"
    goal.parameter_name = "[3.0,0.0,0.0]"
    pair = KeyValue(key="component_name",value="blub")
    goal.function_params.append(pair)
    pair = KeyValue(key="blocking",value="False")
    goal.function_params.append(pair)

    client.send_goal(goal)
    client.wait_for_result()
    return client.get_result()

if __name__ == '__main__':
    try:
        rospy.init_node('test_script_action_client')
        result = script_action_client()
        print("Result: {}".format(result))
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)
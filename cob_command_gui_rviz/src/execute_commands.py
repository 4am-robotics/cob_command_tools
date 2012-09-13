#!/usr/bin/python

import time

import roslib
roslib.load_manifest('cob_rviz_movement_control')
import rospy
import actionlib
import thread
#import roslib
#import os
#import pynotify 
#import sys

from simple_script_server import *

#buttons.py imports:
#import rospy
#from simple_script_server import *

#knoeppkes.py imports:
#import roslib
#roslib.load_manifest('cob_command_gui')
#from cob_relayboard.msg import EmergencyStopState
#from buttons import *
#import thread
#import pygtk
#pygtk.require('2.0')
#import gtk
#import roslib
#import os
#import pynotify
#import sys 

#execute_button_commands.py imports:
#mport time

#import roslib
#roslib.load_manifest('cob_3d_mapping_demonstrator')
#import rospy
#import actionlib

#from cob_script_server.msg import *
#from simple_script_server import *
#from cob_srvs.srv import *
#from cob_3d_mapping_msgs.msg import *
#from cob_3d_mapping_msgs.srv import *
#from sensor_msgs.msg import 


def start(func, largs):
  thread.start_new_thread(func,tuple(largs))

def start_action(data):
  data()

class execute_commands():
	
  #Initializes the actionlib interface of the script server.
  def __init__(self):
    #self.ns_global_prefix = "/cob_rviz_movement_control"
    self.script_action_server = actionlib.SimpleActionServer("cob_rviz_movement_control", ScriptAction, self.execute_cb, False)
    #self.script_action_server.register_preempt_callback(self.execute_stop)
    self.script_action_server.start()
    #self.trigger_client = actionlib.SimpleActionClient('trigger_mapping', TriggerMappingAction)
    self.sss = simple_script_server()

#------------------- Actionlib section -------------------#
  ## Executes actionlib callbacks.
  #
  # \param server_goal ScriptActionGoal
  #
  def execute_cb(self, server_goal):
    server_result  = ScriptActionResult().result
    function_name  = server_goal.function_name
    
    if (function_name == "init") or (function_name == "stop") or (function_name == "recover"):
       args = (server_goal.component_name, False)
       largs = list(args)
    else:
       args = (server_goal.component_name, server_goal.parameter_name, False)
       largs = list(args)
	   
       if server_goal.mode == "diff":
          largs.append(server_goal.mode) 
       elif server_goal.mode == "planned":
          largs.append(server_goal.mode)	
  
    if server_goal.function_name == "move":
	    ret = self.execute_move(largs)
    elif server_goal.function_name == "move_base_rel":
	    ret = self.execute_move_base_rel(largs)
    elif server_goal.function_name == "trigger":
	    ret = self.execute_trigger(largs)
    elif server_goal.function_name == "stop":
	    ret = self.execute_stop(largs)
    elif server_goal.function_name == "init":
	    ret = self.execute_init(largs)
    elif server_goal.function_name == "recover":
	    ret = self.execute_recover(largs)
    elif server_goal.function_name == "mode":
	    ret = self.execute_mode(largs)
   
    if self.script_action_server.is_active():
       self.script_action_server.set_succeeded(server_result)	   

  def execute_move(self, largs):
	  
	#print "-> execute_move"	
	command = lambda func=self.sss.move: start(func, largs)
	start_action(command)
   
  def execute_move_base_rel(self, largs):

	#print "-> execute_move_base_rel"
	command = lambda func=self.sss.move_base_rel: start(func, largs)
	start_action(command)
	
  def execute_trigger(self, largs):
	  
	 #print "-> execute_trigger"
	 command = lambda func=self.sss.trigger: start(func, largs)
	 start_action(command) 
 
  def execute_stop(self, largs):
 
	#print "-> execute_stop"
	command = lambda func=self.sss.stop: start(func, largs)
	start_action(command)
	
  def execute_init(self, largs):
 
	#print "-> execute_init"
	command = lambda func=self.sss.init: start(func, largs)
	start_action(command)

  def execute_recover(self, largs):
 
	#print "-> execute_recover"
	command = lambda func=self.sss.recover: start(func, largs)
	start_action(command)		

  def execute_mode(self, largs): 	
  
	#print "-> execute_mode"
	command = lambda func=self.sss.set_operation_mode: start(func, largs)
	start_action(command)
	
# main routine for running the script server
if __name__ == '__main__':
  rospy.init_node('execute_commands')
  execute_commands()
  rospy.loginfo("execute commands is running")
  rospy.spin()

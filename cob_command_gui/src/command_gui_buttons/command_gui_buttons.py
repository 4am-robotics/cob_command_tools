#!/usr/bin/python
#################################################################
##\file
#
# \note
#   Copyright (c) 2010 \n
#   Fraunhofer Institute for Manufacturing Engineering
#   and Automation (IPA) \n\n
#
#################################################################
#
# \note
#   Project name: care-o-bot
# \note
#   ROS stack name: cob_apps
# \note
#   ROS package name: cob_command_gui
#
# \author
#   Author: Florian Weisshardt, email:florian.weisshardt@ipa.fhg.de
# \author
#   Supervised by: Florian Weisshardt, email:florian.weisshardt@ipa.fhg.de
#
# \date Date of creation: Aug 2010
#
# \brief
#   Implementation of ROS node for command gui.
#
#################################################################
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     - Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer. \n
#     - Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution. \n
#     - Neither the name of the Fraunhofer Institute for Manufacturing
#       Engineering and Automation (IPA) nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission. \n
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License LGPL as 
# published by the Free Software Foundation, either version 3 of the 
# License, or (at your option) any later version.
# 
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU Lesser General Public License LGPL for more details.
# 
# You should have received a copy of the GNU Lesser General Public 
# License LGPL along with this program. 
# If not, see <http://www.gnu.org/licenses/>.
#
#################################################################

import rospy
from simple_script_server import *

## Implements configurable buttons
class command_gui_buttons:
	def __init__(self):
		self.sss = simple_script_server()
		self.panels = []
		self.stop_buttons = []
		self.init_buttons = []
		self.recover_buttons = []
		self.CreateControlPanel()

	## Creates the control panel out of configuration from ROS parameter server
	def CreateControlPanel(self):
		param_prefix = "~control_buttons"
		if not rospy.has_param(param_prefix):
			rospy.logerr("parameter %s does not exist on ROS Parameter Server, aborting...",param_prefix)
			return False
		group_param = rospy.get_param(param_prefix)
		#print group_param
		group_param = self.SortDict(group_param)
		#print group_param
		
		for group in group_param:
			group_name = group[1]["group_name"]
			component_name = group[1]["component_name"] # \todo check component name with robot_components.yaml files
			button_list = group[1]["buttons"]
			buttons = []
			for button in button_list:
				#print "button = ",button
				if button[1] == "move":
					buttons.append(self.CreateButton(button[0],self.sss.move,component_name,button[2]))
				elif button[1] == "move_base_rel":
					buttons.append(self.CreateButton(button[0],self.sss.move_base_rel,component_name,button[2]))
				elif button[1] == "trigger":
					buttons.append(self.CreateButton(button[0],self.sss.trigger,component_name,button[2]))
					if button[2] == "stop":
						self.stop_buttons.append(component_name)
					if button[2] == "init":
						self.init_buttons.append(component_name)
					if button[2] == "recover":
						self.recover_buttons.append(component_name)
				elif button[1] == "stop":
					buttons.append(self.CreateButton(button[0],self.sss.stop,component_name))
					self.stop_buttons.append(component_name)
				elif button[1] == "init":
					buttons.append(self.CreateButton(button[0],self.sss.init,component_name))
					self.init_buttons.append(component_name)
				elif button[1] == "recover":
					buttons.append(self.CreateButton(button[0],self.sss.recover,component_name))
					self.recover_buttons.append(component_name)
				else:
					rospy.logerr("Function <<%s>> not known to command gui",button[1])
					return False
			group = (group_name,buttons)
			
			# add nav buttons (optional)
			if component_name == "base": # \todo get base name from robot_components.yaml
				param_prefix = "~nav_buttons"
				if rospy.has_param(param_prefix):
					nav_buttons_param = rospy.get_param(param_prefix)
					nav_button_list = nav_buttons_param["buttons"]
					#print nav_button_list
					for button in nav_button_list:
						#print "button = ",button
						buttons.append(self.CreateButton(button[0],self.sss.move,component_name,button[2]))
				else:
					rospy.logwarn("parameter %s does not exist on ROS Parameter Server, no nav buttons will be available.",param_prefix)
			self.panels.append(group)

		# uniqify lists to not have double entries
		self.stop_buttons = self.uniqify_list(self.stop_buttons)
		self.init_buttons = self.uniqify_list(self.init_buttons)
		self.recover_buttons = self.uniqify_list(self.recover_buttons)
		
	
	## Creates one button with functionality
	def CreateButton(self,button_name,function,component_name,parameter_name=None):
		if parameter_name == None:
			button = (button_name,function,(component_name,False))
		else:
			button = (button_name,function,(component_name,parameter_name,False))
		return button
	
	## Sorts a dictionary alphabetically
	def SortDict(self,dictionary):
		keys = sorted(dictionary.iterkeys())
		k=[]
		#print "keys = ", keys
		#for key in keys:
		#	print "values = ", dictionary[key]
		return [[key,dictionary[key]] for key in keys]
		
	## Uniqifies a list to not have double entries
	def uniqify_list(self,seq, idfun=None): 
		# order preserving
		if idfun is None:
			def idfun(x): return x
		seen = {}
		result = []
		for item in seq:
			marker = idfun(item)
			# in old Python versions:
			# if seen.has_key(marker)
			# but in new ones:
			if marker in seen: continue
			seen[marker] = 1
			result.append(item)
		return result

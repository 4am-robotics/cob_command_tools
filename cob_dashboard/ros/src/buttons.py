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
#   ROS package name: cob_dashboard
#
# \author
#   Author: Florian Weisshardt, email:florian.weisshardt@ipa.fhg.de
# \author
#   Supervised by: Florian Weisshardt, email:florian.weisshardt@ipa.fhg.de
#
# \date Date of creation: Aug 2010
#
# \brief
#   Implementation of ROS node for dashboard.
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
class buttons:
	def __init__(self):
		self.sss = simple_script_server()
		self.panels = []
		self.CreatePanel()

	## Creates the panel out of configuration from ROS parameter server
	def CreatePanel(self):
		param_prefix = "/dashboard/buttons"
		if not rospy.has_param(param_prefix):
			rospy.logerr("parameter %s does not exist on ROS Parameter Server, aborting...",param_prefix)
			return False
		group_param = rospy.get_param(param_prefix)
		#print group_param
		group_param = self.SortDict(group_param)
		#print group_param
		
		for group in group_param:
			print group[0]
			buttons = []
			for button in group[1]:
				print button
				if button[1] == "move":
					buttons.append(self.CreateButton(button[0],self.sss.move,button[2],button[3]))
				elif button[1] == "trigger":
					buttons.append(self.CreateButton(button[0],self.sss.trigger,button[2],button[3]))
				elif button[1] == "mode":
					buttons.append(self.CreateButton(button[0],self.sss.set_operation_mode,button[2],button[3]))
				else:
					rospy.logerr("Function <<%s>> not known to dashboard",button[1])
					return False
			group = (group[0],buttons)
			self.panels.append(group)
	
	## Creates one button with functionality
	def CreateButton(self,button_name,function,component_name,parameter_name):
		#button = ([(button_name,function,(component_name,parameter_name)),])
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

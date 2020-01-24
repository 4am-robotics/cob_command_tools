#!/usr/bin/env python
#
# Copyright 2017 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#    http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import sys
import types
import string
import os

# graph includes
import pygraphviz as pgv

import rospy
from simple_script_server import script


if __name__ == "__main__":
	if (len(sys.argv) <= 1 ):
		print("Error: wrong number of input arguments")
		print("usage: rosrun cob_script_server script_to_graph.py <<SCRIPTFILE>> [level]")
		# \todo What does "level" do?
		sys.exit(1)
	elif (len(sys.argv) == 2):
		filename = sys.argv[1]
		level = 100
	elif (len(sys.argv) == 3):
		filename = sys.argv[1]
		level = int(sys.argv[2])
	else:
		print("Error: to many arguments")
		print("usage: rosrun cob_script_server script_to_graph.py <<SCRIPTFILE>> [level]")
		sys.exit(1)

	print("Script file = ", filename)
	print("Graph level = ", level)
	rospy.set_param("/script_server/level",level)

	filename_splitted = "/".split(filename)
	#print filename_splitted
	scriptfile = filename_splitted[-1]
	if(len(filename_splitted) > 1):
		filename_splitted.pop(len(filename_splitted)-1)
		scriptdir = ""
		for name in filename_splitted:
			scriptdir += name + "/"
		#print scriptdir
		sys.path.insert(0,scriptdir)
	scriptfile_woext = ".".split(scriptfile)[0]
	#print scriptfile_woext

	try:
		__import__(scriptfile_woext, globals(), locals(), [], -1)
		scriptmodule = sys.modules[scriptfile_woext]
		for classname in dir(scriptmodule):
			subclass = scriptmodule.__getattribute__(classname)
			if(isinstance(subclass, type)):
				if(issubclass(subclass, script)):
					if(classname != "script"):
						s = subclass()
						dotcode = s.Parse()
						print("dotcode = ",dotcode)
						graph=pgv.AGraph(dotcode)
						graph.layout('dot')
						basename, extension = os.path.splitext(filename)
						if (level == 100):
							graph.draw(basename + ".png")
						else:
							graph.draw(basename + "_" + str(level) + ".png")
	except ImportError:
		print("Unable to import script file")
		print("usage: rosrun cob_script_server script_to_graph.py <<SCRIPTFILE>> [level]")


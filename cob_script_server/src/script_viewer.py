#!/usr/bin/env python
#
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


import gtk
import gtk.gdk
import xdot

import pygraphviz as pgv

import rospy
from std_msgs.msg import String
from cob_script_server.msg import *

gtk.gdk.threads_init()


# check, if graph is available on parameter server
if rospy.has_param('script_server/graph'):
	dotcode = rospy.get_param("script_server/graph")
	G=pgv.AGraph(dotcode)
else:
	G=pgv.AGraph()
	G.add_node('no graph available')
	dotcode = G.string()

## Graph callback.
def graph_cb(msg):
	print("new graph received")
	global dotcode
	global G
	dotcode = msg.data
	print(dotcode)
	G=pgv.AGraph(dotcode)

	# update vizualisation
	gtk.gdk.threads_enter()
	widget.set_dotcode(dotcode)
	widget.zoom_to_fit()
	gtk.gdk.threads_leave()


## State callback.
def state_cb(msg):
	global widget

	# modify active node
	active_node = msg.full_graph_name
	rospy.loginfo("Received state <<%s>> from node <<%s>>",str(msg.state),active_node)
	try:
		n=G.get_node(active_node)
	except:
		rospy.logwarn("Node <<%s>> not found in graph",active_node)
		return
	n.attr['style']='filled'
	if msg.state == ScriptState.UNKNOWN:
		n.attr['fillcolor']='white'
	elif msg.state == ScriptState.ACTIVE:
		n.attr['fillcolor']='yellow'
	elif msg.state == ScriptState.SUCCEEDED:
		n.attr['fillcolor']='green'
	elif msg.state == ScriptState.FAILED:
		n.attr['fillcolor']='red'
	elif msg.state == ScriptState.PAUSED:
		n.attr['fillcolor']='orange'
	else:
		n.attr['fillcolor']='blue'
	dotcode = G.string()

	# update vizualisation
	gtk.gdk.threads_enter()
	widget.set_dotcode(dotcode)
	widget.zoom_to_fit()
	gtk.gdk.threads_leave()


# create gtk window
window = gtk.Window()
window.set_title('script viewer')
window.set_default_size(600, 800)
vbox = gtk.VBox()
window.add(vbox)

widget = xdot.DotWidget()
widget.set_dotcode(dotcode)
widget.zoom_to_fit()


vbox.pack_start(widget)

window.show_all()

window.connect('destroy', gtk.main_quit)

rospy.init_node('script_viewer', anonymous=True)
rospy.Subscriber("/script_server/graph", String, graph_cb)
rospy.Subscriber("/script_server/state", ScriptState, state_cb)
gtk.main()

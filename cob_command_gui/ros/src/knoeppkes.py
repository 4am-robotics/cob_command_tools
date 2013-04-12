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
#   Implementation of ROS node for command_gui.
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

import roslib
roslib.load_manifest('cob_command_gui')
from cob_relayboard.msg import EmergencyStopState
from buttons import *
import thread
import pygtk
pygtk.require('2.0')
import gtk
import roslib
import os
import pynotify
import sys 
import signal

planning_enabled = False
base_diff_enabled = False
confirm_commands_enabled = True

initialized = False

#Initializing the gtk's thread engine
#gtk.gdk.threads_init()

## Executes a button click in a new thread
def start(func, args):
  global planning_enabled
  global base_diff_enabled
  global confirm_commands_enabled
  execute_command = True
  
  largs = list(args)
  
  if confirm_commands_enabled and (largs[1] != 'stop'):
    confirm_dialog = gtk.MessageDialog(None, 0, gtk.MESSAGE_QUESTION, gtk.BUTTONS_YES_NO, "Execute Command?")
    if confirm_dialog.run() == gtk.RESPONSE_NO:
      execute_command = False
    confirm_dialog.destroy()
  if execute_command:
    if(largs[0] == "arm"):
      if(planning_enabled):
        largs.append("planned")
    if(largs[0] == "base"):
      if(base_diff_enabled):
        largs.append("diff") 
    #print "Args", tuple(largs)
    #print "func ", func
    thread.start_new_thread(func,tuple(largs))

def startGTK(widget, data):
  data()

## Class for general gtk panel implementation
class GtkGeneralPanel(gtk.Frame):
  def __init__(self,buttons):
    global initialized
    self.sss = simple_script_server()
    gtk.Frame.__init__(self)
    if not pynotify.init ("cob_command_gui"):
      sys.exit (1)
    self.em_stop = False
    self.set_label("general")
    self.set_shadow_type(gtk.SHADOW_IN)
    self.vbox = gtk.VBox(False, 0)
    self.add(self.vbox)
    #hbox=gtk.HBox(True, 0)
    #image = gtk.Image()
    #image.set_from_file(roslib.packages.get_pkg_dir("cob_command_gui") + "/common/files/icons/batti-040.png")
    #hbox.pack_start(image, False, False, 0)
    #label = gtk.Label("40 %")
    #hbox.pack_start(label, False, False, 0)
    #self.vbox.pack_start(hbox, False, False, 5)    
    hbox=gtk.HBox(True, 0)
    self.status_image = gtk.Image()
    #self.status_image.set_from_file(roslib.packages.get_pkg_dir("cob_command_gui") + "/common/files/icons/weather-clear.png")
    hbox.pack_start(self.status_image, False, False, 0)
    self.status_label = gtk.Label("Status OK")
    hbox.pack_start(self.status_label, False, False, 0)
    self.vbox.pack_start(hbox, False, False, 5)

    butstop = gtk.Button("Stop all")
    butstop.connect("clicked", lambda w: self.stop_all(buttons.stop_buttons))
    self.vbox.pack_start(butstop, False, False, 5)

    butinit = gtk.Button("Init all")
    butinit.connect("clicked", lambda w: self.init_all(buttons.init_buttons))
    self.vbox.pack_start(butinit, False, False, 5)

    butrec = gtk.Button("Recover all")
    butrec.connect("clicked", lambda w: self.recover_all(buttons.recover_buttons))
    self.vbox.pack_start(butrec, False, False, 5)

    plan_check = gtk.CheckButton("Planning")#
    plan_check.connect("toggled", self.planned_toggle)
    self.vbox.pack_start(plan_check, False, False, 5)

    base_mode_check = gtk.CheckButton("Base Diff")
    base_mode_check.connect("toggled", self.base_mode_toggle)
    self.vbox.pack_start(base_mode_check, False, False, 5)

    confirm_com_check = gtk.CheckButton("Confirm Commands")
    confirm_com_check.set_active(confirm_commands_enabled)
    confirm_com_check.connect("toggled", self.confirm_com_toggle)
    self.vbox.pack_start(confirm_com_check, False, False, 5)
    
    but = gtk.Button(stock=gtk.STOCK_QUIT )
    but.connect("clicked", lambda w: gtk.main_quit())
    self.vbox.pack_start(but, False, False, 5)
    initialized = True

  def stop_all(self,component_names):
    for component_name in component_names:
      self.sss.stop(component_name,blocking=False)
    
  def init_all(self,component_names):
    for component_name in component_names:
      self.sss.init(component_name,False)

  def recover_all(self,component_names):
    for component_name in component_names:
      self.sss.recover(component_name,False)

  def setEMStop(self, em):
    if(em):
      #print "Emergency Stop Active"
      gtk.threads_enter()
      self.status_image.set_from_file(roslib.packages.get_pkg_dir("cob_command_gui") + "/common/files/icons/weather-storm.png")
      self.status_label.set_text("EM Stop !")
      gtk.threads_leave()
      if(self.em_stop == False):
        self.em_stop = True
        n = pynotify.Notification("Emergency Stop issued!", "", "dialog-warning")
        n.set_timeout(1)
        n.show()
    else:
      #print "Status OK"
      #self.status_image.set_from_file(roslib.packages.get_pkg_dir("cob_command_gui") + "/common/files/icons/weather-clear.png")
      gtk.threads_enter()
      self.status_label.set_text("Status OK")
      gtk.threads_leave()
      if(self.em_stop == True):
        self.em_stop = False
        n = pynotify.Notification("Emergency Stop released!", "", "dialog-ok")
        n.set_timeout(1)
        n.show()
        
  def planned_toggle(self, b):
    global planning_enabled
    if(planning_enabled):
      planning_enabled = False
    else:
      planning_enabled = True  

  def base_mode_toggle(self, b):
    global base_diff_enabled
    if(base_diff_enabled):
      base_diff_enabled = False
    else:
      base_diff_enabled = True

  def confirm_com_toggle(self, b):
    global confirm_commands_enabled
    if(confirm_commands_enabled):
      confirm_commands_enabled = False
    else:
      confirm_commands_enabled = True

## Class for gtk panel implementation
class GtkPanel(gtk.Frame):
  def __init__(self, master=None, labeltext=""):
    gtk.Frame.__init__(self)
    self.set_label(labeltext)
    self.set_shadow_type(gtk.SHADOW_IN)
    self.vbox = gtk.VBox(False, 0)
    self.add(self.vbox)

  def addButton(self, text, command):
    but = gtk.Button(text)  
    but.connect("clicked", startGTK, command)
    #but.set_size_request(120,-1)
    self.vbox.pack_start(but, False, False, 5)
        
## Implementation of knoeppkes command gui
class Knoeppkes():
  def delete_event(self, widget, event, data=None):
    gtk.main_quit()
    return False

  def emcb(self, msg):
    global initialized
    if(initialized):
        self.gpanel.setEMStop(msg.emergency_state)
    
  def __init__(self):
    # init ros node
    rospy.init_node('cob_knoeppkes')
    rospy.Subscriber("/emergency_stop_state", EmergencyStopState, self.emcb)
  
    self.window = gtk.Window(gtk.WINDOW_TOPLEVEL)
    self.window.connect("delete_event", self.delete_event)
    self.window.set_title("cob command gui")  
    self.window.set_size_request(1000, 500)
    vbox = gtk.VBox(False, 1)
    self.hbox = gtk.HBox(True, 10)
    vbox.pack_start(self.hbox, True, True, 0)
    b = buttons()
    self.gpanel = GtkGeneralPanel(b)
    self.hbox.pack_start(self.gpanel,True, True, 3)
    panels = b.panels
    for pname, actions in panels:
      panel = GtkPanel(self, pname)
      for aname, func, args in actions:
        panel.addButton(text=aname, command=lambda f=func, a=args: start(f, a))
      self.hbox.pack_start(panel,True, True, 3)
    
    self.status_bar = gtk.Statusbar()  
    context_id = self.status_bar.get_context_id("Statusbar")
    string = "Connected to $ROS_MASTER_URI=" + os.environ.get("ROS_MASTER_URI")    
    self.status_bar.push(context_id, string)
    vbox.pack_start(self.status_bar, False, False, 0)     
    self.window.add(vbox)    
    self.window.show_all()
    gtk.gdk.threads_init()
    gtk.gdk.threads_enter()
    gtk.main()
    gtk.gdk.threads_leave()
    
def signal_handler(signal, frame):
#  print 'You pressed Ctrl+C!'
  gtk.main_quit()

if __name__ == "__main__":
	signal.signal(signal.SIGINT, signal_handler)
	app = Knoeppkes()


# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the Willow Garage nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import roslib
roslib.load_manifest('pr2_dashboard')

import wx
import rospy

from pr2_msgs.msg import PowerState, PowerBoardState
from pr2_power_board.srv import PowerBoardCommand, PowerBoardCommandRequest

from status_control import StatusControl

from os import path

breaker_prefixes = ("breakerleft", "breakerbase", "breakerright")

class BreakerControl(StatusControl):
  def __init__(self, parent, id, breaker_index, breaker_name, icons_path):
    StatusControl.__init__(self, parent, id, icons_path, breaker_prefixes[breaker_index], True)
    
    self.Bind(wx.EVT_LEFT_DOWN, self.on_left_down)
    
    self._power_control = rospy.ServiceProxy('power_board/control', PowerBoardCommand)
    self._serial = 0
    self._index = breaker_index
    self._name = breaker_name
    self._power_board_state = None
    self._last_status_msg = None
    
  def on_left_down(self, evt):
    menu = wx.Menu()
    menu.Bind(wx.EVT_MENU, self.on_enable, menu.Append(wx.ID_ANY, "Enable"))
    menu.Bind(wx.EVT_MENU, self.on_standby, menu.Append(wx.ID_ANY, "Standby"))
    menu.Bind(wx.EVT_MENU, self.on_disable, menu.Append(wx.ID_ANY, "Disable"))
    menu.AppendSeparator()
    menu.Bind(wx.EVT_MENU, self.on_enable_all, menu.Append(wx.ID_ANY, "Enable All Breakers"))
    menu.Bind(wx.EVT_MENU, self.on_standby_all, menu.Append(wx.ID_ANY, "Standby All Breakers"))
    menu.Bind(wx.EVT_MENU, self.on_disable_all, menu.Append(wx.ID_ANY, "Disable All Breakers"))
    
    self.toggle(True)
    self.PopupMenu(menu)
    self.toggle(False)
    
  def control(self, breaker, cmd):
    if (not self._power_board_state):
      wx.MessageBox("Cannot control breakers until we have received a power board state message", "Error", wx.OK|wx.ICON_ERROR)
      return False
    
    if (not self._power_board_state.run_stop or not self._power_board_state.wireless_stop):
      if (cmd == "start"):
        wx.MessageBox("Breakers will not enable because one of the runstops is pressed", "Error", wx.OK|wx.ICON_ERROR)
        return False
    
    try:
      power_cmd = PowerBoardCommandRequest()
      power_cmd.breaker_number = breaker
      power_cmd.command = cmd
      power_cmd.serial_number = self._serial
      self._power_control(power_cmd)
      
      return True
    except rospy.ServiceException, e:
      wx.MessageBox("Service call failed with error: %s"%(e), "Error", wx.OK|wx.ICON_ERROR)
      return False
      
    return False
  
  def control3(self, cmd):
    if (not self.control(0, cmd)):
      return False
    if (not self.control(1, cmd)):
      return False
    if (not self.control(2, cmd)):
      return False
    
    return True
  
  def on_enable(self, evt):
    self.set_enable()
  
  def on_standby(self, evt):
    self.set_standby()
  
  def on_disable(self, evt):
    self.set_disable()
    
  def on_enable_all(self, evt):
    self.set_enable_all()
  
  def on_standby_all(self, evt):
    self.set_standby_all()
  
  def on_disable_all(self, evt):
    self.set_disable_all()
    
  def set_enable(self):
    if (not self.control(self._index, "reset")):
      return
    
    self.control(self._index, "start")
    
  def set_standby(self):
    if (not self.control(self._index, "reset")):
      return
    
    self.control(self._index, "stop")
    
  def set_disable(self):
    self.control(self._index, "disable")
    
  def set_enable_all(self):
    if (not self.control3("reset")):
      return
    
    self.control3("start")
  
  def set_standby_all(self):
    if (not self.control3("reset")):
      return
    
    self.control3("stop")
  
  def set_disable_all(self):
    self.control3("disable")
    
  def set_power_board_state_msg(self, msg):
    last_voltage = msg.circuit_voltage[self._index]
      
    self._power_board_state = msg
    self._serial = msg.serial_num
    
    status_msg = "OK"
    
    if (msg.circuit_state[self._index] == PowerBoardState.STATE_DISABLED):
      self.set_error()
      status_msg = "Disabled"
    elif (msg.circuit_state[self._index] == PowerBoardState.STATE_NOPOWER):
      self.set_error()
      status_msg = "No Power"
    elif (msg.circuit_state[self._index] == PowerBoardState.STATE_STANDBY):
      self.set_warn()
      status_msg = "Standby"
    else:
      self.set_ok()
    
    if (status_msg != self._last_status_msg or abs(last_voltage - msg.circuit_voltage[self._index]) >= 0.1):  
        self.SetToolTip(wx.ToolTip("Breaker %s (index=[%s], voltage=[%.02f])) State: %s"%(self._name, self._index, msg.circuit_voltage[self._index], status_msg)))
        self._last_status_msg = status_msg
    
  def reset(self):
    self.set_stale()
    self.SetToolTip(wx.ToolTip("Breaker %s: Stale"%(self._name)))

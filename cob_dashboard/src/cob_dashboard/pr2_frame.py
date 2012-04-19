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
roslib.load_manifest('cob_dashboard')

import wx
import wx.aui
import wx.py.shell
import rxtools
import rxtools.cppwidgets as rxtools

import robot_monitor
from robot_monitor.robot_monitor_panel import RobotMonitorPanel
from pr2_msgs.msg import PowerState, PowerBoardState, DashboardState
from pr2_power_board.srv import PowerBoardCommand, PowerBoardCommandRequest
import std_msgs.msg
import std_srvs.srv

import rospy
from roslib import rosenv

from os import path
import threading

from status_control import StatusControl
from breaker_control import BreakerControl
from power_state_control import PowerStateControl
from diagnostics_frame import DiagnosticsFrame
from rosout_frame import RosoutFrame

class PR2Frame(wx.Frame):
    _CONFIG_WINDOW_X="/Window/X"
    _CONFIG_WINDOW_Y="/Window/Y"
    
    def __init__(self, parent, id=wx.ID_ANY, title='cob dashboard', pos=wx.DefaultPosition, size=(400, 50), style=wx.CAPTION|wx.CLOSE_BOX|wx.STAY_ON_TOP):
        wx.Frame.__init__(self, parent, id, title, pos, size, style)
        
        wx.InitAllImageHandlers()
        
        rospy.init_node('cob3_dashboard', anonymous=True)
        try:
            getattr(rxtools, "initRoscpp")
            rxtools.initRoscpp("cob3_dashboard_cpp", anonymous=True)
        except AttributeError:
            pass
        
        self.SetTitle('cob dashboard (%s)'%rosenv.get_master_uri())
        
        icons_path = path.join(roslib.packages.get_pkg_dir('pr2_dashboard'), "icons/")
        
        sizer = wx.BoxSizer(wx.HORIZONTAL)
        self.SetSizer(sizer)
        
        static_sizer = wx.StaticBoxSizer(wx.StaticBox(self, wx.ID_ANY, "Diagnostic"), wx.HORIZONTAL)
        sizer.Add(static_sizer, 0)
        
        # Diagnostics
        self._diagnostics_button = StatusControl(self, wx.ID_ANY, icons_path, "diag", True)
        self._diagnostics_button.SetToolTip(wx.ToolTip("Diagnostics"))
        static_sizer.Add(self._diagnostics_button, 0)
        
        # Rosout
        self._rosout_button = StatusControl(self, wx.ID_ANY, icons_path, "rosout", True)
        self._rosout_button.SetToolTip(wx.ToolTip("Rosout"))
        static_sizer.Add(self._rosout_button, 0)
        
        # Motors
        self._motors_button = StatusControl(self, wx.ID_ANY, icons_path, "motor", True)
        self._motors_button.SetToolTip(wx.ToolTip("Motors"))
        static_sizer.Add(self._motors_button, 0)
        self._motors_button.Bind(wx.EVT_LEFT_DOWN, self.on_motors_clicked)
        
        static_sizer = wx.StaticBoxSizer(wx.StaticBox(self, wx.ID_ANY, "Motors"), wx.HORIZONTAL)
        sizer.Add(static_sizer, 0)
        
        # Breakers
        breaker_names = ["Peft Arm", "Base/Body", "Right Arm"]
        self._breaker_ctrls = []
#        for i in xrange(0, 3):
#          ctrl = BreakerControl(self, wx.ID_ANY, i, breaker_names[i], icons_path)
#          ctrl.SetToolTip(wx.ToolTip("Breaker %s"%(breaker_names[i])))
#          self._breaker_ctrls.append(ctrl)
#          static_sizer.Add(ctrl, 0)
        
        static_sizer = wx.StaticBoxSizer(wx.StaticBox(self, wx.ID_ANY, "EM"), wx.HORIZONTAL)
        sizer.Add(static_sizer, 0)
        
        # run-stop
        self._runstop_ctrl = StatusControl(self, wx.ID_ANY, icons_path, "runstop", False)
        self._runstop_ctrl.SetToolTip(wx.ToolTip("Physical Runstop: Unknown"))
        static_sizer.Add(self._runstop_ctrl, 0)
        
        # Wireless run-stop
        #self._wireless_runstop_ctrl = StatusControl(self, wx.ID_ANY, icons_path, "runstop-wireless", False)
        #self._wireless_runstop_ctrl.SetToolTip(wx.ToolTip("Wireless Runstop: Unknown"))
        #static_sizer.Add(self._wireless_runstop_ctrl, 0)
        
        static_sizer = wx.StaticBoxSizer(wx.StaticBox(self, wx.ID_ANY, "Battery"), wx.HORIZONTAL)
        sizer.Add(static_sizer, 0)
        
        # Battery State
        self._power_state_ctrl = PowerStateControl(self, wx.ID_ANY, icons_path)
        self._power_state_ctrl.SetToolTip(wx.ToolTip("Battery: Stale"))
        static_sizer.Add(self._power_state_ctrl, 1, wx.EXPAND)
        
        self._config = wx.Config("pr2_dashboard")
        
        self.Bind(wx.EVT_CLOSE, self.on_close)
        
        self.Layout()
        self.Fit()
        
        self._diagnostics_frame = DiagnosticsFrame(self, wx.ID_ANY, "Diagnostics")
        self._diagnostics_frame.Hide()
        self._diagnostics_frame.Center()
        self._diagnostics_button.Bind(wx.EVT_BUTTON, self.on_diagnostics_clicked)
        
        self._rosout_frame = RosoutFrame(self, wx.ID_ANY, "Rosout")
        self._rosout_frame.Hide()
        self._rosout_frame.Center()
        self._rosout_button.Bind(wx.EVT_BUTTON, self.on_rosout_clicked)
        
        self.load_config()
        
        self._timer = wx.Timer(self)
        self.Bind(wx.EVT_TIMER, self.on_timer)
        self._timer.Start(500)
    
        self._dashboard_agg_sub = rospy.Subscriber('dashboard_agg', DashboardState, self.dashboard_callback)
        
        self._dashboard_message = None
        self._last_dashboard_message_time = 0.0
        
    def __del__(self):
        self._dashboard_agg_sub.unregister()
        
    def on_timer(self, evt):
      level = self._diagnostics_frame._diagnostics_panel.get_top_level_state()
      if (level == -1 or level == 3):
        if (self._diagnostics_button.set_stale()):
            self._diagnostics_button.SetToolTip(wx.ToolTip("Diagnostics: Stale"))
      elif (level >= 2):
        if (self._diagnostics_button.set_error()):
            self._diagnostics_button.SetToolTip(wx.ToolTip("Diagnostics: Error"))
      elif (level == 1):
        if (self._diagnostics_button.set_warn()):
            self._diagnostics_button.SetToolTip(wx.ToolTip("Diagnostics: Warning"))
      else:
        if (self._diagnostics_button.set_ok()):
            self._diagnostics_button.SetToolTip(wx.ToolTip("Diagnostics: OK"))
        
      self.update_rosout()
      
      if (rospy.get_time() - self._last_dashboard_message_time > 5.0):
          self._motors_button.set_stale()
          self._power_state_ctrl.set_stale()
          [ctrl.reset() for ctrl in self._breaker_ctrls]
          self._runstop_ctrl.set_stale()
          #self._wireless_runstop_ctrl.set_stale()
          ctrls = [self._motors_button, self._power_state_ctrl, self._runstop_ctrl]
          ctrls.extend(self._breaker_ctrls)
          for ctrl in ctrls:
              ctrl.SetToolTip(wx.ToolTip("No message received on dashboard_agg in the last 5 seconds"))
        
      if (rospy.is_shutdown()):
        self.Close()
        
    def on_diagnostics_clicked(self, evt):
      self._diagnostics_frame.Show()
      self._diagnostics_frame.Raise()
      
    def on_rosout_clicked(self, evt):
      self._rosout_frame.Show()
      self._rosout_frame.Raise()
      
    def on_motors_clicked(self, evt):
      menu = wx.Menu()
      menu.Bind(wx.EVT_MENU, self.on_reset_motors, menu.Append(wx.ID_ANY, "Reset"))
      menu.Bind(wx.EVT_MENU, self.on_halt_motors, menu.Append(wx.ID_ANY, "Halt"))
      self._motors_button.toggle(True)
      self.PopupMenu(menu)
      self._motors_button.toggle(False)
      
    def on_reset_motors(self, evt):
      # if any of the breakers is not enabled ask if they'd like to enable them
      if (self._dashboard_message is not None and self._dashboard_message.power_board_state_valid):
          all_breakers_enabled = reduce(lambda x,y: x and y, [state == PowerBoardState.STATE_ON for state in self._dashboard_message.power_board_state.circuit_state])
          if (not all_breakers_enabled):
              if (wx.MessageBox("Resetting the motors may not work because not all breakers are enabled.  Enable all the breakers first?", "Enable Breakers?", wx.YES_NO|wx.ICON_QUESTION, self) == wx.YES):
                  [breaker.set_enable() for breaker in self._breaker_ctrls]
        
      reset = rospy.ServiceProxy("pr2_etherCAT/reset_motors", std_srvs.srv.Empty)
       
      try:
        reset()
      except rospy.ServiceException, e:
        wx.MessageBox("Failed to reset the motors: service call failed with error: %s"%(e), "Error", wx.OK|wx.ICON_ERROR)
    
    def on_halt_motors(self, evt):
      halt = rospy.ServiceProxy("pr2_etherCAT/halt_motors", std_srvs.srv.Empty)
       
      try:
        halt()
      except rospy.ServiceException, e:
        wx.MessageBox("Failed to halt the motors: service call failed with error: %s"%(e), "Error", wx.OK|wx.ICON_ERROR)
      
    def dashboard_callback(self, msg):
      wx.CallAfter(self.new_dashboard_message, msg)
      
    def new_dashboard_message(self, msg):
      self._dashboard_message = msg
      self._last_dashboard_message_time = rospy.get_time()
      
      if (msg.motors_halted_valid):
        if (not msg.motors_halted.data):
          if (self._motors_button.set_ok()):
              self._motors_button.SetToolTip(wx.ToolTip("Motors: Running"))
        else:
          if (self._motors_button.set_error()):
              self._motors_button.SetToolTip(wx.ToolTip("Motors: Halted"))
      else:
          if (self._motors_button.set_stale()):
              self._motors_button.SetToolTip(wx.ToolTip("Motors: Stale"))
          
      if (msg.power_state_valid):
        self._power_state_ctrl.set_power_state(msg.power_state)
      else:
        self._power_state_ctrl.set_stale()
      
      if (msg.power_board_state_valid):
        [ctrl.set_power_board_state_msg(msg.power_board_state) for ctrl in self._breaker_ctrls]
        
        if (not msg.power_board_state.run_stop):
          # if the wireless stop is also off, we can't tell if the runstop is pressed or not
          if (not msg.power_board_state.wireless_stop):
            if (self._runstop_ctrl.set_warn()):
                self._runstop_ctrl.SetToolTip(wx.ToolTip("Physical Runstop: Unknown (Wireless is Pressed)"))
          else:
            if (self._runstop_ctrl.set_error()):
                self._runstop_ctrl.SetToolTip(wx.ToolTip("Physical Runstop: Pressed"))
        else:          
          if (self._runstop_ctrl.set_ok()):
              self._runstop_ctrl.SetToolTip(wx.ToolTip("Physical Runstop: OK"))
          
        #if (not msg.power_board_state.wireless_stop):
        #  pass
	#  #if (self._wireless_runstop_ctrl.set_error()):
        #  #    self._wireless_runstop_ctrl.SetToolTip(wx.ToolTip("Wireless Runstop: Pressed"))
        #else:
        #  if (self._wireless_runstop_ctrl.set_ok()):
        #      self._wireless_runstop_ctrl.SetToolTip(wx.ToolTip("Wireless Runstop: OK"))
      #else:
      #  if (self._wireless_runstop_ctrl.SetToolTip(wx.ToolTip("Wireless Runstop: Stale"))):
      #      self._runstop_ctrl.SetToolTip(wx.ToolTip("Physical Runstop: Stale"))
      #      [ctrl.reset() for ctrl in self._breaker_ctrls]
      #      self._runstop_ctrl.set_stale()
      #       self._wireless_runstop_ctrl.set_stale()
          
    def update_rosout(self):
      summary_dur = 30.0
      if (rospy.get_time() < 30.0):
          summary_dur = rospy.get_time() - 1.0
          
      if (summary_dur < 0):
          summary_dur = 0.0
    
      summary = self._rosout_frame.get_panel().getMessageSummary(summary_dur)
      
      if (summary.fatal or summary.error):
        self._rosout_button.set_error()
      elif (summary.warn):
        self._rosout_button.set_warn()
      else:
        self._rosout_button.set_ok()
        
        
      tooltip = ""
      if (summary.fatal):
        tooltip += "\nFatal: %s"%(summary.fatal)
      if (summary.error):
        tooltip += "\nError: %s"%(summary.error)
      if (summary.warn):
        tooltip += "\nWarn: %s"%(summary.warn)
      if (summary.info):
        tooltip += "\nInfo: %s"%(summary.info)
      if (summary.debug):
        tooltip += "\nDebug: %s"%(summary.debug)
      
      if (len(tooltip) == 0):
        tooltip = "Rosout: no recent activity"
      else:
        tooltip = "Rosout: recent activity:" + tooltip
    
      if (tooltip != self._rosout_button.GetToolTip().GetTip()):
          self._rosout_button.SetToolTip(wx.ToolTip(tooltip))
        
    def load_config(self):
      # Load our window options
      (x, y) = self.GetPositionTuple()
      (width, height) = self.GetSizeTuple()
      if (self._config.HasEntry(self._CONFIG_WINDOW_X)):
          x = self._config.ReadInt(self._CONFIG_WINDOW_X)
      if (self._config.HasEntry(self._CONFIG_WINDOW_Y)):
          y = self._config.ReadInt(self._CONFIG_WINDOW_Y)
      
      self.SetPosition((x, y))
      self.SetSize((width, height))
        
    def save_config(self):
      config = self._config
      
      (x, y) = self.GetPositionTuple()
      (width, height) = self.GetSizeTuple()
      config.WriteInt(self._CONFIG_WINDOW_X, x)
      config.WriteInt(self._CONFIG_WINDOW_Y, y)
      
      config.Flush()
        
    def on_close(self, event):
      self.save_config()
      
      self.Destroy()
      

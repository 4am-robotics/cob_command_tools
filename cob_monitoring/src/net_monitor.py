#!/usr/bin/env python
############################################################################
#    Copyright (C) 2009, Willow Garage, Inc.                               #
#    Copyright (C) 2013 by Ralf Kaestner                                   #
#    ralf.kaestner@gmail.com                                               #
#    Copyright (C) 2013 by Jerome Maye                                     #
#    jerome.maye@mavt.ethz.ch                                              #
#                                                                          #
#    All rights reserved.                                                  #
#                                                                          #
#    Redistribution and use in source and binary forms, with or without    #
#    modification, are permitted provided that the following conditions    #
#    are met:                                                              #
#                                                                          #
#    1. Redistributions of source code must retain the above copyright     #
#       notice, this list of conditions and the following disclaimer.      #
#                                                                          #
#    2. Redistributions in binary form must reproduce the above copyright  #
#       notice, this list of conditions and the following disclaimer in    #
#       the documentation and/or other materials provided with the         #
#       distribution.                                                      #
#                                                                          #
#    3. The name of the copyright holders may be used to endorse or        #
#       promote products derived from this software without specific       #
#       prior written permission.                                          #
#                                                                          #
#    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS   #
#    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT     #
#    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS     #
#    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE        #
#    COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,  #
#    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,  #
#    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;      #
#    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER      #
#    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT    #
#    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN     #
#    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE       #
#    POSSIBILITY OF SUCH DAMAGE.                                           #
############################################################################

# copied from https://github.com/ethz-asl/ros-system-monitor

from __future__ import with_statement

import rospy

import traceback
import threading
from threading import Timer
import sys, os, time
from time import sleep
import subprocess
import string
import re

import socket

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

stat_dict = {0: 'OK', 1: 'Warning', 2: 'Error'}

class NetMonitor():
  def __init__(self):
    rospy.init_node("net_monitor")
    self._mutex = threading.Lock()
    self._diag_hostname = rospy.get_param('~diag_hostname', "localhost")
    self._net_level_warn = rospy.get_param('~net_level_warn', 0.95)
    self._net_capacity = rospy.get_param('~net_capacity', 128)
    self._usage_stat = DiagnosticStatus()
    self._usage_stat.name = '%s Network Usage' % self._diag_hostname
    self._usage_stat.hardware_id = self._diag_hostname
    self._usage_stat.level = DiagnosticStatus.OK
    self._usage_stat.message = 'No Data'

    self._diag_pub = rospy.Publisher('/diagnostics', DiagnosticArray, queue_size = 1)
    self._usage_timer = rospy.Timer(rospy.Duration(1.0), self.check_usage)
    self._diag_timer = rospy.Timer(rospy.Duration(1.0), self.publish_stats)

    self._filehandles = {}  # type Dict[str, file], mapping paths to files to actual, open, file handles

  def read_sysinfo(self, path):
    if path not in self._filehandles:
      self._filehandles[path] = open(path)

    fh = self._filehandles[path]  # type: file
    fh.seek(0)
    return fh.readline().strip()

  def get_sys_net_stat(self, iface, sys):
    try:
      return 0, self.read_sysinfo('/sys/class/net/%s/statistics/%s' % (iface, sys))
    except:
      return -1, None

  def get_sys_net(self, iface, sys):
    try:
      with open('/sys/class/net/%s/%s' % (iface, sys)) as f:
        return 0, f.readline().strip()
    except:
      return -1, None

  def check_network(self):
    values = []
    net_dict = {0: 'OK', 1: 'High Network Usage', 2: 'Network Down', 3: 'Call Error'}
    try:
      p = subprocess.Popen('ifstat -q -S 1 1',
                           stdout = subprocess.PIPE,
                           stderr = subprocess.PIPE, shell = True)
      stdout, stderr = p.communicate()
      retcode = p.returncode
      if retcode != 0:
        values.append(KeyValue(key = "\"ifstat -q -S 1 1\" Call Error",
          value = str(retcode)))
        return DiagnosticStatus.ERROR, net_dict[3], values
      rows = stdout.split('\n')
      data = rows[0].split()
      ifaces = []
      for i in range(0, len(data)):
        ifaces.append(data[i])
      data = rows[2].split()
      kb_in = []
      kb_out = []
      for i in range(0, len(data), 2):
        kb_in.append(data[i])
        kb_out.append(data[i + 1])
      level = DiagnosticStatus.OK
      for i in range(0, len(ifaces)):
        values.append(KeyValue(key = str(i), value = "======================="))
        values.append(KeyValue(key = 'Interface Name',
          value = ifaces[i]))
        (retcode, cmd_out) = self.get_sys_net(ifaces[i], 'operstate')
        if retcode == 0:
          values.append(KeyValue(key = 'State', value = cmd_out))
          ifacematch = re.match('eth[0-9]+', ifaces[i]) or re.match('eno[0-9]+', ifaces[i])
          if ifacematch and (cmd_out == 'down' or cmd_out == 'dormant'):
            level = DiagnosticStatus.ERROR
        values.append(KeyValue(key = 'Input Traffic',
          value = str(float(kb_in[i]) / 1024) + " (MB/s)")) if kb_in[i] != 'n/a' else 0
        values.append(KeyValue(key = 'Output Traffic',
          value = str(float(kb_out[i]) / 1024) + " (MB/s)")) if kb_out[i] != 'n/a' else 0
        net_usage_in = float(kb_in[i]) / 1024 / self._net_capacity if kb_in[i] != 'n/a' else 0
        net_usage_out = float(kb_out[i]) / 1024 / self._net_capacity if kb_out[i] != 'n/a' else 0
        if net_usage_in > self._net_level_warn or\
          net_usage_out > self._net_level_warn:
          level = DiagnosticStatus.WARN
        (retcode, cmd_out) = self.get_sys_net(ifaces[i], 'mtu')
        if retcode == 0:
          values.append(KeyValue(key = 'MTU', value = cmd_out))
        #additional keys (https://www.kernel.org/doc/Documentation/ABI/testing/sysfs-class-net-statistics)
        (retcode, cmd_out) = self.get_sys_net_stat(ifaces[i], 'rx_bytes')
        if retcode == 0:
          values.append(KeyValue(key = 'Total received MB',
            value = str(float(cmd_out) / 1024 / 1024)))
        (retcode, cmd_out) = self.get_sys_net_stat(ifaces[i], 'tx_bytes')
        if retcode == 0:
          values.append(KeyValue(key = 'Total transmitted MB',
            value = str(float(cmd_out) / 1024 / 1024)))
        (retcode, cmd_out) = self.get_sys_net_stat(ifaces[i], 'collisions')
        if retcode == 0:
          values.append(KeyValue(key = 'collisions', value = cmd_out))
        (retcode, cmd_out) = self.get_sys_net_stat(ifaces[i], 'rx_errors')
        if retcode == 0:
          values.append(KeyValue(key = 'rx_errors', value = cmd_out))
        (retcode, cmd_out) = self.get_sys_net_stat(ifaces[i], 'rx_crc_errors')
        if retcode == 0:
          values.append(KeyValue(key = 'rx_crc_errors', value = cmd_out))
        (retcode, cmd_out) = self.get_sys_net_stat(ifaces[i], 'rx_dropped')
        if retcode == 0:
          values.append(KeyValue(key = 'rx_dropped', value = cmd_out))
        (retcode, cmd_out) = self.get_sys_net_stat(ifaces[i], 'rx_fifo_errors')
        if retcode == 0:
          values.append(KeyValue(key = 'rx_fifo_errors', value = cmd_out))
        (retcode, cmd_out) = self.get_sys_net_stat(ifaces[i], 'rx_frame_errors')
        if retcode == 0:
          values.append(KeyValue(key = 'rx_frame_errors', value = cmd_out))
        (retcode, cmd_out) = self.get_sys_net_stat(ifaces[i], 'rx_length_errors')
        if retcode == 0:
          values.append(KeyValue(key = 'rx_length_errors', value = cmd_out))
        (retcode, cmd_out) = self.get_sys_net_stat(ifaces[i], 'rx_missed_errors')
        if retcode == 0:
          values.append(KeyValue(key = 'rx_missed_errors', value = cmd_out))
        (retcode, cmd_out) = self.get_sys_net_stat(ifaces[i], 'rx_over_errors')
        if retcode == 0:
          values.append(KeyValue(key = 'rx_over_errors', value = cmd_out))
        (retcode, cmd_out) = self.get_sys_net_stat(ifaces[i], 'rx_packets')
        if retcode == 0:
          values.append(KeyValue(key = 'rx_packets', value = cmd_out))
        (retcode, cmd_out) = self.get_sys_net_stat(ifaces[i], 'tx_errors')
        if retcode == 0:
          values.append(KeyValue(key = 'tx_errors', value = cmd_out))
        (retcode, cmd_out) = self.get_sys_net_stat(ifaces[i], 'tx_aborted_errors')
        if retcode == 0:
          values.append(KeyValue(key = 'tx_aborted_errors', value = cmd_out))
        (retcode, cmd_out) = self.get_sys_net_stat(ifaces[i], 'tx_carrier_errors')
        if retcode == 0:
          values.append(KeyValue(key = 'tx_carrier_errors', value = cmd_out))
        (retcode, cmd_out) = self.get_sys_net_stat(ifaces[i], 'tx_fifo_errors')
        if retcode == 0:
          values.append(KeyValue(key = 'tx_fifo_errors', value = cmd_out))
        (retcode, cmd_out) = self.get_sys_net_stat(ifaces[i], 'tx_heartbeat_errors')
        if retcode == 0:
          values.append(KeyValue(key = 'tx_heartbeat_errors', value = cmd_out))
        (retcode, cmd_out) = self.get_sys_net_stat(ifaces[i], 'tx_window_errors')
        if retcode == 0:
          values.append(KeyValue(key = 'tx_window_errors', value = cmd_out))
        (retcode, cmd_out) = self.get_sys_net_stat(ifaces[i], 'tx_dropped')
        if retcode == 0:
          values.append(KeyValue(key = 'tx_dropped', value = cmd_out))
        (retcode, cmd_out) = self.get_sys_net_stat(ifaces[i], 'tx_packets')
        if retcode == 0:
          values.append(KeyValue(key = 'tx_packets', value = cmd_out))
        #additional keys (https://www.kernel.org/doc/Documentation/ABI/testing/sysfs-class-net)
        (retcode, cmd_out) = self.get_sys_net(ifaces[i], 'addr_assign_type')
        if retcode == 0:
          try:
            tmp_dict = {'0': 'permanent address', '1': 'randomly generated', '2': 'stolen from another device', '3': 'set using dev_set_mac_address'}
            values.append(KeyValue(key = 'addr_assign_type', value = tmp_dict[cmd_out]))
          except KeyError, e:
            values.append(KeyValue(key = 'addr_assign_type', value = cmd_out))
        (retcode, cmd_out) = self.get_sys_net(ifaces[i], 'address')
        if retcode == 0:
          values.append(KeyValue(key = 'address', value = cmd_out))
        (retcode, cmd_out) = self.get_sys_net(ifaces[i], 'carrier')
        if retcode == 0:
          try:
            tmp_dict = {'0': 'physical link is down', '1': 'physical link is up'}
            values.append(KeyValue(key = 'carrier', value = tmp_dict[cmd_out]))
          except KeyError, e:
            values.append(KeyValue(key = 'carrier', value = cmd_out))
        (retcode, cmd_out) = self.get_sys_net(ifaces[i], 'carrier_changes')
        if retcode == 0:
          values.append(KeyValue(key = 'carrier_changes', value = cmd_out))
        (retcode, cmd_out) = self.get_sys_net(ifaces[i], 'carrier_up_count')
        if retcode == 0:
          values.append(KeyValue(key = 'carrier_up_count', value = cmd_out))
        (retcode, cmd_out) = self.get_sys_net(ifaces[i], 'carrier_down_count')
        if retcode == 0:
          values.append(KeyValue(key = 'carrier_down_count', value = cmd_out))
        (retcode, cmd_out) = self.get_sys_net(ifaces[i], 'speed')
        if retcode == 0:
          values.append(KeyValue(key = 'speed', value = cmd_out))
        (retcode, cmd_out) = self.get_sys_net(ifaces[i], 'tx_queue_len')
        if retcode == 0:
          values.append(KeyValue(key = 'tx_queue_len', value = cmd_out))
    except Exception, e:
      rospy.logerr(traceback.format_exc())
      msg = 'Network Usage Check Error'
      values.append(KeyValue(key = msg, value = str(e)))
      level = DiagnosticStatus.ERROR
    return level, net_dict[level], values

  def check_usage(self, event):
    diag_level = DiagnosticStatus.OK
    diag_vals = []
    diag_msgs = []
    net_level, net_msg, net_vals = self.check_network()
    diag_vals.extend(net_vals)
    if net_level > DiagnosticStatus.OK:
      diag_msgs.append(net_msg)
    diag_level = max(diag_level, net_level)
    if diag_msgs and diag_level > DiagnosticStatus.OK:
      usage_msg = ', '.join(set(diag_msgs))
    else:
      usage_msg = stat_dict[diag_level]
    with self._mutex:
      self._usage_stat.level = diag_level
      self._usage_stat.values = diag_vals
      self._usage_stat.message = usage_msg

  def publish_stats(self, event):
    with self._mutex:
      msg = DiagnosticArray()
      msg.header.stamp = rospy.get_rostime()
      msg.status.append(self._usage_stat)
      self._diag_pub.publish(msg)

if __name__ == '__main__':
  net_node = NetMonitor()
  rospy.spin()

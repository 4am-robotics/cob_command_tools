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


import os
import sys
from subprocess import Popen, PIPE
import paramiko

import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

class IwConfigParser():
    def __init__(self, interface):
        self.interface = interface
        self.norm = ""
        self.essid = ""
        self.mode = ""
        self.frequency = 0.0
        self.access_point = ""
        self.bit_rate = 0.0
        self.tx_power = 0.0
        self.retry_short_limit = 0
        self.rts_thr = ""
        self.fragment_thr = ""
        self.power_management = ""
        self.link_quality = ""
        self.link_quality_percent = 0
        self.signal_level = 0.0
        self.rx_invalid_nwid = 0
        self.rx_invalid_crypt = 0
        self.rx_invalid_frag = 0
        self.tx_excessive_retries = 0
        self.invalic_misc = 0
        self.missed_beacon = 0
        self.stat = DiagnosticStatus()

    def _parse_info(self, info):
        try:
            split = info.split('IEEE ',1)
            split = split[1].split('ESSID:',1)
            self.norm = split[0].encode('utf8').strip()
            split = split[1].split('\n',1)
            self.essid = split[0].encode('utf8').strip()
            split = split[1].split('Mode:',1)
            split = split[1].split('Frequency:',1)
            self.mode = split[0].encode('utf8').strip()
            split = split[1].split(' GHz',1)
            self.frequency = float(split[0].strip())
            split = split[1].split('Access Point: ',1)
            split = split[1].split('\n',1)
            self.access_point = split[0].encode('utf8').strip()
            split = split[1].split('Bit Rate=',1)
            split = split[1].split(' Mb/s',1)
            self.bit_rate = float(split[0].strip())
            if split[1].find('Tx-Power') != -1:
                split = split[1].split('Tx-Power=',1)
                split = split[1].split(' dBm',1)
                self.tx_power = float(split[0].strip())
            if split[1].find('Retry short limit:') != -1:
                split = split[1].split('Retry short limit:',1)
            if split[1].find('Retry short limit=') != -1:
                split = split[1].split('Retry short limit=',1)
            if split[1].find('RTS thr:') != -1:
                split = split[1].split('RTS thr:',1)
            if split[1].find('RTS thr=') != -1:
                split = split[1].split('RTS thr=',1)
            self.retry_short_limit = split[0].encode('utf8').strip()
            if split[1].find('Fragment thr:') != -1:
                split = split[1].split('Fragment thr:',1)
            if split[1].find('Fragment thr=') != -1:
                split = split[1].split('Fragment thr=',1)
            self.rts_thr = split[0].encode('utf8').strip()
            split = split[1].split('\n',1)
            self.fragment_thr = split[0].encode('utf8').strip()
            split = split[1].split('Power Management:',1)
            split = split[1].split('\n',1)
            self.power_managment = split[0].encode('utf8').strip()
            split = split[1].split('Link Quality=',1)
            split = split[1].split('Signal level=',1)
            self.link_quality = split[0].encode('utf8').strip()
            self.link_quality_percent = split[0].split('/')
            self.link_quality_percent = int(float(self.link_quality_percent[0].strip()) / float(self.link_quality_percent[1].strip())*100.0)
            split = split[1].split(' dBm',1)
            self.signal_level = float(split[0].strip())
            split = split[1].split('Rx invalid nwid:',1)
            split = split[1].split('Rx invalid crypt:',1)
            self.rx_invalid_nwid = int(split[0].strip())
            split = split[1].split('Rx invalid frag:',1)
            self.rx_invalid_crypt = int(split[0].strip())
            split = split[1].split('\n',1)
            self.rx_invalid_frag = int(split[0].strip())
            split = split[1].split('Tx excessive retries:',1)
            split = split[1].split('Invalid misc:',1)
            self.tx_excessive_retries = int(split[0].strip())
            split = split[1].split('Missed beacon:',1)
            self.invalid_misc = int(split[0].strip())
            split = split[1].split('\n',1)
            self.missed_beacon = int(split[0].strip())

            #ToDo: set diagnostic warning/error level accordingly
            self.stat.level = DiagnosticStatus.OK
            self.stat.message = "OK"
            self.stat.values = [ KeyValue("Interface" , self.interface),
                                 KeyValue("IEEE Norm", self.norm),
                                 KeyValue("ESSID", self.essid),
                                 KeyValue("Mode", self.mode),
                                 KeyValue("Frequency", str(self.frequency)),
                                 KeyValue("Access Point", self.access_point),
                                 KeyValue("Bit Rate [Mb/s]", str(self.bit_rate)),
                                 KeyValue("Tx-Power [dBm]", str(self.tx_power)),
                                 KeyValue("Retry short limit", str(self.retry_short_limit)),
                                 KeyValue("RTS thr", self.rts_thr),
                                 KeyValue("Fragment thr", self.fragment_thr),
                                 KeyValue("Power Managment", self.power_management),
                                 KeyValue("Link Quality", self.link_quality),
                                 KeyValue("Link Quality %", str(self.link_quality_percent)),
                                 KeyValue("Signal level [dBm]", str(self.signal_level)),
                                 KeyValue("Rx invalid nwid", str(self.rx_invalid_nwid)),
                                 KeyValue("Rx invalid crypt", str(self.rx_invalid_crypt)),
                                 KeyValue("Rx invalid frag", str(self.rx_invalid_frag)),
                                 KeyValue("Tx excessive retries", str(self.tx_excessive_retries)),
                                 KeyValue("Invalid misc", str(self.invalic_misc)),
                                 KeyValue("Missed beacon", str(self.missed_beacon)) ]

        except Exception, e:
            rospy.logerr("Parsing Error: %s" %e)
            self.stat.level = DiagnosticStatus.ERROR
            self.stat.message = 'iwconfig Exception'
            self.stat.values = [ KeyValue(key = 'Exception', value = str(e)) ]

class IwConfigLocal(IwConfigParser):
    def __init__(self, interface):
        IwConfigParser.__init__(self, interface)

    def update(self):
        try:
            p = Popen(["iwconfig", self.interface], stdout=PIPE, stdin=PIPE, stderr=PIPE)
            res = p.wait()
            (stdout,stderr) = p.communicate()

            if res != 0:
                self.stat.level = DiagnosticStatus.ERROR
                self.stat.message = 'iwconfig Error'
                self.stat.values = [ KeyValue(key = 'iwconfig Error', value = stderr),
                                     KeyValue(key = 'Output', value = stdout) ]
            else:
                self._parse_info(stdout)
        except Exception, e:
            self.stat.level = DiagnosticStatus.ERROR
            self.stat.message = 'iwconfig Exception'
            self.stat.values = [ KeyValue(key = 'Exception', value = str(e)) ]

class IwConfigSSH(IwConfigParser):
    def __init__(self, interface, hostname, user, password):
        IwConfigParser.__init__(self, interface)
        self.ssh = paramiko.SSHClient()
        self.ssh.load_system_host_keys()
        ssh_key_file   = os.getenv("HOME")+'/.ssh/id_rsa.pub'
        self.ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())# no known_hosts error
        self.ssh.connect(str(hostname), username=user, key_filename=ssh_key_file) # no passwd needed
        #self.ssh.connect(str(hostname), username=user, password=password)

    def update(self):
        try:
            (stdin, stdout, stderr) = self.ssh.exec_command("iwconfig %s"%self.interface)

            output = ''.join(stdout.readlines())
            self._parse_info(output)
        except Exception, e:
            self.stat.level = DiagnosticStatus.ERROR
            self.stat.message = 'iwconfig Exception'
            self.stat.values = [ KeyValue(key = 'Exception', value = str(e)) ]

class WlanMonitor():
    def __init__(self):
        rospy.init_node("wlan_monitor")
        self.get_params()

        self._wlan_stat = DiagnosticStatus()
        self._wlan_stat.name = '%s WLAN Info' % self.diag_hostname
        self._wlan_stat.hardware_id = self.diag_hostname
        self._wlan_stat.level = DiagnosticStatus.OK
        self._wlan_stat.message = 'No Data'
        self._wlan_stat.values = []
        self.msg = DiagnosticArray()
        self.msg.header.stamp = rospy.get_rostime()
        self.msg.status = [self._wlan_stat]

        self.diag_pub = rospy.Publisher("/diagnostics", DiagnosticArray, queue_size=1)
        self.diag_timer = rospy.Timer(rospy.Duration(1.0), self.publish_diagnostics)

        if self.monitor_local:
            self.iwconfig = IwConfigLocal(self.interface)
        else:
            try:
                self.iwconfig = IwConfigSSH(self.interface, self.diag_hostname, self.user, self.password)
            except Exception, e:
                rospy.logerr("Error connecting ssh to host: %s",e.message)
                self._wlan_stat.level = DiagnosticStatus.ERROR
                self._wlan_stat.message = 'iwconfig Exception'
                self._wlan_stat.values = [ KeyValue(key = 'Exception', value = str(e)) ]
                self.msg.status = [self._wlan_stat]
                return

        self.monitor_timer = rospy.Timer(rospy.Duration(1.0), self.update_diagnostics)

    def update_diagnostics(self, event):
        self.iwconfig.update()

        self.msg = DiagnosticArray()
        self.msg.header.stamp = rospy.get_rostime()
        self._wlan_stat.level = self.iwconfig.stat.level
        self._wlan_stat.message = self.iwconfig.stat.message
        self._wlan_stat.values = self.iwconfig.stat.values
        self.msg.status = [self._wlan_stat]

    def publish_diagnostics(self, event):
        self.diag_pub.publish(self.msg)

    def get_params(self):
        self.diag_hostname = rospy.get_param('~diag_hostname', "localhost")
        self.interface = rospy.get_param('~interface', "wlan0")
        self.monitor_local = rospy.get_param("~monitor_local", True)
        self.user = rospy.get_param('~user', "")
        self.password = rospy.get_param('~password', "")

if __name__ == "__main__":
    monitor = WlanMonitor()
    rospy.spin()

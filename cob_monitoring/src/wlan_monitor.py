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

class IwConfigParser(object):
    def _parse_info(self, info):
        if 'ESSID' in info:
           return self._parse_client(info)
        else:
           return self._parse_ap(info)

    def _parse_ap(self, info):
        values = []
        try:
            split = info.split('IEEE ',1)
            split = split[1].split('Mode:',1)
            norm = split[0].encode('utf8').strip()
            values.append(KeyValue("IEEE Norm", norm))
            split = split[1].split('\n',1)
            mode = split[0].encode('utf8').strip()
            values.append(KeyValue("Mode", mode))
            if split[1].find('Retry short limit:') != -1:
                split = split[1].split('Retry short limit:',1)
            if split[1].find('Retry short limit=') != -1:
                split = split[1].split('Retry short limit=',1)
            if split[1].find('RTS thr:') != -1:
                split = split[1].split('RTS thr:',1)
            if split[1].find('RTS thr=') != -1:
                split = split[1].split('RTS thr=',1)
            retry_short_limit = split[0].encode('utf8').strip()
            values.append(KeyValue("Retry short limit", str(retry_short_limit)))
            if split[1].find('Fragment thr:') != -1:
                split = split[1].split('Fragment thr:',1)
            if split[1].find('Fragment thr=') != -1:
                split = split[1].split('Fragment thr=',1)
            rts_thr = split[0].encode('utf8').strip()
            values.append(KeyValue("RTS thr", rts_thr))
            split = split[1].split('\n',1)
            fragment_thr = split[0].encode('utf8').strip()
            values.append(KeyValue("Fragment thr", fragment_thr))
            split = split[1].split('Power Management:',1)
            split = split[1].split('\n',1)
            power_managment = split[0].encode('utf8').strip()
            values.append(KeyValue("Power Managment", power_managment))

        except Exception, e:
            rospy.logerr("IwConfigParser parsing exception: %s" %e)
            values = [ KeyValue(key = 'parsing exception', value = str(e)) ]
        
        return values

    def _parse_client(self, info):
        values = []
        try:
            split = info.split('IEEE ',1)
            split = split[1].split('ESSID:',1)
            norm = split[0].encode('utf8').strip()
            values.append(KeyValue("IEEE Norm", norm))
            split = split[1].split('\n',1)
            essid = split[0].encode('utf8').strip()
            values.append(KeyValue("ESSID", essid))
            split = split[1].split('Mode:',1)
            split = split[1].split('Frequency:',1)
            mode = split[0].encode('utf8').strip()
            values.append(KeyValue("Mode", mode))
            split = split[1].split(' GHz',1)
            frequency = float(split[0].strip())
            values.append(KeyValue("Frequency", str(frequency)))
            split = split[1].split('Access Point: ',1)
            split = split[1].split('\n',1)
            access_point = split[0].encode('utf8').strip()
            values.append(KeyValue("Access Point", access_point))
            split = split[1].split('Bit Rate=',1)
            split = split[1].split(' Mb/s',1)
            bit_rate = float(split[0].strip())
            values.append(KeyValue("Bit Rate [Mb/s]", str(bit_rate)))
            if split[1].find('Tx-Power') != -1:
                split = split[1].split('Tx-Power=',1)
                split = split[1].split(' dBm',1)
                tx_power = float(split[0].strip())
                values.append(KeyValue("Tx-Power [dBm]", str(tx_power)))
            if split[1].find('Retry short limit:') != -1:
                split = split[1].split('Retry short limit:',1)
            if split[1].find('Retry short limit=') != -1:
                split = split[1].split('Retry short limit=',1)
            if split[1].find('RTS thr:') != -1:
                split = split[1].split('RTS thr:',1)
            if split[1].find('RTS thr=') != -1:
                split = split[1].split('RTS thr=',1)
            retry_short_limit = split[0].encode('utf8').strip()
            values.append(KeyValue("Retry short limit", str(retry_short_limit)))
            if split[1].find('Fragment thr:') != -1:
                split = split[1].split('Fragment thr:',1)
            if split[1].find('Fragment thr=') != -1:
                split = split[1].split('Fragment thr=',1)
            rts_thr = split[0].encode('utf8').strip()
            values.append(KeyValue("RTS thr", rts_thr))
            split = split[1].split('\n',1)
            fragment_thr = split[0].encode('utf8').strip()
            values.append(KeyValue("Fragment thr", fragment_thr))
            split = split[1].split('Power Management:',1)
            split = split[1].split('\n',1)
            power_managment = split[0].encode('utf8').strip()
            values.append(KeyValue("Power Managment", power_managment))
            split = split[1].split('Link Quality=',1)
            split = split[1].split('Signal level=',1)
            link_quality = split[0].encode('utf8').strip()
            values.append(KeyValue("Link Quality", link_quality))
            link_quality_percent = split[0].split('/')
            link_quality_percent = int(float(link_quality_percent[0].strip()) / float(link_quality_percent[1].strip())*100.0)
            values.append(KeyValue("Link Quality %", str(link_quality_percent)))
            split = split[1].split(' dBm',1)
            signal_level = float(split[0].strip())
            values.append(KeyValue("Signal level [dBm]", str(signal_level)))
            split = split[1].split('Rx invalid nwid:',1)
            split = split[1].split('Rx invalid crypt:',1)
            rx_invalid_nwid = int(split[0].strip())
            values.append(KeyValue("Rx invalid nwid", str(rx_invalid_nwid)))
            split = split[1].split('Rx invalid frag:',1)
            rx_invalid_crypt = int(split[0].strip())
            values.append(KeyValue("Rx invalid crypt", str(rx_invalid_crypt)))
            split = split[1].split('\n',1)
            rx_invalid_frag = int(split[0].strip())
            values.append(KeyValue("Rx invalid frag", str(rx_invalid_frag)))
            split = split[1].split('Tx excessive retries:',1)
            split = split[1].split('Invalid misc:',1)
            tx_excessive_retries = int(split[0].strip())
            values.append(KeyValue("Tx excessive retries", str(tx_excessive_retries)))
            split = split[1].split('Missed beacon:',1)
            invalid_misc = int(split[0].strip())
            values.append(KeyValue("Invalid misc", str(invalid_misc)))
            split = split[1].split('\n',1)
            missed_beacon = int(split[0].strip())
            values.append(KeyValue("Missed beacon", str(missed_beacon)))

        except Exception, e:
            rospy.logerr("IwConfigParser parsing exception: %s" %e)
            values = [ KeyValue(key = 'parsing exception', value = str(e)) ]
        
        return values

class IwConfigLocal(IwConfigParser):
    def __init__(self):
        IwConfigParser.__init__(self)

        self.interfaces = []
        self.stat = DiagnosticStatus()
        self.stat.level = DiagnosticStatus.OK
        self.stat.message = "OK"
        self.stat.values = []
        try:
            p = Popen("iw dev | awk '$1==\"Interface\"{print $2}'", stdout=PIPE, stdin=PIPE, stderr=PIPE, shell=True)
            res = p.wait()
            (stdout,stderr) = p.communicate()
            self.interfaces = sorted(os.linesep.join([s for s in stdout.splitlines() if s]).split('\n'))
        except Exception, e:
            rospy.logerr("IwConfigLocal init exception: %s" %e)

    def update(self):
        self.stat.level = DiagnosticStatus.OK
        self.stat.message = "OK"
        self.stat.values = []
        for interface in self.interfaces:
            self.stat.values.append(KeyValue(key = str(interface), value = "======================="))
            try:
                p = Popen(["iwconfig", interface], stdout=PIPE, stdin=PIPE, stderr=PIPE)
                res = p.wait()
                (stdout,stderr) = p.communicate()

                if res != 0:
                    self.stat.values.append(KeyValue(key = 'iwconfig stderr', value = stderr))
                    self.stat.values.append(KeyValue(key = 'iwconfig stdout', value = stdout))
                else:
                    self.stat.values += self._parse_info(stdout)
            except Exception, e:
                rospy.logerr("IwConfigLocal update exception: %s" %e)
                self.stat.values.append(KeyValue(key = 'update exception', value = str(e)))

class IwConfigSSH(IwConfigParser):
    def __init__(self, hostname, user, password):
        IwConfigParser.__init__(self)
        self.ssh = paramiko.SSHClient()
        self.ssh.load_system_host_keys()
        ssh_key_file   = os.getenv("HOME")+'/.ssh/id_rsa.pub'
        self.ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())# no known_hosts error
        self.ssh.connect(str(hostname), username=user, key_filename=ssh_key_file) # no passwd needed
        #self.ssh.connect(str(hostname), username=user, password=password)

        self.interfaces = []
        self.stat = DiagnosticStatus()
        self.stat.level = DiagnosticStatus.OK
        self.stat.message = "OK"
        self.stat.values = []
        try:
            (stdin, stdout, stderr) = self.ssh.exec_command("iw dev | awk '$1==\"Interface\"{print $2}'")
            output = ''.join(stdout.readlines())
            self.interfaces = sorted(os.linesep.join([s for s in output.splitlines() if s]).split('\n'))
        except Exception, e:
            rospy.logerr("IwConfigSSH init exception: %s" %e)

    def update(self):
        self.stat.level = DiagnosticStatus.OK
        self.stat.message = "OK"
        self.stat.values = []
        for interface in self.interfaces:
            self.stat.values.append(KeyValue(key = str(interface), value = "======================="))
            try:
                (stdin, stdout, stderr) = self.ssh.exec_command("iwconfig %s"%interface)

                output = ''.join(stdout.readlines())
                self.stat.values += self._parse_info(output)
            except Exception, e:
                rospy.logerr("IwConfigSSH update exception: %s" %e)
                self.stat.values.append(KeyValue(key = 'update exception', value = str(e)))

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
            self.iwconfig = IwConfigLocal()
        else:
            try:
                self.iwconfig = IwConfigSSH(self.diag_hostname, self.user, self.password)
            except Exception, e:
                msg = "Cannot connect to router via ssh. Please check if ssh key of user '%s' is contained in the router configuration or password is provided. Error message: %s"%(self.user, e.message)
                rospy.logerr(msg)
                self._wlan_stat.level = DiagnosticStatus.ERROR
                self._wlan_stat.message = msg
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
        self.monitor_local = rospy.get_param("~monitor_local", True)
        self.user = rospy.get_param('~user', "")
        self.password = rospy.get_param('~password', "")

if __name__ == "__main__":
    monitor = WlanMonitor()
    rospy.spin()

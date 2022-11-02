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

import getpass
import re
import os
import sys
import traceback
from subprocess import Popen, PIPE
import paramiko

import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

class IwConfigParser(object):
    def _parse_info(self, info):
        values = []
        # split by either double-space or newline
        for information in re.split("  |\n",info):
            # split by either : or =
            content = re.split(":|=", information)
            if len(content) == 2:
                try:
                    content[0] = content[0].decode()  #python3
                    content[1] = content[1].decode()  #python3
                except (UnicodeDecodeError, AttributeError):
                    pass
                values.append(KeyValue(content[0].lstrip(), content[1].rstrip()))
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
            try:
                stdout = stdout.decode()  #python3
            except (UnicodeDecodeError, AttributeError):
                pass
            self.interfaces = sorted(os.linesep.join([s for s in stdout.splitlines() if s]).split('\n'))
        except Exception as e:
            self.stat.level = DiagnosticStatus.ERROR
            self.stat.message = "IwConfigLocal init exception: %s" % e
            rospy.logerr("IwConfigLocal init exception: %s" % e)

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
                try:
                    stdout = stdout.decode()  #python3
                except (UnicodeDecodeError, AttributeError):
                    pass

                if res != 0:
                    self.stat.values.append(KeyValue(key = 'iwconfig stderr', value = stderr))
                    self.stat.values.append(KeyValue(key = 'iwconfig stdout', value = stdout))
                else:
                    self.stat.values += self._parse_info(stdout)
            except Exception as e:
                self.stat.level = DiagnosticStatus.ERROR
                self.stat.message = "IwConfigLocal update exception: %s" % e
                rospy.logerr("IwConfigLocal update exception: %s" % e)
                self.stat.values.append(KeyValue(key = 'update exception', value = str(e)))
                self.stat.values.append(KeyValue(key = 'Traceback', value = str(traceback.format_exc())))

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
        except Exception as e:
            self.stat.level = DiagnosticStatus.ERROR
            self.stat.message = "IwConfigSSH init exception: %s" % e
            rospy.logerr("IwConfigSSH init exception: %s" % e)

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
            except Exception as e:
                self.stat.level = DiagnosticStatus.ERROR
                self.stat.message = "IwConfigSSH update exception: %s" % e
                rospy.logerr("IwConfigSSH update exception: %s" % e)
                self.stat.values.append(KeyValue(key = 'update exception', value = str(e)))
                self.stat.values.append(KeyValue(key = 'Traceback', value = str(traceback.format_exc())))

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
            except Exception as e:
                msg = "Cannot connect to router via ssh. Please check if ssh key of user '{}' is contained in the router configuration or password is provided. Error message: {}".format(getpass.getuser(), e)
                rospy.logerr(msg)
                self._wlan_stat.level = DiagnosticStatus.ERROR
                self._wlan_stat.message = msg
                self._wlan_stat.values = [ KeyValue(key = 'Exception', value = str(e)), KeyValue(key = 'Traceback', value = str(traceback.format_exc())) ]
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

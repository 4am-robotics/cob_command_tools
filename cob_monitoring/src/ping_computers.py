#!/usr/bin/env python
#
# Copyright 2018 Mojin Robotics GmbH
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

import rospy
import subprocess

from diagnostic_msgs.msg import DiagnosticStatus, KeyValue

class ping_computers:
    def __init__(self):
        self.diagnostics_publisher = rospy.Publisher('/diagnostics', DiagnosticStatus,queue_size=1)
        self.diagnostics_frequency = rospy.get_param('~diagostics_frequency', None)
        rospy.Timer(rospy.Duration(self.diagnostics_frequency), self.publish_diagnostics)

        self.ping_frequency = rospy.get_param('~ping_frequency', None)
        rospy.Timer(rospy.Duration(self.ping_frequency), self.ping_hosts)
        self.ping_number_packages = rospy.get_param('~ping_number_packages', None)

        self.hosts = rospy.get_param('~host_ips', None)
        if not type(self.hosts) == list:
            rospy.logerr("Ping Computers: hosts are not a list: %s", str(self.hosts))
            exit
        # use all but last digits from IP address
        self.ip_prefix = ".".join(subprocess.check_output(['hostname', '-I']).split(".")[:-1])
        rospy.logdebug("Ping Computers: computer IP: %s, IP prefix: %s",subprocess.check_output(['hostname', '-I']), self.ip_prefix)
        self.hosts_pingable = {}
        for host in self.hosts:
            host_ip = self.ip_prefix+"."+str(host)
            self.hosts_pingable.update({host_ip:False})
        # ping initially to cover the case diagostics_frequency > ping_frequency
        self.ping_hosts(None)

    def ping_hosts(self, event):
        for host_ip, pingable in self.hosts_pingable.iteritems():
            pingable = self.ping(host_ip)
            rospy.logdebug("Ping Computers: host: %s, pingable %s", str(host_ip), str(pingable))
            self.hosts_pingable[host_ip] = pingable

    def ping(self, host):
        """
        Returns True if host (str) responds to a ping request.
        """
        rospy.logdebug("Ping Computers: pinging %s", host)
        # Building the command. e.g.: "ping -c 1 google.com"
        command = ['ping', '-c', str(self.ping_number_packages), host, '-q']
        # if ping returns 0, ping was successful
        return subprocess.call(command) == 0

    def publish_diagnostics(self, event):
        stat = DiagnosticStatus()
        stat.name = "ping computers"
        stat.hardware_id = "None"

        all_pingable = True
        # add info for all hosts
        for host_ip, pingable in self.hosts_pingable.iteritems():
            stat.values.append(KeyValue(key=host_ip, value=str(pingable)))
            # only valid at the end if all computers were pingable
            all_pingable = all_pingable and pingable

        if all_pingable:
            stat.level = DiagnosticStatus.OK
            stat.message = "Ping Computers: all computers pingable"
        else:
            stat.level = DiagnosticStatus.ERROR
            stat.message = "Ping Computers: not all computers pingable"
        rospy.logdebug("Ping Computers: DiagnosticStatus:\n%s", str(stat))
        self.diagnostics_publisher.publish(stat)

if __name__ == '__main__':
    rospy.init_node('ping_computers')
    print "\nPing Computers node is running ..."
    ping = ping_computers()     
    rospy.spin()
 

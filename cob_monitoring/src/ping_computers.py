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
import sys

from diagnostic_msgs.msg import DiagnosticStatus, DiagnosticArray, KeyValue
from std_msgs.msg import String

class ping_computers:
    def __init__(self):
        self.ping_number_packages = rospy.get_param('~ping_number_packages', None)

        self.hosts = rospy.get_param('~host_ips', None)
        if not type(self.hosts) == list:
            rospy.logerr("Ping Computers: hosts are not a list: %s", str(self.hosts))
            sys.exit

        self.host_pc = rospy.get_param('~host_pc', None)
        self.hosts_info = {}
        self.hosts_unreachable = []
        diagnostics_frequency = rospy.get_param('~diagostics_frequency', None)
        if diagnostics_frequency < 0:
            rospy.logerr("Ping Computers: invalid frequency: %f", diagnostics_frequency)
            sys.exit

        if self.host_pc == "ROBOT":
            # on robot, publish diagnostics
            self.diagnostics_publisher = rospy.Publisher('/diagnostics', DiagnosticArray, queue_size=1)
            rospy.Timer(rospy.Duration(1.0/diagnostics_frequency), self.publish_diagnostics)
            # use all but last digits from local IP address
            ip_prefix = ".".join(subprocess.check_output(['hostname', '-I']).split(".")[:-1])
            rospy.loginfo("Ping Computers: ROBOT mode\nlocal computer IP: %s, subnet: %s",subprocess.check_output(['hostname', '-I']), ip_prefix)
            for host in self.hosts:
                host_ip = "10.4.9"+"."+str(host)
                #host_ip = ip_prefix+"."+str(host)
                self.hosts_info.update({host_ip:{}})
        elif self.host_pc == "ROBOWATCH":
            # publish directly to topic when running on robowatch pc
            publish_topic = rospy.get_param('~publish_topic', None)
            self.slack_publisher = rospy.Publisher(publish_topic, String, queue_size=1)
            rospy.Timer(rospy.Duration(1.0/diagnostics_frequency), self.publish_slack)
            # use IP from subnets param
            subnets = rospy.get_param('~subnets', None)
            for ip_prefix in subnets:
                for host in self.hosts:
                    host_ip = ip_prefix+"."+str(host)
                    self.hosts_info.update({host_ip:{}})
        else:
            rospy.logerr("Ping Computers: specified host pc not available: got %s, but available are ROBOT and ROBOWATCH", str(self.host_pc))
            sys.exit

        # ping initially to avoid publishing an error when diagostics_frequency > ping_frequency
        self.ping_hosts(None)
        # start timer after all variables have been initialized
        ping_frequency = rospy.get_param('~ping_frequency', None)
        if ping_frequency > 0:
            rospy.Timer(rospy.Duration(1.0/ping_frequency), self.ping_hosts)
        else:
            rospy.logerr("Ping Computers: invalid frequency: %f", ping_frequency)
            sys.exit

    def ping_hosts(self, event):
        rospy.logdebug("Ping Computers:  hosts to be pinged:\n%s", self.hosts_info.keys())
        for host_ip, pingable in self.hosts_info.iteritems():
            self.ping(host_ip)

    def ping(self, host):
        rospy.logdebug("Ping Computers: pinging %s", host)
        # Building the command. e.g.: "ping -c 1 google.com"
        command = ['ping', '-c', str(self.ping_number_packages), host, '-q']
        process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        returncode = process.wait()
        rospy.logdebug("Ping Computers: ping returned %d", returncode)
        ping_result = process.stdout.read()
        rospy.logdebug("Ping Computers: host: %s, pingable %s", str(host), str(returncode == 0))
        self.hosts_info[host].update({"pingable":returncode==0})

        lines = ping_result.split("\n")
        for line in lines:
            if "transmitted" in line:
                infos = line.split(",")
                for info in infos:
                    words = info.split(" ")
                    for word in words:
                        if word.isdigit():
                            if "transmitted" in info:
                                rospy.logdebug("Ping Computers: %d packages transmitted", int(word))
                                self.hosts_info[host].update({"transmitted":int(word)})
                            elif "received" in info:
                                rospy.logdebug("Ping Computers: %d packages received", int(word))
                                self.hosts_info[host].update({"received":int(word)})
                            else:
                                rospy.logerr("Ping Computers: cold not match %s to received or transmitted packages", info)
                        elif "%" in word:
                            if "packet loss" in info:
                                rospy.logdebug("Ping Computers: %f percent packages lost", float(word.replace("%", "")))
                                self.hosts_info[host].update({"lost":float(word.replace("%", ""))})
                            else:
                                rospy.logerr("Ping Computers: cold not match %s to packages lost", info)
                        elif "ms" in word:
                            if "time" in info:
                                rospy.logdebug("Ping Computers: %fms average duration", float(word.replace("ms", ""))/self.ping_number_packages)
                                self.hosts_info[host].update({"avg_time":float(word.replace("ms", ""))/self.ping_number_packages})
                            else:
                                rospy.logerr("Ping Computers: cold not match %s to time", info)

        rospy.logdebug("Ping Computers: hosts info:\n%s", str(self.hosts_info))
        return

    def publish_diagnostics(self, event):
        diagnostics_array = DiagnosticArray()
        diagnostics_array.header.stamp = rospy.Time.now()

        all_pingable = True
        # add info for all hosts
        for host_ip, info in self.hosts_info.iteritems():
            diagnostics_status = DiagnosticStatus()
            diagnostics_status.name = "ping computers"
            diagnostics_status.hardware_id = "None"
            if "pingable" in info.keys() and info["pingable"]:
                diagnostics_status.level = DiagnosticStatus.OK
                diagnostics_status.message = "Ping Computers: " + host_ip + " pingable"
            elif "pingable" in info.keys() and not info["pingable"]:
                diagnostics_status.level = DiagnosticStatus.ERROR
                diagnostics_status.message = "Ping Computers: " + host_ip + " not pingable"
            diagnostics_array.status.append(diagnostics_status)
            if "pingable" in info.keys():
                diagnostics_status.values.append(KeyValue(key="pingable", value=str(info["pingable"])))
                # only valid at the end if all computers were pingable
                all_pingable = all_pingable and info["pingable"]
                if "lost" in info.keys() and "avg_time" in info.keys():
                    diagnostics_status.values.append(KeyValue(key="package loss", value=str(info["lost"])))
                    diagnostics_status.values.append(KeyValue(key="average time", value=str(info["avg_time"])))
        rospy.logdebug("Ping Computers: DiagnosticStatus:\n%s", str(diagnostics_array))
        self.diagnostics_publisher.publish(diagnostics_array)

    def publish_slack(self, event):
        hosts_unreachable = []
        message = String()
        message.data = "ERROR: Ping Computers\n"
        message.data += '----------- unreachable hosts -----------\n'
        for host_ip, info in self.hosts_info.iteritems():
            if "pingable" in info.keys() and not info["pingable"]:
                message.data += host_ip + "cannot be pinged\n"
                hosts_unreachable.append(host_ip)
        # check if still the same hosts are unrechable
        if (len(hosts_unreachable) > 0) and (set(hosts_unreachable) != set(self.hosts_unreachable)):
            self.hosts_unreachable = hosts_unreachable
            rospy.logdebug("Ping Computers: not all computers pingable, slacking:\n%s", message.data)
            self.slack_publisher.publish(message)
        else:
            rospy.logdebug("Ping Computers: all computers pingable except: %s", str(hosts_unreachable))

if __name__ == '__main__':
    rospy.init_node('ping_computers')
    print "\nPing Computers node is running ..."
    ping = ping_computers()     
    rospy.spin()
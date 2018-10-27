#!/usr/bin/env python

import rospy
from subprocess import call as system_call  # Execute a shell command

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from diagnostic_updater import Updater

class ping_computers:
    def __init__(self):
        self.diagnostics_frequency = rospy.get_param('~diagostics_frequency', None)
        self.ping_number_packages = rospy.get_param('~ping_number_packages', None)
        self.hosts = rospy.get_param('~hosts', None)
        if not type(self.hosts) == list:
            rospy.logerr("Ping Computers: hosts are not a list: %s", str(self.hosts))
        self.updater = Updater()
        self.updater.setHardwareID("ping computers")
        self.ping_successful = False
        rospy.Timer(rospy.Duration(self.diagnostics_frequency), self.publish_diagnostics)

    def ping(self, host):
        """
        Returns True if host (str) responds to a ping request.
        Remember that a host may not respond to a ping (ICMP) request even if the host name is valid.
        """
        # Building the command. e.g.: "ping -c 1 google.com"
        command = ['ping', '-c', str(self.ping_number_packages), host]
        # if ping returns 0, ping was successful
        return system_call(command) == 0

    def publish_diagnostics(self, event):
        self.updater.add("ping computers", self.produce_diagnostics)
        self.updater.update()

    def produce_diagnostics(self, stat):
        all_pingable = True
        # add info for all hosts
        for host in self.hosts:
            pingable = self.ping(host)
            stat.add(host, pingable)
            # only valid at the end if all computers were pingable
            all_pingable = all_pingable and pingable
        if all_pingable:
            stat.summary(DiagnosticStatus.OK, "Ping Computers: all computers pingable")
        else:
            stat.summary(DiagnosticStatus.ERROR, "Ping Computers: not all computers pingable")
        return stat

if __name__ == '__main__':
    rospy.init_node('ping_computers')
    print "\nPing Computers node is running ..."
    ping = ping_computers()     
    rospy.spin()
 

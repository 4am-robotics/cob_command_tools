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

from __future__ import print_function

import sys
import socket
from subprocess import Popen, PIPE
import time
import re

import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

class NtpMonitor():
    def __init__(self, argv=sys.argv):
        rospy.init_node("ntp_monitor")
        self.parse_args(argv)

        stat = DiagnosticStatus()
        stat.level = DiagnosticStatus.WARN
        stat.name = '%s NTP Offset' % self.diag_hostname
        stat.message = 'No Data'
        stat.hardware_id = self.diag_hostname
        stat.values = []
        self.msg = DiagnosticArray()
        self.msg.header.stamp = rospy.get_rostime()
        self.msg.status = [stat]

        self.update_diagnostics()

        self.diag_pub = rospy.Publisher("/diagnostics", DiagnosticArray, queue_size=1)
        self.diag_timer = rospy.Timer(rospy.Duration(1.0), self.publish_diagnostics)
        self.monitor_timer = rospy.Timer(rospy.Duration(60.0), self.update_diagnostics)

    def update_diagnostics(self, event=None):
        stat = DiagnosticStatus()
        stat.level = DiagnosticStatus.WARN
        stat.name = '%s NTP Offset' % self.diag_hostname
        stat.message = 'No Data'
        stat.hardware_id = self.diag_hostname
        stat.values = []
        
        try:
            for st,host,off in [(stat, self.ntp_server, self.offset)]:
                p = Popen(["ntpdate", "-q", host], stdout=PIPE, stdin=PIPE, stderr=PIPE)
                retcode = p.wait()
                stdout, stderr = p.communicate()

                if retcode != 0:
                    st.level = DiagnosticStatus.ERROR
                    st.message = 'ntpdate Error'
                    st.values = [ KeyValue(key = 'ntpdate Error', value = stderr),
                                  KeyValue(key = 'Output', value = stdout) ]
                    continue

                measured_offset = float(re.search("offset (.*),", stdout).group(1))*1000000

                st.level = DiagnosticStatus.OK
                st.message = "OK"
                st.values = [ KeyValue("NTP Server" , self.ntp_server),
                              KeyValue("Offset (us)", str(measured_offset)),
                              KeyValue("Offset tolerance (us)", str(off)),
                              KeyValue("Offset tolerance (us) for Error", str(self.error_offset)) ]

                if (abs(measured_offset) > off):
                    st.level = DiagnosticStatus.WARN
                    st.message = "NTP Offset Too High"
                if (abs(measured_offset) > self.error_offset):
                    st.level = DiagnosticStatus.ERROR
                    st.message = "NTP Offset Too High"

        except Exception, e:
            stat = DiagnosticStatus.ERROR
            stat.message = 'ntpdate Exception'
            stat.values = [ KeyValue(key = 'Exception', value = str(e)) ]

        self.msg = DiagnosticArray()
        self.msg.header.stamp = rospy.get_rostime()
        self.msg.status = [stat]

    def publish_diagnostics(self, event):
        self.diag_pub.publish(self.msg)

    def parse_args(self, argv=sys.argv):
        import optparse
        parser = optparse.OptionParser(usage="usage: ntp_monitor ntp-hostname []")
        parser.add_option("--offset", dest="offset",
                          action="store", default=500,
                          help="Offset from NTP host", metavar="OFFSET")
        parser.add_option("--error-offset", dest="error_offset",
                          action="store", default=5000000,
                          help="Offset from NTP host. Above this is error", metavar="OFFSET")
        parser.add_option("--diag-hostname", dest="diag_hostname",
                          help="Computer name in diagnostics output (ex: 'c1')",
                          metavar="DIAG_HOSTNAME",
                          action="store", default=None)
        options, args = parser.parse_args(rospy.myargv())

        if (len(args) != 2):
            parser.error("Invalid arguments. Must have HOSTNAME [args]. %s" % args)
            print('Invalid arguments.', file=sys.stderr)
            sys.exit(0)

        try:
            offset = int(options.offset)
            error_offset = int(options.error_offset)
        except:
            parser.error("Offsets must be numbers")
            print('Offsets must be numbers', file=sys.stderr)
            sys.exit(0)

        self.ntp_server = args[1]
        self.diag_hostname = options.diag_hostname
        hostname = socket.gethostname()
        if self.diag_hostname is None:
            self.diag_hostname = hostname

        self.offset = rospy.get_param('~offset', offset)
        self.error_offset = rospy.get_param('~error_offset', error_offset)


if __name__ == "__main__":
    ntp = NtpMonitor(argv=sys.argv)
    rospy.spin()

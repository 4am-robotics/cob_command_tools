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


import sys
import socket
from subprocess import Popen, PIPE
import time
import re

import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

##### monkey-patch to suppress threading error message in python 2.7.3
##### See http://stackoverflow.com/questions/13193278/understand-python-threading-bug
if sys.version_info[:3] == (2, 7, 3):
    import threading
    threading._DummyThread._Thread__stop = lambda x: 42
#####

NAME = 'ntp_monitor'

def ntp_monitor(ntp_hostname, offset=500, self_offset=500, diag_hostname = None, error_offset = 5000000):
    pub = rospy.Publisher("/diagnostics", DiagnosticArray,queue_size=50 )

    hostname = socket.gethostname()
    if diag_hostname is None:
        diag_hostname = hostname

    stat = DiagnosticStatus()
    stat.level = 0
    stat.name = "NTP offset from "+ diag_hostname + " to " + ntp_hostname
    stat.message = "OK"
    stat.hardware_id = hostname
    stat.values = []

    self_stat = DiagnosticStatus()
    self_stat.level = DiagnosticStatus.OK
    self_stat.name = "NTP self-offset for "+ diag_hostname
    self_stat.message = "OK"
    self_stat.hardware_id = hostname
    self_stat.values = []

    while not rospy.is_shutdown():
        for st,host,off in [(stat,ntp_hostname,offset), (self_stat, hostname,self_offset)]:
            try:
                p = Popen(["ntpdate", "-q", host], stdout=PIPE, stdin=PIPE, stderr=PIPE)
                res = p.wait()
                (o,e) = p.communicate()
            except OSError, (errno, msg):
                if errno == 4:
                    break #ctrl-c interrupt
                else:
                    raise
            if (res == 0):
                measured_offset = float(re.search("offset (.*),", o).group(1))*1000000

                st.level = DiagnosticStatus.OK
                st.message = "OK"
                st.values = [ KeyValue("Offset (us)", str(measured_offset)),
                              KeyValue("Offset tolerance (us)", str(off)),
                              KeyValue("Offset tolerance (us) for Error", str(error_offset)) ]

                if (abs(measured_offset) > off):
                    st.level = DiagnosticStatus.WARN
                    st.message = "NTP Offset Too High"
                if (abs(measured_offset) > error_offset):
                    st.level = DiagnosticStatus.ERROR
                    st.message = "NTP Offset Too High"

            else:
                st.level = DiagnosticStatus.ERROR
                st.message = "Error Running ntpdate. Returned %d" % res
                st.values = [ KeyValue("Offset (us)", "N/A"),
                              KeyValue("Offset tolerance (us)", str(off)),
                              KeyValue("Offset tolerance (us) for Error", str(error_offset)),
                              KeyValue("Output", o),
                              KeyValue("Errors", e) ]


        msg = DiagnosticArray()
        msg.header.stamp = rospy.get_rostime()
        msg.status = [stat, self_stat]
        pub.publish(msg)
        time.sleep(1)

def ntp_monitor_main(argv=sys.argv):
    import optparse
    parser = optparse.OptionParser(usage="usage: ntp_monitor ntp-hostname []")
    parser.add_option("--offset-tolerance", dest="offset_tol",
                      action="store", default=500,
                      help="Offset from NTP host", metavar="OFFSET-TOL")
    parser.add_option("--error-offset-tolerance", dest="error_offset_tol",
                      action="store", default=5000000,
                      help="Offset from NTP host. Above this is error", metavar="OFFSET-TOL")
    parser.add_option("--self_offset-tolerance", dest="self_offset_tol",
                      action="store", default=500,
                      help="Offset from self", metavar="SELF_OFFSET-TOL")
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
        offset = int(options.offset_tol)
        self_offset = int(options.self_offset_tol)
        error_offset = int(options.error_offset_tol)
    except:
        parser.error("Offsets must be numbers")
        print('Offsets must be numbers', file=sys.stderr)
        sys.exit(0)

    try:
        rospy.init_node(NAME, anonymous=True)
    except rospy.exceptions.ROSInitException:
        print('NTP monitor is unable to initialize node. Master may not be running.', file=sys.stderr)
        sys.exit(0)

    offset = rospy.get_param('~offset_tolerance', offset)
    error_offset_tol = rospy.get_param('~error_offset_tolerance', error_offset_tol)
    self_offset_tol = rospy.get_param('~self_offset_tolerance', self_offset_tol)
    print("NTP parameters:")
    print offset
    print error_offset_tol
    print self_offset_tol

    ntp_monitor(args[1], offset, self_offset, options.diag_hostname, error_offset)


if __name__ == "__main__":
    try:
        ntp_monitor_main(rospy.myargv())
    except KeyboardInterrupt: pass
    except SystemExit: pass
    except:
        import traceback
        traceback.print_exc()

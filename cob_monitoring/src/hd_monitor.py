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


from __future__ import with_statement, print_function

import traceback
import sys, os, time
import subprocess
import socket

import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

stat_dict = { DiagnosticStatus.OK: 'OK', DiagnosticStatus.WARN: 'Warning', DiagnosticStatus.ERROR: 'Error' }
usage_dict = { DiagnosticStatus.OK: 'OK', DiagnosticStatus.WARN: 'Low Disk Space', DiagnosticStatus.ERROR: 'Very Low Disk Space' }

class hd_monitor():
    def __init__(self, hostname, diag_hostname, home_dir = ''):
        self._hostname = hostname
        self._home_dir = home_dir

        self.unit = 'G'
        self.low_hd_level = rospy.get_param('~low_hd_level', 5) #self.unit
        self.critical_hd_level = rospy.get_param('~critical_hd_level', 1) #self.unit

        self._usage_stat = DiagnosticStatus()
        self._usage_stat.level = DiagnosticStatus.WARN
        self._usage_stat.hardware_id = hostname
        self._usage_stat.name = '%s HD Usage' % diag_hostname
        self._usage_stat.message = 'No Data'
        self._usage_stat.values = []

        self._io_stat = DiagnosticStatus()
        self._io_stat.name = '%s HD IO' % diag_hostname
        self._io_stat.level = DiagnosticStatus.WARN
        self._io_stat.hardware_id = hostname
        self._io_stat.message = 'No Data'
        self._io_stat.values = []

        self._diag_pub = rospy.Publisher('/diagnostics', DiagnosticArray, queue_size=1)
        self._publish_timer = rospy.Timer(rospy.Duration(1.0), self.publish_stats)
        self._usage_timer = rospy.Timer(rospy.Duration(5.0), self.check_disk_usage)
        self._io_timer = rospy.Timer(rospy.Duration(5.0), self.check_io_stat)

    def check_io_stat(self, event):
        diag_vals = []
        diag_msg = 'OK'
        diag_level = DiagnosticStatus.OK

        try:
            p = subprocess.Popen('iostat -d',
                                 stdout = subprocess.PIPE,
                                 stderr = subprocess.PIPE, shell = True)
            stdout, stderr = p.communicate()
            retcode = p.returncode

            if retcode != 0:
                diag_level = DiagnosticStatus.ERROR
                diag_msg = 'HD IO Error'
                diag_vals = [ KeyValue(key = 'HD IO Error', value = stderr),
                              KeyValue(key = 'Output', value = stdout) ]
                return (diag_vals, diag_msg, diag_level)

            for index, row in enumerate(stdout.split('\n')):
                if index < 3:
                    continue

                lst = row.split()
                #Device:            tps    kB_read/s    kB_wrtn/s    kB_read    kB_wrtn
                device = lst[0]
                tps = lst[1]
                kB_read_s = lst[2]
                kB_wrtn_s = lst[3]
                kB_read = lst[4]
                kB_wrtn = lst[5]

                diag_vals.append(KeyValue(
                        key = '%s tps' % device, value=tps))
                diag_vals.append(KeyValue(
                        key = '%s kB_read/s' % device, value=kB_read_s))
                diag_vals.append(KeyValue(
                        key = '%s kB_wrtn/s' % device, value=kB_wrtn_s))
                diag_vals.append(KeyValue(
                        key = '%s kB_read' % device, value=kB_read))
                diag_vals.append(KeyValue(
                        key = '%s kB_wrtn' % device, value=kB_wrtn))

        except Exception, e:
            diag_level = DiagnosticStatus.ERROR
            diag_msg = 'HD IO Exception'
            diag_vals = [ KeyValue(key = 'Exception', value = traceback.format_exc()) ]

        self._io_stat.values = diag_vals
        self._io_stat.message = diag_msg
        self._io_stat.level = diag_level

    def check_disk_usage(self, event):
        diag_vals = []
        diag_message = ''
        diag_level = DiagnosticStatus.OK
        try:
            p = subprocess.Popen(["df", "--print-type", "--portability", "--block-size=1"+self.unit, self._home_dir],
                                 stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            stdout, stderr = p.communicate()
            retcode = p.returncode

            if retcode != 0:
                diag_level = DiagnosticStatus.ERROR
                diag_message = 'HD Usage Error'
                diag_vals = [ KeyValue(key = 'HD Usage Error', value = stderr),
                              KeyValue(key = 'Output', value = stdout) ]

            else:
                diag_vals.append(KeyValue(key = 'Disk Space Reading', value = 'OK'))
                row_count = 0
                for row in stdout.split('\n'):
                    if len(row.split()) < 2:
                        continue
                    if not unicode(row.split()[2]).isnumeric() or float(row.split()[2]) < 10: # Ignore small drives
                        continue

                    row_count += 1
                    #Filesystem     Type 1073741824-blocks  Used Available Capacity Mounted on
                    name = row.split()[0]
                    hd_type = row.split()[1]
                    size = row.split()[2]
                    used = row.split()[3]
                    available = row.split()[4]
                    capacity = row.split()[5]
                    mount_pt = row.split()[6]

                    if (float(available) > self.low_hd_level):
                        level = DiagnosticStatus.OK
                    elif (float(available) > self.critical_hd_level):
                        level = DiagnosticStatus.WARN
                    else:
                        level = DiagnosticStatus.ERROR

                    diag_vals.append(KeyValue(
                            key = 'Disk %d Name' % row_count, value = name))
                    diag_vals.append(KeyValue(
                            key = 'Disk %d Size' % row_count, value = size + ' ' +self.unit))
                    diag_vals.append(KeyValue(
                            key = 'Disk %d Used' % row_count, value = used + ' ' +self.unit))
                    diag_vals.append(KeyValue(
                            key = 'Disk %d Available' % row_count, value = available + ' ' +self.unit))
                    diag_vals.append(KeyValue(
                            key = 'Disk %d Capacity' % row_count, value = capacity))
                    diag_vals.append(KeyValue(
                            key = 'Disk %d Status' % row_count, value = stat_dict[level]))
                    diag_vals.append(KeyValue(
                            key = 'Disk %d Mount Point' % row_count, value = mount_pt))

                    diag_level = max(diag_level, level)
                    diag_message = usage_dict[diag_level]

        except Exception, e:
            diag_level = DiagnosticStatus.ERROR
            diag_message = 'HD Usage Exception'
            diag_vals = [ KeyValue(key = 'Exception', value = traceback.format_exc()) ]

        self._usage_stat.values = diag_vals
        self._usage_stat.message = diag_message
        self._usage_stat.level = diag_level

    def publish_stats(self, event):
        msg = DiagnosticArray()
        msg.header.stamp = rospy.get_rostime()
        msg.status.append(self._usage_stat)
        self._diag_pub.publish(msg)


##\todo Need to check HD input/output too using iostat

if __name__ == '__main__':
    hostname = socket.gethostname()

    import optparse
    parser = optparse.OptionParser(usage="usage: hd_monitor.py --diag-hostname=X --directory=/name_of_dir")
    parser.add_option("--diag-hostname", 
                      dest="diag_hostname",
                      help="Computer name in diagnostics output (ex: 'b1' for the base PC, 'h32' for the head PC and so on)",
                      metavar="DIAG_HOSTNAME",
                      action="store", 
                      default=False)
    parser.add_option("--directory", 
                      dest="directory",
                      help="Enter the directory name (ex: /directory/sub_directory)",
                      metavar="DIR_NAME",
                      default="/") ## taking the root directory as the default directory for checking HDD usage
    options, args = parser.parse_args(rospy.myargv())
    if len(sys.argv[1:]) == 0:
        parser.error("argument not found.")

    try:
        ## the hostname consists of hiphens, 
        ## replacing hiphens "-" with underscore "_", in order to have legal node name
        node_name = ("hd_monitor_"+hostname).replace ("-", "_")
        rospy.init_node(node_name)
    except rospy.exceptions.ROSInitException:
        print('HD monitor is unable to initialize node. Master may not be running.', file=sys.stderr)
        sys.exit(0)

    hd_monitor = hd_monitor(hostname, options.diag_hostname, options.directory)
    rospy.spin()

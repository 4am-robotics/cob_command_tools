#!/usr/bin/python
#################################################################
##\file
#
# \note
#   Copyright (c) Felix Messmer \n
#   Fraunhofer Institute for Manufacturing Engineering
#   and Automation (IPA) \n
#
#   All rights reserved. \n\n
#
#################################################################
#
# \note
#   Repository name: cob_command_tools
# \note
#   ROS package name: cob_helper_tools
#
# \author
#   Author: Felix Messmer
#
# \date Date of creation: January 2017
#
# \brief
#   A script for faking diagnostics
#
#################################################################

import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus

class FakeDiagnostics():
    def __init__(self, options):
        self._options = options
        self._last_publish_time = 0
        self._fake_diag_pub = rospy.Publisher('/diagnostics', DiagnosticArray, queue_size=1)
        rospy.Timer(rospy.Duration(1.0), self.publish_stats)

    def publish_stats(self, event):
        msg = DiagnosticArray()
        msg.header.stamp = rospy.get_rostime()
        # Add all fake
        hostname_list = self._options.diag_hostnames.split(", ")
        for hostname in hostname_list:
            status = DiagnosticStatus()
            status.name = hostname
            status.level = DiagnosticStatus.OK
            status.message = "fake diagnostics"
            status.hardware_id = hostname
            msg.status.append(status)

        self._fake_diag_pub.publish(msg)

if __name__ == '__main__':

    import optparse
    parser = optparse.OptionParser(usage="usage: fake_diagnostics.py [--diag-hostnames=hostname1, hostname2, ...]")
    parser.add_option("--diag-hostnames", dest="diag_hostnames",
                      help="Fake Diagnostics")
    options, args = parser.parse_args(rospy.myargv())

    rospy.init_node('fake_diagnostics')

    fake_diagnostics = FakeDiagnostics(options)

    rospy.spin()

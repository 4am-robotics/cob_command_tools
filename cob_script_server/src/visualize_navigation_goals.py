#!/usr/bin/env python
import sys
import rospy
import tf
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point

class VisualizerNavigationGoals():
    def __init__(self):
        self.pubGoals = rospy.Publisher('visualize_navigation_goals', MarkerArray, queue_size=1, latch=True)

    def pubMarker(self):
        navigation_goals = rospy.get_param("/script_server/base", {})

        markerarray = MarkerArray()
        i=0
        for name, pose in navigation_goals.items():

            # check if pose is valid
            if len(pose) != 3:
                continue
            
            # arrow
            marker_arrow = Marker()
            marker_arrow.header.stamp = rospy.Time.now()
            marker_arrow.header.frame_id = "/map"
            marker_arrow.ns = "/pose"
            marker_arrow.id = i
            marker_arrow.type = Marker.ARROW
            marker_arrow.action = Marker.ADD
            marker_arrow.scale.x = 1.0
            marker_arrow.scale.y = 0.1
            marker_arrow.scale.z = 1.0
            marker_arrow.color.r = 0.0
            marker_arrow.color.g = 0.0
            marker_arrow.color.b = 1.0
            marker_arrow.color.a = 1.0
            marker_arrow.pose.position.x = pose[0]
            marker_arrow.pose.position.y = pose[1]
            marker_arrow.pose.position.z = 0.2
            quaternion = tf.transformations.quaternion_from_euler(0, 0, pose[2])
            marker_arrow.pose.orientation.x = quaternion[0]
            marker_arrow.pose.orientation.y = quaternion[1]
            marker_arrow.pose.orientation.z = quaternion[2]
            marker_arrow.pose.orientation.w = quaternion[3]
            markerarray.markers.append(marker_arrow)
            
            # text
            marker_text = Marker()
            marker_text.header.stamp = rospy.Time.now()
            marker_text.header.frame_id = "/map"
            marker_text.ns = "/name"
            marker_text.id = i + 1000000
            marker_text.type = Marker.TEXT_VIEW_FACING
            marker_text.action = Marker.ADD
            marker_text.scale.z = 0.5
            marker_text.color.r = 0.0
            marker_text.color.g = 0.0
            marker_text.color.b = 1.0
            marker_text.color.a = 1.0
            marker_text.pose.position.x = pose[0]
            marker_text.pose.position.y = pose[1] + 0.2
            marker_text.pose.position.z = 0.2
            quaternion = tf.transformations.quaternion_from_euler(0, 0, pose[2])
            marker_text.pose.orientation.x = quaternion[0]
            marker_text.pose.orientation.y = quaternion[1]
            marker_text.pose.orientation.z = quaternion[2]
            marker_text.pose.orientation.w = quaternion[3]
            marker_text.text = name
            markerarray.markers.append(marker_text)
            
            i = i + 1

        self.pubGoals.publish(markerarray)

if __name__ == "__main__":
    rospy.init_node('navigation_goal_visualizer')
    p = VisualizerNavigationGoals()
    r = rospy.Rate(1)
    while not rospy.is_shutdown():
        p.pubMarker()
        r.sleep()

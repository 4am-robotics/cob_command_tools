#!/usr/bin/env python

import signal
import subprocess

import rospy
import tf2_ros
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped, Vector3
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from nav_msgs.msg import Path
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import (
    InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback, Marker
)


class InteractivePathPublisher():
    def __init__(self):
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        map_base_link_xyz_quat = rospy.get_param("~map_base_link_xyz_quat")

        self.path_topic = rospy.get_param("~path_topic", "/path")
        self.path_pub = rospy.Publisher(self.path_topic, Path, queue_size=1)

        self.target_frame_id = rospy.get_param("~target_frame_id", "base_link")
        self.path_frame_id = rospy.get_param("~path_frame_id", "map")
        self.path = self.get_path(self.path_frame_id)

        interactive_marker_server = rospy.get_param(
            "~interactive_marker_server", "interactive_marker"
        )
        interactive_marker_name = rospy.get_param("~interactive_marker_name", "path_marker")
        interactive_marker_description = rospy.get_param(
            "~interactive_marker_description", "Drag to create path"
        )
        self.server = InteractiveMarkerServer(interactive_marker_server)
        self.init_interactive_marker(
            interactive_marker_name, interactive_marker_description, self.path_frame_id,
            init_position=Vector3(
                0.9 * map_base_link_xyz_quat[0],
                0.9 * map_base_link_xyz_quat[1],
                0.2
            )
        )

        self.tf_path_target = self.get_tf_map_base_link(
            self.path_frame_id, self.target_frame_id, map_base_link_xyz_quat
        )

    def get_path(self, path_frame_id: str):
        path = Path()
        path.header.frame_id = path_frame_id
        return path

    def init_interactive_marker(
        self, interactive_marker_name: str, interactive_marker_description: str, frame_id: str,
        init_position: Vector3,
        scale: Vector3 = Vector3(0.4, 0.4, 0.4), color: ColorRGBA = ColorRGBA(0, 1, 0, 1)
    ):
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = frame_id
        int_marker.name = interactive_marker_name
        int_marker.description = interactive_marker_description
        int_marker.pose.position = init_position

        marker = Marker()
        marker.type = Marker.SPHERE
        marker.scale = scale
        marker.color = color

        control = InteractiveMarkerControl()
        control.always_visible = True
        control.markers.append(marker)
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        int_marker.controls.append(control)

        self.server.insert(int_marker, self.marker_callback)
        self.server.applyChanges()

    def marker_callback(self, feedback: InteractiveMarkerFeedback):
        if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            self.publish_path(feedback.pose)

    def publish_path(self, target_pose: Pose):
        pose = PoseStamped()
        pose.header.frame_id = self.path.header.frame_id
        pose.header.stamp = rospy.Time.now()
        pose.pose = target_pose
        self.path.header.stamp = rospy.Time.now()
        self.path.poses = [pose]
        self.path_pub.publish(self.path)

    def get_tf_map_base_link(
            self, frame_id: str, child_frame_id: str, map_base_link_xyz_quat: list
    ):
        transform = TransformStamped()
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = frame_id
        transform.child_frame_id = child_frame_id
        transform.transform.translation.x = map_base_link_xyz_quat[0]
        transform.transform.translation.y = map_base_link_xyz_quat[1]
        transform.transform.translation.z = map_base_link_xyz_quat[2]
        transform.transform.rotation.x = map_base_link_xyz_quat[3]
        transform.transform.rotation.y = map_base_link_xyz_quat[4]
        transform.transform.rotation.z = map_base_link_xyz_quat[5]
        transform.transform.rotation.w = map_base_link_xyz_quat[6]
        return transform

    def publish_test_data(self):
        now = rospy.Time.now()
        self.tf_path_target.header.stamp = now
        self.tf_broadcaster.sendTransform([self.tf_path_target])


if __name__ == "__main__":
    rospy.init_node("interactive_path_publisher")

    map_bag_file = rospy.get_param("~map_bag_file")
    rosbag_play_frequency = rospy.get_param("~rosbag_play_frequency", 1)

    rosbag_play_cmd = [
        "rosbag", "play", map_bag_file,
        "--loop", "--hz", str(rosbag_play_frequency)
    ]
    print(f"rosbag_play_cmd: {rosbag_play_cmd}")

    node = InteractivePathPublisher()
    test_data_publish_rate = rospy.get_param("~test_data_publish_rate", 1)
    rate = rospy.Rate(test_data_publish_rate)

    rosbag_process = subprocess.Popen(rosbag_play_cmd)

    try:
        while not rospy.is_shutdown():
            node.publish_test_data()
            rate.sleep()
    except Exception as e:
        print(f"Exception caught: {e}")
        rosbag_process.send_signal(signal.SIGINT)

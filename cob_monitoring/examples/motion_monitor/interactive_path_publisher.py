#!/usr/bin/env python3

import signal
import subprocess

import rospy
import tf2_ros
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped, Vector3, Quaternion
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from nav_msgs.msg import Path
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import (
    InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback, Marker
)


class InteractivePathPublisher():
    def __init__(self):
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        path_target_xyz_quat = rospy.get_param("~path_target_xyz_quat")

        self.path_topic = rospy.get_param("~path_topic", "/path")
        self.path_pub = rospy.Publisher(self.path_topic, Path, queue_size=1)

        self.target_frame_id = rospy.get_param("~target_frame_id", "base_link")
        self.path_frame_id = rospy.get_param("~path_frame_id", "map")
        self.path = self.get_path(self.path_frame_id)

        # interactive marker
        interactive_marker_server = rospy.get_param(
            "~interactive_marker_server", "interactive_marker"
        )
        interactive_marker_name = rospy.get_param("~interactive_marker_name", "path_marker")
        interactive_marker_description = rospy.get_param(
            "~interactive_marker_description", "Drag to create path"
        )
        interactive_marker_scale_xyz = rospy.get_param(
            "~interactive_marker_scale_xyz", [1.5, 0.2, 0.2]
        )
        interactive_marker_color_rgba = rospy.get_param(
            "~interactive_marker_color_rgba", [0, 1, 0, 1]
        )
        interactive_marker_orientation_quat = rospy.get_param(
            "~interactive_marker_orientation_quat", [0, 0.7071, 0, 0.7071]
        )
        self.server = InteractiveMarkerServer(interactive_marker_server)
        self.init_interactive_marker(
            name=interactive_marker_name,
            description=interactive_marker_description,
            frame_id=self.path_frame_id,
            scale=Vector3(*interactive_marker_scale_xyz),
            color=ColorRGBA(*interactive_marker_color_rgba),
            orientation=Quaternion(*interactive_marker_orientation_quat),
            init_position=Vector3(
                0.9 * path_target_xyz_quat[0],
                0.9 * path_target_xyz_quat[1],
                0.2
            )
        )

        self.tf_path_target = InteractivePathPublisher.get_transform_from_pose_xyz_quat(
            self.path_frame_id, self.target_frame_id, path_target_xyz_quat
        )

    def init_interactive_marker(
        self, name: str, description: str, frame_id: str, init_position: Vector3,
        scale: Vector3, color: ColorRGBA, orientation: Quaternion
    ) -> None:
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = frame_id
        int_marker.name = name
        int_marker.description = description
        int_marker.pose.position = init_position

        marker = Marker()
        marker.type = Marker.SPHERE
        marker.scale = scale
        marker.color = color

        control = InteractiveMarkerControl()
        control.always_visible = True
        control.markers.append(marker)
        control.orientation = orientation
        control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        int_marker.controls.append(control)

        self.server.insert(int_marker, self.marker_callback)
        self.server.applyChanges()

    def marker_callback(self, feedback: InteractiveMarkerFeedback) -> None:
        if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            self.publish_path(feedback.pose)

    def publish_path(self, target_pose: Pose) -> None:
        pose = PoseStamped()
        pose.header.frame_id = self.path.header.frame_id
        pose.header.stamp = rospy.Time.now()
        pose.pose = target_pose
        self.path.header.stamp = rospy.Time.now()
        self.path.poses = [pose]
        self.path_pub.publish(self.path)

    def run(self) -> None:
        now = rospy.Time.now()
        self.tf_path_target.header.stamp = now
        self.tf_broadcaster.sendTransform([self.tf_path_target])

    @staticmethod
    def get_path(path_frame_id: str) -> Path:
        path = Path()
        path.header.frame_id = path_frame_id
        return path

    @staticmethod
    def get_transform_from_pose_xyz_quat(
            frame_id: str, child_frame_id: str, xyz_quat: list
    ) -> TransformStamped:
        transform = TransformStamped()
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = frame_id
        transform.child_frame_id = child_frame_id
        transform.transform.translation = Vector3(*xyz_quat[:3])
        transform.transform.rotation = Quaternion(*xyz_quat[3:7])
        return transform


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
    publish_rate = rospy.get_param("~publish_rate", 1)
    rate = rospy.Rate(publish_rate)

    rosbag_process = subprocess.Popen(rosbag_play_cmd)

    try:
        while not rospy.is_shutdown():
            node.run()
            rate.sleep()
    except Exception as e:
        print(f"Exception caught: {e}")
        rosbag_process.send_signal(signal.SIGINT)

#!/usr/bin/env python

import math
from dataclasses import dataclass
from typing import List, Tuple

import rospy
import tf2_ros
from geometry_msgs.msg import Pose, Quaternion, Vector3
from nav_msgs.msg import Path
from std_msgs.msg import ColorRGBA
from tf2_geometry_msgs import do_transform_pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from visualization_msgs.msg import Marker


@dataclass
class MotionCategory:
    name: str
    ranges: List[Tuple[float, float]]  # min max angles for range [-pi, pi], e.g. atan2
    color: ColorRGBA
    yaw: float


class MotionMonitor():
    def __init__(self):
        self.path_topic = rospy.get_param("~path_topic", "/path")
        self.distance_threshold_m = rospy.get_param("~distance_threshold_m", 2.0)
        self.angle_threshold_rad = rospy.get_param(
            "~angle_threshold_rad", math.pi/6)  # 30 degrees in radians
        self.target_frame_id = rospy.get_param("~target_frame_id", "base_link")

        motion_monitor_marker_topic = rospy.get_param(
            "~motion_monitor_marker_topic", "/motion_monitor/motion_monitor_marker"
        )
        self.direction_marker = MotionMonitor.get_direction_marker(self.target_frame_id)
        self.marker_pub = rospy.Publisher(motion_monitor_marker_topic, Marker, queue_size=10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        rospy.Subscriber(self.path_topic, Path, self.path_callback, queue_size=1)

        self.categories = MotionMonitor.get_motion_categories(self.angle_threshold_rad)

    def publish_direction_marker(self, color: ColorRGBA, yaw: float):
        self.direction_marker.color = color
        self.direction_marker.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, yaw))
        self.marker_pub.publish(self.direction_marker)

    def get_transform(self, target_frame: str, source_frame: str):
        try:
            return self.tf_buffer.lookup_transform(
                target_frame, source_frame, rospy.Time(0)
            )
        except (
            tf2_ros.LookupException,  # pylint: disable=no-member
            tf2_ros.ConnectivityException,  # pylint: disable=no-member
            tf2_ros.ExtrapolationException,  # pylint: disable=no-member
        ) as ex:
            rospy.logerr(f"Transform lookup failed: {ex}")
            return None

    def get_valid_category(self, target_yaw: float) -> MotionCategory:
        for category in self.categories:
            valid_category = any([min_angle < target_yaw and target_yaw < max_angle
                                  for min_angle, max_angle in category.ranges])
            if valid_category:
                return category
        return None

    def path_callback(self, path: Path):
        transform_target_path = self.get_transform(self.target_frame_id, path.header.frame_id)

        for pose in path.poses:
            # Transform pose to target frame
            pose_in_target_frame = do_transform_pose(pose, transform_target_path)

            distance = MotionMonitor.get_distance(pose_in_target_frame.pose)
            if distance > self.distance_threshold_m:
                target_yaw = math.atan2(pose_in_target_frame.pose.position.y,
                                        pose_in_target_frame.pose.position.x)

                rospy.loginfo(f"distance: {distance}, target_yaw: {target_yaw}")

                category = self.get_valid_category(target_yaw)
                if category:
                    rospy.loginfo(f"Motion category: {category.name}")
                    self.publish_direction_marker(category.color, category.yaw)
                    break
        else:
            rospy.loginfo(
                "No pose found in the path which is more than the threshold distance away "
                f"({self.distance_threshold_m:.2f}m)"
            )

    @staticmethod
    def get_motion_categories(angle_threshold_rad) -> List[MotionCategory]:
        return [
            MotionCategory(
                "forward",
                [(-angle_threshold_rad, angle_threshold_rad)],
                ColorRGBA(0, 1, 0, 1),
                0),
            MotionCategory(
                "forward-left",
                [(angle_threshold_rad, math.pi / 2)],
                ColorRGBA(0, 0, 1, 1),
                angle_threshold_rad),
            MotionCategory(
                "forward-right", [(-math.pi/2, -angle_threshold_rad)],
                ColorRGBA(1, 0, 1, 1), -angle_threshold_rad),
            MotionCategory(
                "backward",
                [(math.pi - angle_threshold_rad, math.pi),
                 (-math.pi, -math.pi + angle_threshold_rad)],
                ColorRGBA(1, 0, 0, 1),
                math.pi),
            MotionCategory(
                "backward-left",
                [(-math.pi + angle_threshold_rad, -math.pi/2)],
                ColorRGBA(0, 1, 1, 1),
                -math.pi + angle_threshold_rad),
            MotionCategory(
                "backward-right", [(math.pi/2, math.pi - angle_threshold_rad)],
                ColorRGBA(1, 0.5, 0, 1), math.pi - angle_threshold_rad)
        ]

    @staticmethod
    def get_distance(pose: Pose):
        return math.sqrt(pose.position.x**2 + pose.position.y**2)

    @staticmethod
    def get_direction_marker(frame_id: str, scale: Vector3 = Vector3(1.5, 0.2, 0.2)):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose.orientation.z = 0.1
        marker.pose.orientation.w = 1.0
        marker.scale = scale
        return marker


if __name__ == "__main__":
    rospy.init_node("motion_monitor")
    MotionMonitor()
    rospy.spin()

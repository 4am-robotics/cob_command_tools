#!/usr/bin/env python3

import math
from dataclasses import dataclass
import threading
from typing import List, Tuple

import rospy
import tf2_ros
from geometry_msgs.msg import Point, Pose, Quaternion, Vector3
from nav_msgs.msg import Path
from std_msgs.msg import ColorRGBA
from tf2_geometry_msgs import do_transform_pose
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker


@dataclass
class AngleInterval:
    min_angle: float
    max_angle: float

    def __post_init__(self):
        if self.min_angle >= self.max_angle:
            raise ValueError("min_angle should be less than max_angle")


@dataclass
class MotionCategory:
    name: str
    angle_intervals: List[AngleInterval]
    display_color: ColorRGBA
    display_yaw: float

    def is_angle_in_intervals(self, target_angle: float) -> bool:
        return any(
            interval.min_angle < target_angle < interval.max_angle
            for interval in self.angle_intervals
        )


class MotionMonitor():
    def __init__(self):
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.distance_threshold_m = rospy.get_param("~distance_threshold_m", 2.0)
        self.angle_threshold_rad = rospy.get_param(
            "~angle_threshold_rad", math.pi/6)  # 30 degrees in radians
        self.target_frame_id = rospy.get_param("~target_frame_id", "base_link")

        # marker related params
        self.should_publish_marker = rospy.get_param("~should_publish_marker", False)
        motion_monitor_marker_topic = rospy.get_param(
            "~motion_monitor_marker_topic", "/motion_monitor/motion_monitor_marker"
        )
        marker_position_xyz = rospy.get_param("~marker_position_xyz", [0, 0, 0.1])
        marker_scale_xyz = rospy.get_param("~marker_scale_xyz", [1.5, 0.2, 0.2])
        self.direction_marker = MotionMonitor.get_direction_marker(
            frame_id=self.target_frame_id,
            scale=Vector3(*marker_scale_xyz),
            positon=Point(*marker_position_xyz)
        )
        self.marker_pub = rospy.Publisher(motion_monitor_marker_topic, Marker, queue_size=10)

        self.motion_categories = MotionMonitor.get_motion_categories(self.angle_threshold_rad)

        self.last_path = None
        self.last_path_mutex = threading.Lock()
        self.max_path_age = rospy.get_param("~max_path_age", 1.0)
        process_path_rate = rospy.get_param("~process_path_rate", 1.0)
        self.path_topic = rospy.get_param("~path_topic", "/path")
        rospy.Subscriber(self.path_topic, Path, self.path_callback, queue_size=1)
        self.process_path_timer = rospy.Timer(
            rospy.Duration(1.0 / process_path_rate), self.process_path
        )

    def path_callback(self, path: Path) -> None:
        with self.last_path_mutex:
            self.last_path = path

    def publish_direction_marker(self, color: ColorRGBA, yaw: float) -> None:
        """Publish a marker showing the direction of motion."""
        if self.should_publish_marker:
            self.direction_marker.color = color
            self.direction_marker.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, yaw))
            self.marker_pub.publish(self.direction_marker)

    def get_transform(self, target_frame: str, source_frame: str):
        """Look up the most recent transform between target_frame and source_frame."""
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
        """Return the motion category that the target_yaw angle falls into."""
        for category in self.motion_categories:
            if category.is_angle_in_intervals(target_yaw):
                return category
        return None

    def process_path(self, _: rospy.timer.TimerEvent) -> None:
        """Timer callback method to process the latest path message."""
        with self.last_path_mutex:
            if not self.last_path:
                return

            path_timestamp_diff = rospy.Time.now() - self.last_path.header.stamp
            if path_timestamp_diff > rospy.Duration(self.max_path_age):
                return

            transform_target_path = self.get_transform(
                self.target_frame_id, self.last_path.header.frame_id
            )
            if not transform_target_path:
                return

            for pose in self.last_path.poses:
                # Transform pose to target frame
                pose_stamped_in_target_frame = do_transform_pose(pose, transform_target_path)

                distance = MotionMonitor.get_distance(pose_stamped_in_target_frame.pose)
                if distance > self.distance_threshold_m:
                    target_yaw = math.atan2(pose_stamped_in_target_frame.pose.position.y,
                                            pose_stamped_in_target_frame.pose.position.x)

                    rospy.loginfo(f"distance: {distance:.2f}m, target_yaw: {target_yaw:.2f}rad")

                    category = self.get_valid_category(target_yaw)
                    if category:
                        rospy.loginfo(f"Motion category: {category.name}")
                        self.publish_direction_marker(category.display_color, category.display_yaw)
                        break
            else:
                rospy.loginfo(
                    "No pose found in the path which is more than the threshold distance away "
                    f"({self.distance_threshold_m:.2f}m)"
                )

    @staticmethod
    def get_motion_categories(angle_threshold_rad) -> List[MotionCategory]:
        """Return a list of motion categories split by the specified angle."""
        return [
            MotionCategory(
                "forward",
                [AngleInterval(-angle_threshold_rad, angle_threshold_rad)],
                ColorRGBA(0, 1, 0, 1),
                0),
            MotionCategory(
                "forward-left",
                [AngleInterval(angle_threshold_rad, math.pi / 2)],
                ColorRGBA(0, 0, 1, 1),
                angle_threshold_rad),
            MotionCategory(
                "forward-right",
                [AngleInterval(-math.pi/2, -angle_threshold_rad)],
                ColorRGBA(1, 0, 1, 1),
                -angle_threshold_rad),
            MotionCategory(
                "backward",
                [AngleInterval(math.pi - angle_threshold_rad, math.pi),
                 AngleInterval(-math.pi, -math.pi + angle_threshold_rad)],
                ColorRGBA(1, 0, 0, 1),
                math.pi),
            MotionCategory(
                "backward-left",
                [AngleInterval(-math.pi + angle_threshold_rad, -math.pi/2)],
                ColorRGBA(0, 1, 1, 1),
                -math.pi + angle_threshold_rad),
            MotionCategory(
                "backward-right",
                [AngleInterval(math.pi/2, math.pi - angle_threshold_rad)],
                ColorRGBA(1, 0.5, 0, 1),
                math.pi - angle_threshold_rad)
        ]

    @staticmethod
    def get_distance(pose: Pose) -> float:
        """Return the Euclidean distance from the origin to the pose's position."""
        return math.sqrt(pose.position.x**2 + pose.position.y**2)

    @staticmethod
    def get_direction_marker(frame_id: str, scale: Vector3, positon: Point) -> Marker:
        """Return a Marker message of type ARROW, which points in the direction of motion."""
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose.position = positon
        marker.pose.orientation.w = 1.0
        marker.scale = scale
        return marker


if __name__ == "__main__":
    rospy.init_node("motion_monitor")
    MotionMonitor()
    rospy.spin()

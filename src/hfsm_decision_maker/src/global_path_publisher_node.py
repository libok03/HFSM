#!/usr/bin/env python3

import math

import rospy
import tf.transformations

from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Path


class GlobalPathPublisherNode:
    def __init__(self):
        rospy.init_node("global_path_publisher_node")

        self.frame_id = rospy.get_param("~frame_id", "map")
        self.start_x = rospy.get_param("~start_x", 0.0)
        self.start_y = rospy.get_param("~start_y", 0.0)
        self.yaw = rospy.get_param("~yaw", 0.0)
        self.path_length = rospy.get_param("~path_length", 300.0)
        self.point_interval = rospy.get_param("~point_interval", 1.0)
        self.publish_hz = rospy.get_param("~publish_hz", 1.0)

        self.pub_path = rospy.Publisher(
            "/global_path",
            Path,
            queue_size=1,
            latch=True
        )

        self.path = self.build_straight_path()
        self.timer = rospy.Timer(
            rospy.Duration(1.0 / max(self.publish_hz, 0.1)),
            self.timer_callback
        )

        rospy.loginfo(
            "global_path_publisher_node initialized. points=%d frame=%s",
            len(self.path.poses),
            self.frame_id
        )

    def build_straight_path(self):
        path = Path()
        path.header.frame_id = self.frame_id

        q = tf.transformations.quaternion_from_euler(0.0, 0.0, self.yaw)
        count = max(2, int(self.path_length / max(self.point_interval, 0.1)) + 1)

        for i in range(count):
            s = i * self.point_interval

            pose = PoseStamped()
            pose.header.frame_id = self.frame_id
            pose.pose.position.x = self.start_x + s * math.cos(self.yaw)
            pose.pose.position.y = self.start_y + s * math.sin(self.yaw)
            pose.pose.orientation = Quaternion(*q)

            path.poses.append(pose)

        return path

    def timer_callback(self, event):
        stamp = rospy.Time.now()
        self.path.header.stamp = stamp
        for pose in self.path.poses:
            pose.header.stamp = stamp

        self.pub_path.publish(self.path)


if __name__ == "__main__":
    try:
        node = GlobalPathPublisherNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

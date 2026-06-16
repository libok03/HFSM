#!/usr/bin/env python3

import math

import rospy
import tf.transformations

from autoware_msgs.msg import DetectedObjectArray
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32


class ObstacleEvaluatorNode:
    def __init__(self):
        rospy.init_node("obstacle_evaluator_node")

        self.odom_topic = rospy.get_param(
            "~odom_topic",
            "/localization_kinematic_state"
        )
        self.obstacle_topic = rospy.get_param(
            "~obstacle_topic",
            "/estimation/objects"
        )

        self.ego_path_half_width = rospy.get_param("~ego_path_half_width", 1.75)
        self.obstacle_detect_dist = rospy.get_param("~obstacle_detect_dist", 30.0)
        self.avoidance_possible_dist = rospy.get_param("~avoidance_possible_dist", 7.0)
        self.ttc_safe_value = rospy.get_param("~ttc_safe_value", 999.9)
        self.publish_hz = rospy.get_param("~publish_hz", 10.0)

        self.odom_received = False
        self.ego_x = 0.0
        self.ego_y = 0.0
        self.ego_yaw = 0.0
        self.ego_speed = 0.0
        self.objects = []

        self.pub_exists = rospy.Publisher("/obstacle/exists", Bool, queue_size=10)
        self.pub_front_distance = rospy.Publisher(
            "/obstacle/front_distance",
            Float32,
            queue_size=10
        )
        self.pub_in_ego_path = rospy.Publisher(
            "/obstacle/in_ego_path",
            Bool,
            queue_size=10
        )
        self.pub_avoidance_possible = rospy.Publisher(
            "/obstacle/avoidance_possible",
            Bool,
            queue_size=10
        )
        self.pub_ttc = rospy.Publisher("/ttc", Float32, queue_size=10)

        rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback, queue_size=1)
        rospy.Subscriber(
            self.obstacle_topic,
            DetectedObjectArray,
            self.objects_callback,
            queue_size=1
        )

        self.timer = rospy.Timer(
            rospy.Duration(1.0 / max(self.publish_hz, 0.1)),
            self.timer_callback
        )

        rospy.loginfo("obstacle_evaluator_node initialized")

    def odom_callback(self, msg):
        self.odom_received = True
        self.ego_x = msg.pose.pose.position.x
        self.ego_y = msg.pose.pose.position.y
        self.ego_yaw = self.get_yaw(msg.pose.pose.orientation)
        self.ego_speed = math.hypot(
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y
        )

    def objects_callback(self, msg):
        self.objects = msg.objects

    @staticmethod
    def get_yaw(q):
        _, _, yaw = tf.transformations.euler_from_quaternion(
            [q.x, q.y, q.z, q.w]
        )
        return yaw

    @staticmethod
    def object_speed(obj):
        try:
            return math.hypot(obj.velocity.linear.x, obj.velocity.linear.y)
        except AttributeError:
            return 0.0

    def object_local_position(self, obj):
        dx = obj.pose.position.x - self.ego_x
        dy = obj.pose.position.y - self.ego_y

        local_x = math.cos(self.ego_yaw) * dx + math.sin(self.ego_yaw) * dy
        local_y = -math.sin(self.ego_yaw) * dx + math.cos(self.ego_yaw) * dy
        return local_x, local_y

    def evaluate(self):
        if not self.odom_received:
            return False, -1.0, False, False, self.ttc_safe_value

        nearest_front = None
        nearest_in_path = None
        nearest_obj = None

        for obj in self.objects:
            local_x, local_y = self.object_local_position(obj)

            if local_x < 0.0 or local_x > self.obstacle_detect_dist:
                continue

            if nearest_front is None or local_x < nearest_front:
                nearest_front = local_x

            if abs(local_y) <= self.ego_path_half_width:
                if nearest_in_path is None or local_x < nearest_in_path:
                    nearest_in_path = local_x
                    nearest_obj = obj

        exists = nearest_front is not None
        front_distance = nearest_in_path if nearest_in_path is not None else -1.0
        in_ego_path = nearest_in_path is not None
        avoidance_possible = (
            in_ego_path and
            nearest_in_path > self.avoidance_possible_dist
        )

        ttc = self.ttc_safe_value
        if nearest_obj is not None:
            rel_speed = self.ego_speed - self.object_speed(nearest_obj)
            if rel_speed > 0.1:
                ttc = nearest_in_path / rel_speed

        return exists, front_distance, in_ego_path, avoidance_possible, ttc

    def timer_callback(self, event):
        exists, front_distance, in_path, avoidance_possible, ttc = self.evaluate()

        self.pub_exists.publish(Bool(exists))
        self.pub_front_distance.publish(Float32(front_distance))
        self.pub_in_ego_path.publish(Bool(in_path))
        self.pub_avoidance_possible.publish(Bool(avoidance_possible))
        self.pub_ttc.publish(Float32(ttc))


if __name__ == "__main__":
    try:
        node = ObstacleEvaluatorNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

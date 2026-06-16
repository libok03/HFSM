#!/usr/bin/env python3

import rospy

from std_msgs.msg import String, Float32, Bool


class BehaviorRouterNode:
    def __init__(self):
        rospy.init_node("behavior_router_node")

        # ---------------- Parameters ----------------
        self.hfsm_state_topic = rospy.get_param(
            "~hfsm_state_topic",
            "/hfsm/current_state"
        )

        self.lane_change_direction_topic = rospy.get_param(
            "~lane_change_direction_topic",
            "/lane_change/recommended_direction"
        )

        self.speed_basic = rospy.get_param("~speed_basic", 5.0)
        self.speed_intersection = rospy.get_param("~speed_intersection", 2.0)
        self.speed_obstacle = rospy.get_param("~speed_obstacle", 1.5)
        self.speed_lane_change = rospy.get_param("~speed_lane_change", 3.0)
        self.speed_avoidance = rospy.get_param("~speed_avoidance", 2.0)
        self.speed_parking_approach = rospy.get_param(
            "~speed_parking_approach",
            1.0
        )
        self.speed_stop = rospy.get_param("~speed_stop", 0.0)

        self.publish_hz = rospy.get_param("~publish_hz", 10.0)

        # ---------------- State ----------------
        self.current_hfsm_state = "INIT"
        self.lane_change_direction = "NONE"

        # ---------------- Publishers ----------------
        self.pub_planner_mode = rospy.Publisher(
            "/behavior/planner_mode",
            String,
            queue_size=10
        )

        self.pub_target_speed = rospy.Publisher(
            "/behavior/target_speed",
            Float32,
            queue_size=10
        )

        self.pub_lane_change_direction = rospy.Publisher(
            "/behavior/lane_change_direction",
            String,
            queue_size=10
        )

        self.pub_avoidance_enable = rospy.Publisher(
            "/behavior/avoidance_enable",
            Bool,
            queue_size=10
        )

        self.pub_stop_request = rospy.Publisher(
            "/behavior/stop_request",
            Bool,
            queue_size=10
        )

        # ---------------- Subscribers ----------------
        rospy.Subscriber(
            self.hfsm_state_topic,
            String,
            self.hfsm_state_callback,
            queue_size=1
        )

        rospy.Subscriber(
            self.lane_change_direction_topic,
            String,
            self.lane_change_direction_callback,
            queue_size=1
        )

        self.timer = rospy.Timer(
            rospy.Duration(1.0 / self.publish_hz),
            self.timer_callback
        )

        rospy.loginfo("behavior_router_node initialized")

    def hfsm_state_callback(self, msg):
        self.current_hfsm_state = msg.data

    def lane_change_direction_callback(self, msg):
        if msg.data in ["LEFT", "RIGHT", "NONE"]:
            self.lane_change_direction = msg.data
        else:
            self.lane_change_direction = "NONE"

    def make_command(self, state):
        """
        HFSM state -> planner command 변환.
        여기서는 판단하지 않는다. 단순 mapping만 수행한다.
        """

        # 기본값: 안전 정지
        planner_mode = "STOP"
        target_speed = self.speed_stop
        lane_change_direction = "NONE"
        avoidance_enable = False
        stop_request = True

        if state == "INIT":
            planner_mode = "STOP"
            target_speed = self.speed_stop
            stop_request = True

        elif state == "BASIC_DRIVING":
            planner_mode = "LANE_FOLLOW"
            target_speed = self.speed_basic
            stop_request = False

        elif state == "INTERSECTION":
            planner_mode = "LANE_FOLLOW"
            target_speed = self.speed_intersection
            stop_request = False

        elif state == "OBS_MANAGER":
            planner_mode = "STOP"
            target_speed = self.speed_stop
            stop_request = True

        elif state == "LANE_CHANGE":
            planner_mode = "LANE_CHANGE"
            target_speed = self.speed_lane_change
            lane_change_direction = self.lane_change_direction
            stop_request = False

            if lane_change_direction not in ["LEFT", "RIGHT"]:
                rospy.logwarn_throttle(
                    1.0,
                    "LANE_CHANGE state but lane_change_direction is invalid: %s",
                    lane_change_direction
                )

        elif state == "OBS_AVOIDANCE":
            planner_mode = "AVOIDANCE"
            target_speed = self.speed_avoidance
            avoidance_enable = True
            stop_request = False

        elif state == "LANE_FOLLOWING_PARKING":
            planner_mode = "LANE_FOLLOW"
            target_speed = self.speed_parking_approach
            stop_request = False

        elif state == "PARKING_MANEUVER":
            # Parking planner is not implemented yet. Hold the vehicle safely.
            planner_mode = "STOP"
            target_speed = self.speed_stop
            stop_request = True

        elif state == "ESTOP":
            planner_mode = "STOP"
            target_speed = self.speed_stop
            stop_request = True

        elif state == "FINISHED":
            planner_mode = "STOP"
            target_speed = self.speed_stop
            stop_request = True

        else:
            rospy.logwarn_throttle(
                1.0,
                "Unknown HFSM state: %s. Publish STOP command.",
                state
            )

        return {
            "planner_mode": planner_mode,
            "target_speed": target_speed,
            "lane_change_direction": lane_change_direction,
            "avoidance_enable": avoidance_enable,
            "stop_request": stop_request
        }

    def timer_callback(self, event):
        cmd = self.make_command(self.current_hfsm_state)

        self.pub_planner_mode.publish(String(cmd["planner_mode"]))
        self.pub_target_speed.publish(Float32(cmd["target_speed"]))
        self.pub_lane_change_direction.publish(String(cmd["lane_change_direction"]))
        self.pub_avoidance_enable.publish(Bool(cmd["avoidance_enable"]))
        self.pub_stop_request.publish(Bool(cmd["stop_request"]))

        rospy.loginfo_throttle(
            1.0,
            "BehaviorRouter | hfsm=%s mode=%s speed=%.2f dir=%s avoidance=%s stop=%s",
            self.current_hfsm_state,
            cmd["planner_mode"],
            cmd["target_speed"],
            cmd["lane_change_direction"],
            cmd["avoidance_enable"],
            cmd["stop_request"]
        )


if __name__ == "__main__":
    try:
        node = BehaviorRouterNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

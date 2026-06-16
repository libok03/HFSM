#!/usr/bin/env python3
"""
behavior_command_node.py
------------------------
Subscribes to /hfsm/current_state (std_msgs/String) published by hfsm_node,
and translates the active HFSM state into concrete behavior commands:
  - /behavior/target_speed    (std_msgs/Float32)  : desired speed [m/s]
  - /behavior/planner_mode    (std_msgs/String)   : mode string for local planner
  - /behavior/stop_request    (std_msgs/Bool)     : True = request full stop

Topic names and default speeds are all configurable via ROS parameters or launch file.
"""

import rospy
from std_msgs.msg import Float32, Bool, String


class BehaviorCommandNode:
    def __init__(self):
        rospy.init_node('behavior_command_node')

        # ── Speed parameters [m/s] ──────────────────────────────────────
        self.speed_basic_driving   = rospy.get_param('~speed_basic_driving',   10.0)
        self.speed_intersection    = rospy.get_param('~speed_intersection',     5.0)
        self.speed_lane_change     = rospy.get_param('~speed_lane_change',      7.0)
        self.speed_obs_avoidance   = rospy.get_param('~speed_obs_avoidance',    5.0)
        self.speed_parking_approach= rospy.get_param('~speed_parking_approach', 3.0)
        self.speed_stop            = rospy.get_param('~speed_stop',             0.0)

        # ── Publishers ──────────────────────────────────────────────────
        self.pub_target_speed  = rospy.Publisher('/behavior/target_speed',  Float32, queue_size=10)
        self.pub_planner_mode  = rospy.Publisher('/behavior/planner_mode',  String,  queue_size=10)
        self.pub_stop_request  = rospy.Publisher('/behavior/stop_request',  Bool,    queue_size=10)

        # ── Subscriber ──────────────────────────────────────────────────
        hfsm_state_topic = rospy.get_param('~hfsm_state_topic', '/hfsm/current_state')
        rospy.Subscriber(hfsm_state_topic, String, self.state_callback)

        rospy.loginfo("behavior_command_node started.")

    # State → command mapping
    STATE_MAP = {
        # state_string       : (target_speed_attr,      planner_mode,        stop)
        'INIT'               : ('speed_stop',            'STANDBY',           False),
        'BASIC_DRIVING'      : ('speed_basic_driving',   'LANE_FOLLOW',       False),
        'INTERSECTION'       : ('speed_intersection',    'INTERSECTION',      False),
        'OBS_MANAGER'        : ('speed_stop',            'STOP',              True),  # transient
        'ESTOP'              : ('speed_stop',            'STOP',              True),
        'LANE_CHANGE'        : ('speed_lane_change',     'LANE_CHANGE',       False),
        'OBS_AVOIDANCE'      : ('speed_obs_avoidance',   'OBS_AVOIDANCE',     False),
        'LANE_FOLLOWING_PARKING': ('speed_parking_approach', 'LANE_FOLLOW',   False),
        'PARKING_MANEUVER'   : ('speed_stop',            'PARKING_MANEUVER',  False),
        'FINISHED'           : ('speed_stop',            'STOP',              True),
    }

    def state_callback(self, msg):
        state = msg.data.strip()
        entry = self.STATE_MAP.get(state)

        if entry is None:
            rospy.logwarn_throttle(5.0, f"[behavior_command_node] Unknown HFSM state: '{state}'")
            return

        speed_attr, planner_mode, stop_req = entry
        target_speed = getattr(self, speed_attr, 0.0)

        self.pub_target_speed.publish(Float32(target_speed))
        self.pub_planner_mode.publish(String(planner_mode))
        self.pub_stop_request.publish(Bool(stop_req))


if __name__ == '__main__':
    try:
        node = BehaviorCommandNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

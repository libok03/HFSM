#!/usr/bin/env python3
"""
dummy_publisher_node.py  (v2 - Keyboard Interactive)
=====================================================
HFSM 전체 상태를 키보드로 즉시 전환하며 테스트할 수 있는 더미 퍼블리셔.

평가기(Evaluator) 출력 토픽을 직접 발행하므로,
Evaluator 노드 없이 hfsm_node 단독으로도 모든 상태를 검증할 수 있다.

Keyboard Controls
-----------------
  1  Normal driving        → BASIC_DRIVING 유지
  2  Intersection approach → BASIC_DRIVING → INTERSECTION
  3  In intersection       → INTERSECTION 유지
  4  Intersection exit     → INTERSECTION → BASIC_DRIVING
  5  E-Stop               → OBSTACLE / ESTOP
  6  Lane Change          → OBSTACLE / LANE_CHANGE
  7  Obs Avoidance        → OBSTACLE / OBS_AVOIDANCE
  8  Parking approach     → LANE_FOLLOWING_PARKING
  9  Parking maneuver     → PARKING_MANEUVER
  0  Parking done         → (FINISHED)
  h  Print this help
  q  Quit

Published Topics (Evaluator outputs → direct to hfsm_node)
-----------------------------------------------------------
/localization_kinematic_state   nav_msgs/Odometry
/ttc                            std_msgs/Float32
/lane_status                    std_msgs/Bool
/local_planner/status           std_msgs/String

/obstacle/exists                std_msgs/Bool
/obstacle/front_distance        std_msgs/Float32
/obstacle/in_ego_path           std_msgs/Bool
/obstacle/avoidance_possible    std_msgs/Bool

/lane_change/adjacent_lane_safe std_msgs/Bool
/lane_change/recommended_direction std_msgs/String

/route_event/dist_to_intersection  std_msgs/Float32
/route_event/in_intersection       std_msgs/Bool
/route_event/intersection_exit_passed std_msgs/Bool
/route_event/dist_to_parking_zone  std_msgs/Float32

/parking/target_on_route        std_msgs/Bool
/parking/at_start_position      std_msgs/Bool
/parking/dist_to_goal           std_msgs/Float32
/parking/yaw_error              std_msgs/Float32
"""

import sys
import tty
import termios
import threading
import rospy
from std_msgs.msg import Float32, Bool, String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion, Twist, Vector3


# ──────────────────────────────────────────────────────────────────
# Scenario definitions
# Each entry maps to the VehicleState fields hfsm_node reads.
# ──────────────────────────────────────────────────────────────────
SCENARIOS = {
    '1': {
        'name': 'Normal (BASIC_DRIVING)',
        # Obstacle
        'obs_exists': False, 'obs_dist': 999.0,
        'obs_in_ego_path': False, 'obs_avoidance_possible': True,
        'ttc': 999.0,
        # Lane change
        'adjacent_lane_safe': True, 'recommended_dir': 'LEFT',
        # Route events
        'dist_to_intersection': 999.0, 'in_intersection': False,
        'intersection_exit_passed': False,
        'dist_to_parking_zone': 999.0,
        # Parking
        'parking_target_on_route': False, 'at_parking_start': False,
        'dist_to_parking_goal': 999.0, 'yaw_error_parking': 0.0,
        # Ego / Lane
        'ego_speed': 5.0, 'lane_status_ok': True,
        'planner_status': '',
    },
    '2': {
        'name': 'Intersection Approach (BASIC_DRIVING → INTERSECTION)',
        'obs_exists': False, 'obs_dist': 999.0,
        'obs_in_ego_path': False, 'obs_avoidance_possible': True,
        'ttc': 999.0,
        'adjacent_lane_safe': True, 'recommended_dir': 'LEFT',
        # dist < INTERSECTION_ENTER_DIST (20 m) triggers transition
        'dist_to_intersection': 10.0, 'in_intersection': False,
        'intersection_exit_passed': False,
        'dist_to_parking_zone': 999.0,
        'parking_target_on_route': False, 'at_parking_start': False,
        'dist_to_parking_goal': 999.0, 'yaw_error_parking': 0.0,
        'ego_speed': 5.0, 'lane_status_ok': True,
        'planner_status': '',
    },
    '3': {
        'name': 'In Intersection (INTERSECTION hold)',
        'obs_exists': False, 'obs_dist': 999.0,
        'obs_in_ego_path': False, 'obs_avoidance_possible': True,
        'ttc': 999.0,
        'adjacent_lane_safe': True, 'recommended_dir': 'LEFT',
        'dist_to_intersection': 0.0, 'in_intersection': True,
        'intersection_exit_passed': False,
        'dist_to_parking_zone': 999.0,
        'parking_target_on_route': False, 'at_parking_start': False,
        'dist_to_parking_goal': 999.0, 'yaw_error_parking': 0.0,
        'ego_speed': 3.0, 'lane_status_ok': True,
        'planner_status': '',
    },
    '4': {
        'name': 'Intersection Exit (INTERSECTION → BASIC_DRIVING)',
        'obs_exists': False, 'obs_dist': 999.0,
        'obs_in_ego_path': False, 'obs_avoidance_possible': True,
        'ttc': 999.0,
        'adjacent_lane_safe': True, 'recommended_dir': 'LEFT',
        'dist_to_intersection': 999.0, 'in_intersection': False,
        # This fires the latch → hfsm_node sees intersection_exit_passed=True
        'intersection_exit_passed': True,
        'dist_to_parking_zone': 999.0,
        'parking_target_on_route': False, 'at_parking_start': False,
        'dist_to_parking_goal': 999.0, 'yaw_error_parking': 0.0,
        'ego_speed': 5.0, 'lane_status_ok': True,
        'planner_status': '',
    },
    '5': {
        'name': 'E-Stop (OBSTACLE/ESTOP)',
        # ttc < TTC_ESTOP(1.5) triggers ESTOP
        'obs_exists': True, 'obs_dist': 3.0,
        'obs_in_ego_path': True, 'obs_avoidance_possible': False,
        'ttc': 1.0,
        'adjacent_lane_safe': False, 'recommended_dir': 'NONE',
        'dist_to_intersection': 999.0, 'in_intersection': False,
        'intersection_exit_passed': False,
        'dist_to_parking_zone': 999.0,
        'parking_target_on_route': False, 'at_parking_start': False,
        'dist_to_parking_goal': 999.0, 'yaw_error_parking': 0.0,
        'ego_speed': 0.0, 'lane_status_ok': True,
        'planner_status': '',
    },
    '6': {
        'name': 'Lane Change (OBSTACLE/LANE_CHANGE)',
        # TTC_ESTOP(1.5) <= ttc < TTC_LANE_CHANGE(4.0), adj lane safe
        'obs_exists': True, 'obs_dist': 20.0,
        'obs_in_ego_path': True, 'obs_avoidance_possible': True,
        'ttc': 3.0,
        'adjacent_lane_safe': True, 'recommended_dir': 'LEFT',
        'dist_to_intersection': 999.0, 'in_intersection': False,
        'intersection_exit_passed': False,
        'dist_to_parking_zone': 999.0,
        'parking_target_on_route': False, 'at_parking_start': False,
        'dist_to_parking_goal': 999.0, 'yaw_error_parking': 0.0,
        'ego_speed': 5.0, 'lane_status_ok': True,
        'planner_status': '',
    },
    '7': {
        'name': 'Obs Avoidance (OBSTACLE/OBS_AVOIDANCE)',
        # adj_lane not safe and front distance < OBSTACLE_SAFE_DIST(15 m) -> avoidance
        'obs_exists': True, 'obs_dist': 10.0,
        'obs_in_ego_path': True, 'obs_avoidance_possible': True,
        'ttc': 4.5,
        'adjacent_lane_safe': False, 'recommended_dir': 'NONE',
        'dist_to_intersection': 999.0, 'in_intersection': False,
        'intersection_exit_passed': False,
        'dist_to_parking_zone': 999.0,
        'parking_target_on_route': False, 'at_parking_start': False,
        'dist_to_parking_goal': 999.0, 'yaw_error_parking': 0.0,
        'ego_speed': 5.0, 'lane_status_ok': True,
        'planner_status': '',
    },
    '8': {
        'name': 'Parking Approach (LANE_FOLLOWING_PARKING)',
        # parking_target_on_route=True, dist < PARKING_APPROACH_DIST(30 m)
        'obs_exists': False, 'obs_dist': 999.0,
        'obs_in_ego_path': False, 'obs_avoidance_possible': True,
        'ttc': 999.0,
        'adjacent_lane_safe': True, 'recommended_dir': 'LEFT',
        'dist_to_intersection': 999.0, 'in_intersection': False,
        'intersection_exit_passed': False,
        'dist_to_parking_zone': 20.0,
        'parking_target_on_route': True, 'at_parking_start': False,
        'dist_to_parking_goal': 999.0, 'yaw_error_parking': 0.0,
        'ego_speed': 3.0, 'lane_status_ok': True,
        'planner_status': '',
    },
    '9': {
        'name': 'Parking Maneuver (PARKING_MANEUVER)',
        # at_parking_start=True, speed=0 → enters PARKING_MANEUVER
        'obs_exists': False, 'obs_dist': 999.0,
        'obs_in_ego_path': False, 'obs_avoidance_possible': True,
        'ttc': 999.0,
        'adjacent_lane_safe': True, 'recommended_dir': 'LEFT',
        'dist_to_intersection': 999.0, 'in_intersection': False,
        'intersection_exit_passed': False,
        'dist_to_parking_zone': 0.5,
        'parking_target_on_route': True, 'at_parking_start': True,
        'dist_to_parking_goal': 0.3, 'yaw_error_parking': 0.02,
        'ego_speed': 0.0, 'lane_status_ok': True,
        'planner_status': '',
    },
    '0': {
        'name': 'Parking Done → FINISHED',
        # dist_to_parking_goal < PARKING_POS_TOL(0.5), yaw_error < tol, speed < 0.1
        'obs_exists': False, 'obs_dist': 999.0,
        'obs_in_ego_path': False, 'obs_avoidance_possible': True,
        'ttc': 999.0,
        'adjacent_lane_safe': True, 'recommended_dir': 'LEFT',
        'dist_to_intersection': 999.0, 'in_intersection': False,
        'intersection_exit_passed': False,
        'dist_to_parking_zone': 0.0,
        'parking_target_on_route': True, 'at_parking_start': True,
        'dist_to_parking_goal': 0.1, 'yaw_error_parking': 0.01,
        'ego_speed': 0.0, 'lane_status_ok': True,
        'planner_status': 'PARKING_DONE',
    },
}

HELP_TEXT = """
╔══════════════════════════════════════════════════════╗
║          HFSM Dummy Publisher - Keyboard Control      ║
╠══════════════════════════════════════════════════════╣
║  1  Normal              → BASIC_DRIVING               ║
║  2  Intersection Appr.  → BASIC_DRIVING→INTERSECTION  ║
║  3  In Intersection     → INTERSECTION hold           ║
║  4  Intersection Exit   → INTERSECTION→BASIC_DRIVING  ║
║  5  E-Stop              → OBSTACLE/ESTOP              ║
║  6  Lane Change         → OBSTACLE/LANE_CHANGE        ║
║  7  Obs Avoidance       → OBSTACLE/OBS_AVOIDANCE      ║
║  8  Parking Approach    → LANE_FOLLOWING_PARKING      ║
║  9  Parking Maneuver    → PARKING_MANEUVER            ║
║  0  Parking Done        → FINISHED                    ║
║  h  Help                                              ║
║  q  Quit                                              ║
╚══════════════════════════════════════════════════════╝
Monitor: rostopic echo /hfsm/current_state
"""


class DummyPublisherNode:
    def __init__(self):
        rospy.init_node('dummy_publisher_node')

        self.current_scenario = SCENARIOS['1']
        self._lock = threading.Lock()

        # ── Publishers ─────────────────────────────────────────────
        self.pubs = {
            'odom':        rospy.Publisher('/localization_kinematic_state',
                                           Odometry, queue_size=10),
            'ttc':         rospy.Publisher('/ttc', Float32, queue_size=10),
            'lane_status': rospy.Publisher('/lane_status', Bool, queue_size=10),
            'planner':     rospy.Publisher('/local_planner/status',
                                           String, queue_size=10),
            # Obstacle evaluator outputs
            'obs_exists':  rospy.Publisher('/obstacle/exists', Bool, queue_size=10),
            'obs_dist':    rospy.Publisher('/obstacle/front_distance',
                                           Float32, queue_size=10),
            'obs_path':    rospy.Publisher('/obstacle/in_ego_path',
                                           Bool, queue_size=10),
            'obs_avoid':   rospy.Publisher('/obstacle/avoidance_possible',
                                           Bool, queue_size=10),
            # Lane change evaluator outputs
            'lc_safe':     rospy.Publisher('/lane_change/adjacent_lane_safe',
                                           Bool, queue_size=10),
            'lc_dir':      rospy.Publisher('/lane_change/recommended_direction',
                                           String, queue_size=10),
            # Route event evaluator outputs
            'dist_inter':  rospy.Publisher('/route_event/dist_to_intersection',
                                           Float32, queue_size=10),
            'in_inter':    rospy.Publisher('/route_event/in_intersection',
                                           Bool, queue_size=10),
            'inter_exit':  rospy.Publisher('/route_event/intersection_exit_passed',
                                           Bool, queue_size=10),
            'dist_park':   rospy.Publisher('/route_event/dist_to_parking_zone',
                                           Float32, queue_size=10),
            # Parking topics
            'park_target': rospy.Publisher('/parking/target_on_route',
                                           Bool, queue_size=10),
            'park_start':  rospy.Publisher('/parking/at_start_position',
                                           Bool, queue_size=10),
            'park_goal':   rospy.Publisher('/parking/dist_to_goal',
                                           Float32, queue_size=10),
            'park_yaw':    rospy.Publisher('/parking/yaw_error',
                                           Float32, queue_size=10),
        }

        # ── Timer ──────────────────────────────────────────────────
        self.timer = rospy.Timer(rospy.Duration(0.1), self._publish_all)

        # ── Keyboard thread ────────────────────────────────────────
        self._kb_thread = threading.Thread(
            target=self._keyboard_loop, daemon=True)
        self._kb_thread.start()

        print(HELP_TEXT)
        rospy.loginfo("[dummy] Started. Press 1-9/0 to switch scenario.")

    # ── Publish ────────────────────────────────────────────────────

    def _publish_all(self, _event):
        with self._lock:
            sc = self.current_scenario

        stamp = rospy.Time.now()

        # Odometry
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'map'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position = Point(x=0.0, y=0.0, z=0.0)
        odom.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        odom.twist.twist = Twist(
            linear=Vector3(x=sc['ego_speed'], y=0.0, z=0.0),
            angular=Vector3(x=0.0, y=0.0, z=0.0))
        self.pubs['odom'].publish(odom)

        # Evaluator outputs
        self.pubs['ttc'].publish(Float32(sc['ttc']))
        self.pubs['lane_status'].publish(Bool(sc['lane_status_ok']))
        if sc['planner_status']:
            self.pubs['planner'].publish(String(sc['planner_status']))

        self.pubs['obs_exists'].publish(Bool(sc['obs_exists']))
        self.pubs['obs_dist'].publish(Float32(sc['obs_dist']))
        self.pubs['obs_path'].publish(Bool(sc['obs_in_ego_path']))
        self.pubs['obs_avoid'].publish(Bool(sc['obs_avoidance_possible']))

        self.pubs['lc_safe'].publish(Bool(sc['adjacent_lane_safe']))
        self.pubs['lc_dir'].publish(String(sc['recommended_dir']))

        self.pubs['dist_inter'].publish(Float32(sc['dist_to_intersection']))
        self.pubs['in_inter'].publish(Bool(sc['in_intersection']))
        self.pubs['inter_exit'].publish(Bool(sc['intersection_exit_passed']))
        self.pubs['dist_park'].publish(Float32(sc['dist_to_parking_zone']))

        self.pubs['park_target'].publish(Bool(sc['parking_target_on_route']))
        self.pubs['park_start'].publish(Bool(sc['at_parking_start']))
        self.pubs['park_goal'].publish(Float32(sc['dist_to_parking_goal']))
        self.pubs['park_yaw'].publish(Float32(sc['yaw_error_parking']))

    # ── Keyboard ───────────────────────────────────────────────────

    def _keyboard_loop(self):
        """Non-blocking single-character read using tty raw mode."""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            while not rospy.is_shutdown():
                ch = sys.stdin.read(1)
                if ch == 'q':
                    rospy.loginfo("[dummy] Quit requested.")
                    rospy.signal_shutdown("User quit")
                    break
                elif ch == 'h':
                    print(HELP_TEXT)
                elif ch in SCENARIOS:
                    sc = SCENARIOS[ch]
                    with self._lock:
                        self.current_scenario = sc
                    rospy.loginfo(
                        f"[dummy] Scenario → [{ch}] {sc['name']}")
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


if __name__ == '__main__':
    try:
        node = DummyPublisherNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

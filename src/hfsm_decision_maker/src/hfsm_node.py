#!/usr/bin/env python3

import rospy
import smach
import smach_ros
import math

from std_msgs.msg import Float32, Bool, String
from nav_msgs.msg import Odometry

state_pub = None


def publish_state(state_str):
    global state_pub
    if state_pub is not None:
        msg = String()
        msg.data = state_str
        state_pub.publish(msg)


# ==========================================
# Thresholds
# ==========================================
TTC_ESTOP               = 1.5
TTC_LANE_CHANGE         = 4.0
TTC_SAFE                = 5.0

ESTOP_DIST              = 5.0
OBSTACLE_SAFE_DIST      = 15.0

INTERSECTION_ENTER_DIST = 20.0
PARKING_APPROACH_DIST   = 30.0

LC_FRONT_SAFE_DIST      = 20.0
LC_REAR_SAFE_DIST       = 15.0
LC_TTC_SAFE             = 5.0

PARKING_READY_SPEED     = 0.5
PARKING_POS_TOL         = 0.5
PARKING_YAW_TOL         = math.radians(5)


def load_params():
    global TTC_ESTOP, TTC_LANE_CHANGE, TTC_SAFE
    global ESTOP_DIST, OBSTACLE_SAFE_DIST
    global INTERSECTION_ENTER_DIST, PARKING_APPROACH_DIST
    global LC_FRONT_SAFE_DIST, LC_REAR_SAFE_DIST, LC_TTC_SAFE
    global PARKING_READY_SPEED, PARKING_POS_TOL, PARKING_YAW_TOL

    TTC_ESTOP = rospy.get_param('~TTC_ESTOP', 1.5)
    TTC_LANE_CHANGE = rospy.get_param('~TTC_LANE_CHANGE', 4.0)
    TTC_SAFE = rospy.get_param('~TTC_SAFE', 5.0)

    ESTOP_DIST = rospy.get_param('~ESTOP_DIST', 5.0)
    OBSTACLE_SAFE_DIST = rospy.get_param('~OBSTACLE_SAFE_DIST', 15.0)

    INTERSECTION_ENTER_DIST = rospy.get_param('~INTERSECTION_ENTER_DIST', 20.0)
    PARKING_APPROACH_DIST = rospy.get_param('~PARKING_APPROACH_DIST', 30.0)

    LC_FRONT_SAFE_DIST = rospy.get_param('~LC_FRONT_SAFE_DIST', 20.0)
    LC_REAR_SAFE_DIST = rospy.get_param('~LC_REAR_SAFE_DIST', 15.0)
    LC_TTC_SAFE = rospy.get_param('~LC_TTC_SAFE', 5.0)

    PARKING_READY_SPEED = rospy.get_param('~PARKING_READY_SPEED', 0.5)
    PARKING_POS_TOL = rospy.get_param('~PARKING_POS_TOL', 0.5)

    yaw_deg = rospy.get_param('~PARKING_YAW_TOL_DEG', 5.0)
    PARKING_YAW_TOL = math.radians(yaw_deg)

    rospy.loginfo("HFSM thresholds loaded from ROS params")


# ==========================================
# Shared Vehicle State
# ==========================================
class VehicleState:
    def __init__(self):
        # Initialization info
        self.odom_received = False

        # Ego info
        self.speed = 0.0
        self.yaw = 0.0
        self.heading_stable = True
        self.yaw_stable = True

        # Lane info
        self.lane_status_ok = True

        # Obstacle info
        # /ttc convention:
        #   >= 0.0 : valid TTC
        #   999.9  : safe or separating
        #   -1.0   : invalid/unreliable TTC
        self.ttc = 999.9

        # /obstacle/front_distance convention:
        #   >= 0.0 : valid front distance
        #   -1.0   : no front obstacle / invalid
        self.obs_dist = -1.0

        self.obs_in_ego_path = False
        self.obs_avoidance_possible = False
        self.obs_avoidance_complete = False
        self.obs_exists = False

        # Intersection info
        self.dist_to_intersection = 999.0
        self.in_intersection = False
        self.intersection_exit_passed = False

        # Lane change info
        self.adjacent_lane_safe = True
        self.lane_change_complete = False
        self.recommended_direction = "NONE"  # LEFT, RIGHT, NONE

        # Parking info
        self.parking_target_on_route = False
        self.dist_to_parking_zone = 999.0
        self.at_parking_start = False
        self.parking_space_found = False
        self.dist_to_parking_goal = 999.0
        self.yaw_error_parking = 0.0


vs = VehicleState()


# ==========================================
# Helper functions for obstacle decision
# ==========================================
def has_valid_ttc():
    return vs.ttc >= 0.0


def has_valid_obs_dist():
    return vs.obs_dist >= 0.0


def is_ttc_danger():
    return has_valid_ttc() and vs.ttc < TTC_ESTOP


def is_ttc_lane_change_range():
    return has_valid_ttc() and TTC_ESTOP <= vs.ttc < TTC_LANE_CHANGE


def is_ttc_unsafe_for_driving():
    return has_valid_ttc() and vs.ttc < TTC_SAFE


def is_front_distance_estop():
    return has_valid_obs_dist() and vs.obs_dist < ESTOP_DIST


def is_front_distance_obstacle():
    return has_valid_obs_dist() and vs.obs_dist < OBSTACLE_SAFE_DIST


def is_obstacle_cleared():
    if not vs.obs_exists:
        return True

    ttc_safe = (not has_valid_ttc()) or vs.ttc > TTC_SAFE
    dist_safe = (not has_valid_obs_dist()) or vs.obs_dist > OBSTACLE_SAFE_DIST

    return ttc_safe and dist_safe


def should_enter_obstacle_from_driving():
    if not vs.obs_exists:
        return False

    if vs.obs_in_ego_path:
        return True

    if is_ttc_unsafe_for_driving():
        return True

    if is_front_distance_obstacle():
        return True

    return False


def should_enter_estop():
    if not vs.obs_exists:
        return False

    if not vs.obs_in_ego_path:
        return False

    if is_ttc_danger():
        return True

    if is_front_distance_estop():
        return True

    if not vs.lane_status_ok:
        return True

    if not vs.obs_avoidance_possible and not vs.adjacent_lane_safe:
        return True

    return False


def should_enter_lane_change():
    if not vs.obs_exists:
        return False

    if not vs.obs_in_ego_path:
        return False

    if not vs.adjacent_lane_safe:
        return False

    if vs.recommended_direction not in ["LEFT", "RIGHT"]:
        return False

    # TTC가 유효하면 lane-change range에서만 차선 변경
    if is_ttc_lane_change_range():
        return True

    # TTC가 invalid(-1.0)인 경우에도 거리상 여유가 있고 회피 가능하면 차선 변경 후보
    if (not has_valid_ttc()) and has_valid_obs_dist():
        return vs.obs_dist > ESTOP_DIST and vs.obs_dist < OBSTACLE_SAFE_DIST

    return False


def should_enter_obs_avoidance():
    if not vs.obs_exists:
        return False

    if not vs.obs_in_ego_path:
        return False

    if not vs.obs_avoidance_possible:
        return False

    if is_front_distance_obstacle():
        return True

    # TTC는 unsafe지만 lane change 조건이 안 되면 local avoidance 시도
    if is_ttc_unsafe_for_driving():
        return True

    return False


# ==========================================
# Callbacks
# ==========================================
def odom_callback(msg):
    vs.odom_received = True
    vs.speed = math.sqrt(
        msg.twist.twist.linear.x ** 2 +
        msg.twist.twist.linear.y ** 2
    )


def ttc_callback(msg):
    vs.ttc = msg.data


def lane_status_callback(msg):
    vs.lane_status_ok = msg.data


def obs_exists_callback(msg):
    vs.obs_exists = msg.data


def obs_dist_callback(msg):
    vs.obs_dist = msg.data


def obs_in_ego_path_callback(msg):
    vs.obs_in_ego_path = msg.data


def obs_avoidance_possible_callback(msg):
    vs.obs_avoidance_possible = msg.data


def adjacent_lane_safe_callback(msg):
    vs.adjacent_lane_safe = msg.data


def dist_to_intersection_callback(msg):
    vs.dist_to_intersection = msg.data


def in_intersection_callback(msg):
    vs.in_intersection = msg.data


def intersection_exit_passed_callback(msg):
    if msg.data:
        vs.intersection_exit_passed = True


def dist_to_parking_zone_callback(msg):
    vs.dist_to_parking_zone = msg.data


def recommended_direction_callback(msg):
    direction = msg.data.strip().upper()
    if direction in ["LEFT", "RIGHT"]:
        vs.recommended_direction = direction
    else:
        vs.recommended_direction = "NONE"


def parking_target_on_route_callback(msg):
    vs.parking_target_on_route = msg.data


def at_parking_start_callback(msg):
    vs.at_parking_start = msg.data


def dist_to_parking_goal_callback(msg):
    vs.dist_to_parking_goal = msg.data


def yaw_error_parking_callback(msg):
    vs.yaw_error_parking = msg.data


def planner_status_callback(msg):
    status = msg.data

    if status == "LANE_CHANGE_DONE":
        vs.lane_change_complete = True

    elif status == "OBS_AVOIDANCE_DONE":
        vs.obs_avoidance_complete = True

    elif status == "PARKING_DONE":
        vs.parking_space_found = True


# ==========================================
# STATES
# ==========================================
class Init(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['initialized', 'stay'])

    def execute(self, userdata):
        publish_state('INIT')

        if rospy.is_shutdown():
            return 'stay'

        if vs.odom_received:
            rospy.loginfo("Odom received. Exiting INIT state.")
            return 'initialized'

        rospy.sleep(0.1)
        return 'stay'


# ==========================================
# 1. DRIVING States
# ==========================================
class BasicDriving(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['to_intersection', 'to_obstacle', 'to_parking', 'stay']
        )

    def execute(self, userdata):
        rospy.sleep(0.1)
        publish_state('BASIC_DRIVING')

        if not vs.lane_status_ok:
            rospy.logwarn("BasicDriving -> Obstacle: lane status lost")
            return 'to_obstacle'

        if vs.in_intersection or vs.dist_to_intersection < INTERSECTION_ENTER_DIST:
            rospy.loginfo("BasicDriving -> Intersection")
            return 'to_intersection'

        if should_enter_obstacle_from_driving():
            rospy.loginfo(
                "BasicDriving -> Obstacle | exists=%s, in_path=%s, dist=%.2f, ttc=%.2f",
                vs.obs_exists,
                vs.obs_in_ego_path,
                vs.obs_dist,
                vs.ttc
            )
            return 'to_obstacle'

        if vs.parking_target_on_route and vs.dist_to_parking_zone < PARKING_APPROACH_DIST:
            rospy.loginfo("BasicDriving -> Parking")
            return 'to_parking'

        return 'stay'


class Intersection(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['to_basic_driving', 'to_obstacle', 'stay']
        )
        self._entry_time = None
        self._timeout = rospy.get_param('~intersection_timeout', 30.0)

    def execute(self, userdata):
        rospy.sleep(0.1)
        publish_state('INTERSECTION')

        if self._entry_time is None:
            self._entry_time = rospy.Time.now()

        if should_enter_obstacle_from_driving():
            self._entry_time = None
            rospy.logwarn("Intersection -> Obstacle")
            return 'to_obstacle'

        if vs.intersection_exit_passed and vs.heading_stable and vs.lane_status_ok:
            vs.intersection_exit_passed = False
            self._entry_time = None
            rospy.loginfo("Intersection -> BasicDriving")
            return 'to_basic_driving'

        if not vs.in_intersection and self._entry_time is not None:
            elapsed = (rospy.Time.now() - self._entry_time).to_sec()
            if elapsed > self._timeout:
                rospy.logwarn(
                    "Intersection timeout %.1fs -> BasicDriving",
                    self._timeout
                )
                self._entry_time = None
                return 'to_basic_driving'

        return 'stay'


# ==========================================
# 2. OBSTACLE States
# ==========================================
class ObstacleManager(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['to_estop', 'to_lane_change', 'to_obs_avoidance']
        )

    def execute(self, userdata):
        rospy.sleep(0.1)
        publish_state('OBS_MANAGER')

        # Priority 1: E-Stop
        if should_enter_estop():
            rospy.logwarn(
                "OBS_MANAGER -> ESTOP | dist=%.2f, ttc=%.2f, in_path=%s, lane_ok=%s",
                vs.obs_dist,
                vs.ttc,
                vs.obs_in_ego_path,
                vs.lane_status_ok
            )
            return 'to_estop'

        # Priority 2: Lane Change
        if should_enter_lane_change():
            rospy.loginfo(
                "OBS_MANAGER -> LANE_CHANGE | direction=%s, dist=%.2f, ttc=%.2f",
                vs.recommended_direction,
                vs.obs_dist,
                vs.ttc
            )
            return 'to_lane_change'

        # Priority 3: Local Obstacle Avoidance
        if should_enter_obs_avoidance():
            rospy.loginfo(
                "OBS_MANAGER -> OBS_AVOIDANCE | dist=%.2f, ttc=%.2f",
                vs.obs_dist,
                vs.ttc
            )
            return 'to_obs_avoidance'

        # Conservative fallback
        rospy.logwarn(
            "OBS_MANAGER fallback -> ESTOP | exists=%s, in_path=%s, dist=%.2f, ttc=%.2f",
            vs.obs_exists,
            vs.obs_in_ego_path,
            vs.obs_dist,
            vs.ttc
        )
        return 'to_estop'


class EStop(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['to_driving', 'to_lane_change', 'stay']
        )

    def execute(self, userdata):
        rospy.sleep(0.1)
        publish_state('ESTOP')

        if is_obstacle_cleared() and vs.lane_status_ok:
            rospy.loginfo("EStop -> Driving: obstacle cleared")
            return 'to_driving'

        if (
            vs.obs_exists and
            vs.obs_in_ego_path and
            vs.adjacent_lane_safe and
            vs.recommended_direction in ["LEFT", "RIGHT"] and
            vs.speed < 0.1 and
            not is_front_distance_estop()
        ):
            rospy.loginfo("EStop -> LaneChange: stopped and adjacent lane safe")
            return 'to_lane_change'

        return 'stay'


class LaneChange(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['to_driving', 'to_estop', 'stay']
        )

    def execute(self, userdata):
        rospy.sleep(0.1)
        publish_state('LANE_CHANGE')

        if should_enter_estop():
            rospy.logwarn("LaneChange -> EStop: obstacle became critical")
            return 'to_estop'

        if vs.lane_change_complete and vs.yaw_stable:
            vs.lane_change_complete = False
            rospy.loginfo("LaneChange -> Driving: planner done")
            return 'to_driving'

        if is_obstacle_cleared():
            rospy.loginfo("LaneChange -> Driving: obstacle cleared")
            return 'to_driving'

        return 'stay'


class ObsAvoidance(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['to_driving', 'to_estop', 'stay']
        )

    def execute(self, userdata):
        rospy.sleep(0.1)
        publish_state('OBS_AVOIDANCE')

        if should_enter_estop():
            rospy.logwarn("ObsAvoidance -> EStop: obstacle became critical")
            return 'to_estop'

        if vs.obs_avoidance_complete and vs.lane_status_ok:
            vs.obs_avoidance_complete = False
            rospy.loginfo("ObsAvoidance -> Driving: planner done")
            return 'to_driving'

        if is_obstacle_cleared():
            rospy.loginfo("ObsAvoidance -> Driving: obstacle cleared")
            return 'to_driving'

        return 'stay'


# ==========================================
# 3. PARKING States
# ==========================================
class LaneFollowingParking(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['to_parking', 'to_driving', 'stay']
        )

    def execute(self, userdata):
        rospy.sleep(0.1)
        publish_state('LANE_FOLLOWING_PARKING')

        if vs.at_parking_start and vs.speed < PARKING_READY_SPEED and vs.heading_stable:
            rospy.loginfo("LaneFollowingParking -> Parking")
            return 'to_parking'

        if not vs.parking_target_on_route:
            rospy.logwarn("LaneFollowingParking -> Driving: parking target cancelled")
            return 'to_driving'

        return 'stay'


class Parking(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['completed', 'stay'])
        self._entry_time = None
        self._timeout = 60.0

    def execute(self, userdata):
        rospy.sleep(0.1)
        publish_state('PARKING_MANEUVER')

        if self._entry_time is None:
            self._entry_time = rospy.Time.now()
            self._timeout = rospy.get_param(
                '/hfsm_decision_maker_node/parking_max_time',
                60.0
            )

        if (
            vs.dist_to_parking_goal < PARKING_POS_TOL and
            abs(vs.yaw_error_parking) < PARKING_YAW_TOL and
            vs.speed < 0.1
        ):
            rospy.loginfo("Parking -> Completed")
            self._entry_time = None
            return 'completed'

        elapsed = (rospy.Time.now() - self._entry_time).to_sec()
        if elapsed > self._timeout:
            rospy.logwarn(
                "Parking timeout %.1fs -> treating as Completed",
                self._timeout
            )
            self._entry_time = None
            return 'completed'

        return 'stay'


# ==========================================
# Main State Machine Construction
# ==========================================
def build_state_machine():
    top_sm = smach.StateMachine(outcomes=['FINISHED'])

    with top_sm:
        # ---------------- INIT ----------------
        smach.StateMachine.add(
            'INIT',
            Init(),
            transitions={
                'initialized': 'DRIVING',
                'stay': 'INIT'
            }
        )

        # ---------------- DRIVING ----------------
        driving_sm = smach.StateMachine(
            outcomes=['out_to_obstacle', 'out_to_parking']
        )

        with driving_sm:
            smach.StateMachine.add(
                'BASIC_DRIVING',
                BasicDriving(),
                transitions={
                    'to_intersection': 'INTERSECTION',
                    'to_obstacle': 'out_to_obstacle',
                    'to_parking': 'out_to_parking',
                    'stay': 'BASIC_DRIVING'
                }
            )

            smach.StateMachine.add(
                'INTERSECTION',
                Intersection(),
                transitions={
                    'to_basic_driving': 'BASIC_DRIVING',
                    'to_obstacle': 'out_to_obstacle',
                    'stay': 'INTERSECTION'
                }
            )

        smach.StateMachine.add(
            'DRIVING',
            driving_sm,
            transitions={
                'out_to_obstacle': 'OBSTACLE',
                'out_to_parking': 'PARKING'
            }
        )

        # ---------------- OBSTACLE ----------------
        obstacle_sm = smach.StateMachine(outcomes=['out_to_driving'])

        with obstacle_sm:
            smach.StateMachine.add(
                'OBS_MANAGER',
                ObstacleManager(),
                transitions={
                    'to_estop': 'ESTOP',
                    'to_lane_change': 'LANE_CHANGE',
                    'to_obs_avoidance': 'OBS_AVOIDANCE'
                }
            )

            smach.StateMachine.add(
                'ESTOP',
                EStop(),
                transitions={
                    'to_driving': 'out_to_driving',
                    'to_lane_change': 'LANE_CHANGE',
                    'stay': 'ESTOP'
                }
            )

            smach.StateMachine.add(
                'LANE_CHANGE',
                LaneChange(),
                transitions={
                    'to_driving': 'out_to_driving',
                    'to_estop': 'ESTOP',
                    'stay': 'LANE_CHANGE'
                }
            )

            smach.StateMachine.add(
                'OBS_AVOIDANCE',
                ObsAvoidance(),
                transitions={
                    'to_driving': 'out_to_driving',
                    'to_estop': 'ESTOP',
                    'stay': 'OBS_AVOIDANCE'
                }
            )

        smach.StateMachine.add(
            'OBSTACLE',
            obstacle_sm,
            transitions={
                'out_to_driving': 'DRIVING'
            }
        )

        # ---------------- PARKING ----------------
        parking_sm = smach.StateMachine(
            outcomes=['out_finished', 'out_to_driving']
        )

        with parking_sm:
            smach.StateMachine.add(
                'LANE_FOLLOWING_PARKING',
                LaneFollowingParking(),
                transitions={
                    'to_parking': 'PARKING_MANEUVER',
                    'to_driving': 'out_to_driving',
                    'stay': 'LANE_FOLLOWING_PARKING'
                }
            )

            smach.StateMachine.add(
                'PARKING_MANEUVER',
                Parking(),
                transitions={
                    'completed': 'out_finished',
                    'stay': 'PARKING_MANEUVER'
                }
            )

        smach.StateMachine.add(
            'PARKING',
            parking_sm,
            transitions={
                'out_finished': 'FINISHED',
                'out_to_driving': 'DRIVING'
            }
        )

    return top_sm


if __name__ == '__main__':
    rospy.init_node('hfsm_decision_maker_node')

    # Parameters for Topics
    odom_topic = rospy.get_param(
        '~odom_topic',
        '/localization_kinematic_state'
    )

    ttc_topic = rospy.get_param(
        '~ttc_topic',
        '/ttc'
    )

    lane_status_topic = rospy.get_param(
        '~lane_status_topic',
        '/lane_status'
    )

    planner_status_topic = rospy.get_param(
        '~planner_status_topic',
        '/local_planner/status'
    )

    # Load parameters
    load_params()

    # Publisher
    state_pub = rospy.Publisher(
        '/hfsm/current_state',
        String,
        queue_size=10
    )

    # Subscribers: core inputs
    rospy.Subscriber(odom_topic, Odometry, odom_callback)
    rospy.Subscriber(ttc_topic, Float32, ttc_callback)
    rospy.Subscriber(lane_status_topic, Bool, lane_status_callback)
    rospy.Subscriber(planner_status_topic, String, planner_status_callback)

    # Subscribers: obstacle evaluator outputs
    rospy.Subscriber('/obstacle/exists', Bool, obs_exists_callback)
    rospy.Subscriber('/obstacle/front_distance', Float32, obs_dist_callback)
    rospy.Subscriber('/obstacle/in_ego_path', Bool, obs_in_ego_path_callback)
    rospy.Subscriber('/obstacle/avoidance_possible', Bool, obs_avoidance_possible_callback)

    # Subscribers: lane change checker outputs
    rospy.Subscriber('/lane_change/adjacent_lane_safe', Bool, adjacent_lane_safe_callback)
    rospy.Subscriber('/lane_change/recommended_direction', String, recommended_direction_callback)

    # Subscribers: route event checker outputs
    rospy.Subscriber('/route_event/dist_to_intersection', Float32, dist_to_intersection_callback)
    rospy.Subscriber('/route_event/in_intersection', Bool, in_intersection_callback)
    rospy.Subscriber('/route_event/intersection_exit_passed', Bool, intersection_exit_passed_callback)
    rospy.Subscriber('/route_event/dist_to_parking_zone', Float32, dist_to_parking_zone_callback)

    # Subscribers: parking checker outputs
    rospy.Subscriber('/parking/target_on_route', Bool, parking_target_on_route_callback)
    rospy.Subscriber('/parking/at_start_position', Bool, at_parking_start_callback)
    rospy.Subscriber('/parking/dist_to_goal', Float32, dist_to_parking_goal_callback)
    rospy.Subscriber('/parking/yaw_error', Float32, yaw_error_parking_callback)

    sm = build_state_machine()

    sis = smach_ros.IntrospectionServer(
        'hfsm_viewer',
        sm,
        '/SM_ROOT'
    )

    sis.start()

    outcome = sm.execute()

    rospy.loginfo("HFSM finished with outcome: %s", outcome)

    rospy.spin()
    sis.stop()

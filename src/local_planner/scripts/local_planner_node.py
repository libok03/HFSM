#!/usr/bin/env python3

import rospy
import math
import numpy as np

from std_msgs.msg import String, Float32, Bool
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, Quaternion
import tf.transformations

from autoware_msgs.msg import DetectedObjectArray

# Helper math functions
def distance_2d(p1, p2):
    return math.hypot(p1.pose.position.x - p2.pose.position.x, 
                      p1.pose.position.y - p2.pose.position.y)

def get_yaw(pose):
    q = pose.orientation
    _, _, yaw = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
    return yaw

class FrenetPath:
    def __init__(self, global_path):
        self.global_path = global_path.poses
        self.s_map = [0.0]
        # Precompute s for each waypoint
        for i in range(1, len(self.global_path)):
            ds = distance_2d(self.global_path[i-1], self.global_path[i])
            self.s_map.append(self.s_map[-1] + ds)
            
    def get_closest_waypoint(self, x, y):
        if not self.global_path:
            return 0
        min_dist = float('inf')
        idx = 0
        for i, p in enumerate(self.global_path):
            dx = x - p.pose.position.x
            dy = y - p.pose.position.y
            dist = math.hypot(dx, dy)
            if dist < min_dist:
                min_dist = dist
                idx = i
        return idx
        
    def cartesian_to_frenet(self, x, y):
        if not self.global_path:
            return 0.0, 0.0
            
        idx = self.get_closest_waypoint(x, y)
        if idx >= len(self.global_path) - 1:
            idx = len(self.global_path) - 2
        elif idx == 0:
            idx = 0
            
        p1 = self.global_path[idx].pose.position
        p2 = self.global_path[idx+1].pose.position
        
        vx = p2.x - p1.x
        vy = p2.y - p1.y
        wx = x - p1.x
        wy = y - p1.y
        
        norm_v = math.hypot(vx, vy)
        if norm_v < 1e-6:
            return self.s_map[idx], math.hypot(wx, wy)
            
        proj = (wx * vx + wy * vy) / norm_v
        cross = (vx * wy - vy * wx) / norm_v # positive if point is to the left
        
        s = self.s_map[idx] + proj
        d = cross
        return s, d
        
    def frenet_to_cartesian(self, s, d):
        if not self.global_path:
            return 0.0, 0.0, 0.0
            
        if s <= self.s_map[0]:
            idx = 0
            proj = s - self.s_map[0]
        elif s >= self.s_map[-1]:
            idx = len(self.global_path) - 2
            proj = s - self.s_map[-1]
        else:
            idx = 0
            for i in range(len(self.s_map)-1):
                if self.s_map[i] <= s <= self.s_map[i+1]:
                    idx = i
                    break
            proj = s - self.s_map[idx]
            
        p1 = self.global_path[idx].pose.position
        p2 = self.global_path[idx+1].pose.position
        
        yaw = math.atan2(p2.y - p1.y, p2.x - p1.x)
        
        base_x = p1.x + proj * math.cos(yaw)
        base_y = p1.y + proj * math.sin(yaw)
        
        # d is positive to the left (counter-clockwise 90 deg)
        normal_yaw = yaw + math.pi/2.0
        
        x = base_x + d * math.cos(normal_yaw)
        y = base_y + d * math.sin(normal_yaw)
        
        return x, y, yaw

class LocalPlannerNode:
    def __init__(self):
        rospy.init_node("local_planner_node")
        
        # --- Params ---
        self.lookahead_distance = rospy.get_param("~lookahead_distance", 50.0)
        self.points_interval = rospy.get_param("~points_interval", 0.5)
        self.hz = rospy.get_param("~hz", 10.0)
        
        self.avoidance_offset = rospy.get_param("~avoidance_offset", 1.5) # meters
        self.lane_change_offset = rospy.get_param("~lane_change_offset", 3.5) # meters
        self.transition_s = rospy.get_param("~transition_s", 25.0) # meters to complete transition
        self.avoidance_hold_s = rospy.get_param("~avoidance_hold_s", 10.0)
        
        # --- State ---
        self.global_path_msg = None
        self.frenet_path = None
        
        self.ego_x = 0.0
        self.ego_y = 0.0
        self.ego_yaw = 0.0
        self.ego_speed = 0.0
        
        self.planner_mode = "STOP"
        self.target_speed = 0.0
        self.lane_change_direction = "NONE"
        self.avoidance_enable = False
        self.stop_request = True
        
        self.objects = []
        
        # Frenet tracking state
        self.current_d_offset = 0.0
        self.maneuver_start_s = 0.0
        self.maneuver_start_d = 0.0
        self.maneuver_target_d = 0.0
        self.maneuver_mode = "LANE_FOLLOW"
        
        # --- Publishers ---
        self.pub_local_path = rospy.Publisher("/local_trajectory/path", Path, queue_size=1)
        self.pub_target_speed = rospy.Publisher("/local_trajectory/target_speed", Float32, queue_size=1)
        self.pub_status = rospy.Publisher("/local_planner/status", String, queue_size=1)
        
        # --- Subscribers ---
        rospy.Subscriber("/global_path", Path, self.global_path_cb)
        rospy.Subscriber("/localization_kinematic_state", Odometry, self.odom_cb)
        rospy.Subscriber("/behavior/planner_mode", String, self.planner_mode_cb)
        rospy.Subscriber("/behavior/target_speed", Float32, self.target_speed_cb)
        rospy.Subscriber("/behavior/lane_change_direction", String, self.lane_change_dir_cb)
        rospy.Subscriber("/behavior/avoidance_enable", Bool, self.avoidance_cb)
        rospy.Subscriber("/behavior/stop_request", Bool, self.stop_request_cb)
        rospy.Subscriber("/estimation/objects", DetectedObjectArray, self.objects_cb)
        
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.hz), self.timer_cb)
        rospy.loginfo("local_planner_node initialized")
        
    def global_path_cb(self, msg):
        self.global_path_msg = msg
        if len(msg.poses) > 1:
            self.frenet_path = FrenetPath(msg)
        
    def odom_cb(self, msg):
        self.ego_x = msg.pose.pose.position.x
        self.ego_y = msg.pose.pose.position.y
        self.ego_yaw = get_yaw(msg.pose.pose)
        self.ego_speed = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
        
    def planner_mode_cb(self, msg):
        self.planner_mode = msg.data
        
    def target_speed_cb(self, msg):
        self.target_speed = msg.data
        
    def lane_change_dir_cb(self, msg):
        self.lane_change_direction = msg.data
        
    def avoidance_cb(self, msg):
        self.avoidance_enable = msg.data
        
    def stop_request_cb(self, msg):
        self.stop_request = msg.data
        
    def objects_cb(self, msg):
        self.objects = msg.objects
        
    def generate_quintic_polynomial(self, start_s, start_d, end_s, end_d):
        # Generates a smooth transition curve d(s) using quintic ease-in-out
        delta_s = end_s - start_s
        if delta_s <= 0.1:
            return lambda s: end_d
            
        def blend(s):
            if s <= start_s: return start_d
            if s >= end_s: return end_d
            t = (s - start_s) / delta_s
            # quintic equation: 6t^5 - 15t^4 + 10t^3 ensures smooth velocity and acceleration
            t_blend = 6*(t**5) - 15*(t**4) + 10*(t**3)
            return start_d + (end_d - start_d) * t_blend
            
        return blend
        
    def timer_cb(self, event):
        if self.frenet_path is None or not self.global_path_msg:
            return
            
        ego_s, ego_d = self.frenet_path.cartesian_to_frenet(self.ego_x, self.ego_y)
        
        # 1. Determine target 'd' offset based on Behavior States
        target_d = 0.0
        
        if self.planner_mode == "LANE_CHANGE":
            if self.lane_change_direction == "LEFT":
                target_d = self.lane_change_offset
            elif self.lane_change_direction == "RIGHT":
                target_d = -self.lane_change_offset
                
        elif self.planner_mode == "AVOIDANCE" and self.avoidance_enable:
            # Default to left avoidance for now (assuming right-hand driving)
            target_d = self.avoidance_offset
            
        # 2. Smoothly transition d_offset over S distance if target changed
        if (
            self.maneuver_mode != self.planner_mode or
            abs(self.maneuver_target_d - target_d) > 0.1
        ):
            self.maneuver_start_s = ego_s
            self.maneuver_start_d = self.current_d_offset
            self.maneuver_target_d = target_d
            self.maneuver_mode = self.planner_mode

        end_s = self.maneuver_start_s + self.transition_s
        d_function = self.generate_quintic_polynomial(
            self.maneuver_start_s,
            self.maneuver_start_d,
            end_s,
            self.maneuver_target_d
        )

        self.current_d_offset = d_function(ego_s)
            
        # 3. Build local path by translating (s, d) back to Cartesian (x, y)
        local_path = Path()
        local_path.header.frame_id = self.global_path_msg.header.frame_id
        local_path.header.stamp = rospy.Time.now()
        
        s_steps = np.arange(ego_s, ego_s + self.lookahead_distance, self.points_interval)
        
        for s in s_steps:
            d = d_function(s)
            x, y, yaw = self.frenet_path.frenet_to_cartesian(s, d)
            
            pose = PoseStamped()
            pose.header = local_path.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            
            q = tf.transformations.quaternion_from_euler(0, 0, yaw)
            pose.pose.orientation = Quaternion(*q)
            
            local_path.poses.append(pose)
            
        self.pub_local_path.publish(local_path)
        
        # 4. Handle speed
        final_speed = self.target_speed
        if self.stop_request or self.planner_mode == "STOP":
            final_speed = 0.0
            
        self.pub_target_speed.publish(Float32(final_speed))
        
        # 5. Publish status feedback back to HFSM
        status = "ACTIVE"
        if self.planner_mode == "LANE_CHANGE" and abs(self.current_d_offset - target_d) < 0.2:
            status = "LANE_CHANGE_DONE"
        elif (
            self.planner_mode == "AVOIDANCE" and
            abs(self.current_d_offset - target_d) < 0.2 and
            ego_s >= self.maneuver_start_s + self.transition_s + self.avoidance_hold_s
        ):
            status = "OBS_AVOIDANCE_DONE"
            
        self.pub_status.publish(String(status))

if __name__ == "__main__":
    try:
        node = LocalPlannerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

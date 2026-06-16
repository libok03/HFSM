#!/usr/bin/env python3

import rospy
import math
import xml.etree.ElementTree as ET

from std_msgs.msg import Bool, Float32, String
from nav_msgs.msg import Odometry
from autoware_msgs.msg import DetectedObjectArray


EARTH_RADIUS_M = 6378137.0


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def distance_2d(p1, p2):
    dx = p1[0] - p2[0]
    dy = p1[1] - p2[1]
    return math.sqrt(dx * dx + dy * dy)


def project_point_to_segment(px, py, ax, ay, bx, by):
    abx = bx - ax
    aby = by - ay
    apx = px - ax
    apy = py - ay

    ab2 = abx * abx + aby * aby
    if ab2 < 1e-9:
        return ax, ay, 0.0, math.sqrt((px - ax) ** 2 + (py - ay) ** 2)

    t = (apx * abx + apy * aby) / ab2
    t_clamped = max(0.0, min(1.0, t))

    qx = ax + t_clamped * abx
    qy = ay + t_clamped * aby
    dist = math.sqrt((px - qx) ** 2 + (py - qy) ** 2)

    return qx, qy, t_clamped, dist


class LaneletInfo:
    def __init__(self, lanelet_id):
        self.id = lanelet_id
        self.left_way_id = None
        self.right_way_id = None
        self.regulatory_element_ids = []
        self.follows_ids = []
        self.turn_direction = "straight"
        self.subtype = ""
        self.left_points = []
        self.right_points = []
        self.centerline = []
        self.heading = 0.0
        self.is_intersection = False


class Lanelet2EventEvaluatorNode:
    def __init__(self):
        rospy.init_node("lanelet2_event_evaluator_node")

        # ---------------- Parameters ----------------
        self.osm_file = rospy.get_param("~osm_file", "")

        self.odom_topic = rospy.get_param(
            "~odom_topic",
            "/localization_kinematic_state"
        )

        self.obstacle_topic = rospy.get_param(
            "~obstacle_topic",
            "/estimation/objects"
        )

        # OSM lat/lon -> local meter map origin
        self.map_origin_lat = rospy.get_param("~map_origin_lat", 37.22922449)
        self.map_origin_lon = rospy.get_param("~map_origin_lon", 126.76910549)

        # If odometry map frame has offset/yaw compared to converted OSM local frame
        self.map_offset_x = rospy.get_param("~map_offset_x", 0.0)
        self.map_offset_y = rospy.get_param("~map_offset_y", 0.0)
        self.map_yaw_offset = rospy.get_param("~map_yaw_offset", 0.0)

        self.lane_match_threshold = rospy.get_param("~lane_match_threshold", 3.0)
        self.intersection_radius = rospy.get_param("~intersection_radius", 8.0)

        self.lane_width = rospy.get_param("~lane_width", 3.5)
        self.adjacent_lane_lateral_min = rospy.get_param(
            "~adjacent_lane_lateral_min",
            0.5 * self.lane_width
        )
        self.adjacent_lane_lateral_max = rospy.get_param(
            "~adjacent_lane_lateral_max",
            1.5 * self.lane_width
        )

        self.LC_FRONT_SAFE_DIST = rospy.get_param("~LC_FRONT_SAFE_DIST", 20.0)
        self.LC_REAR_SAFE_DIST = rospy.get_param("~LC_REAR_SAFE_DIST", 15.0)

        self.same_direction_yaw_threshold = rospy.get_param(
            "~same_direction_yaw_threshold_deg",
            35.0
        )
        self.same_direction_yaw_threshold = math.radians(
            self.same_direction_yaw_threshold
        )

        self.loop_hz = rospy.get_param("~loop_hz", 10.0)

        self.default_dist_to_intersection = rospy.get_param(
            "~default_dist_to_intersection",
            999.9
        )
        self.default_dist_to_parking_zone = rospy.get_param(
            "~default_dist_to_parking_zone",
            999.9
        )

        # ---------------- State ----------------
        self.odom_received = False
        self.ego_x = 0.0
        self.ego_y = 0.0
        self.ego_yaw = 0.0

        self.objects = []

        self.prev_in_intersection = False

        self.nodes = {}
        self.ways = {}
        self.regulatory_elements = {}
        self.lanelets = []

        # ---------------- Publishers ----------------
        self.pub_lane_status = rospy.Publisher(
            "/lane_status",
            Bool,
            queue_size=10
        )

        self.pub_adjacent_lane_safe = rospy.Publisher(
            "/lane_change/adjacent_lane_safe",
            Bool,
            queue_size=10
        )

        self.pub_recommended_direction = rospy.Publisher(
            "/lane_change/recommended_direction",
            String,
            queue_size=10
        )

        self.pub_dist_to_intersection = rospy.Publisher(
            "/route_event/dist_to_intersection",
            Float32,
            queue_size=10
        )

        self.pub_in_intersection = rospy.Publisher(
            "/route_event/in_intersection",
            Bool,
            queue_size=10
        )

        self.pub_intersection_exit_passed = rospy.Publisher(
            "/route_event/intersection_exit_passed",
            Bool,
            queue_size=10
        )

        self.pub_dist_to_parking_zone = rospy.Publisher(
            "/route_event/dist_to_parking_zone",
            Float32,
            queue_size=10
        )

        # ---------------- Subscribers ----------------
        rospy.Subscriber(
            self.odom_topic,
            Odometry,
            self.odom_callback,
            queue_size=1
        )

        rospy.Subscriber(
            self.obstacle_topic,
            DetectedObjectArray,
            self.objects_callback,
            queue_size=1
        )

        # ---------------- Init map ----------------
        if self.osm_file == "":
            rospy.logerr("~osm_file is empty. Cannot load Lanelet2 OSM.")
        else:
            self.load_osm(self.osm_file)

        self.timer = rospy.Timer(
            rospy.Duration(1.0 / self.loop_hz),
            self.timer_callback
        )

        rospy.loginfo(
            "lanelet2_event_evaluator_node initialized. lanelets=%d",
            len(self.lanelets)
        )

    # ============================================================
    # Coordinate transform
    # ============================================================
    def latlon_to_local_xy(self, lat, lon):
        lat0 = math.radians(self.map_origin_lat)
        lon0 = math.radians(self.map_origin_lon)

        lat_rad = math.radians(lat)
        lon_rad = math.radians(lon)

        x = EARTH_RADIUS_M * math.cos(lat0) * (lon_rad - lon0)
        y = EARTH_RADIUS_M * (lat_rad - lat0)

        # Optional map yaw + offset
        cy = math.cos(self.map_yaw_offset)
        sy = math.sin(self.map_yaw_offset)

        xr = cy * x - sy * y + self.map_offset_x
        yr = sy * x + cy * y + self.map_offset_y

        return xr, yr

    # ============================================================
    # OSM parsing
    # ============================================================
    def load_osm(self, osm_file):
        rospy.loginfo("Loading Lanelet2 OSM: %s", osm_file)

        tree = ET.parse(osm_file)
        root = tree.getroot()

        # Parse nodes
        for elem in root.findall("node"):
            node_id = elem.attrib["id"]
            lat = float(elem.attrib["lat"])
            lon = float(elem.attrib["lon"])
            x, y = self.latlon_to_local_xy(lat, lon)
            self.nodes[node_id] = (x, y)

        # Parse ways
        for elem in root.findall("way"):
            way_id = elem.attrib["id"]
            refs = []
            tags = {}

            for child in elem:
                if child.tag == "nd":
                    refs.append(child.attrib["ref"])
                elif child.tag == "tag":
                    tags[child.attrib["k"]] = child.attrib["v"]

            points = []
            for ref in refs:
                if ref in self.nodes:
                    points.append(self.nodes[ref])

            self.ways[way_id] = {
                "refs": refs,
                "points": points,
                "tags": tags
            }

        # First pass: regulatory elements
        for elem in root.findall("relation"):
            rel_id = elem.attrib["id"]
            tags = {}
            members = []

            for child in elem:
                if child.tag == "member":
                    members.append(child.attrib)
                elif child.tag == "tag":
                    tags[child.attrib["k"]] = child.attrib["v"]

            if tags.get("type", "") == "regulatory_element":
                self.regulatory_elements[rel_id] = {
                    "tags": tags,
                    "members": members
                }

        # Second pass: lanelets
        for elem in root.findall("relation"):
            rel_id = elem.attrib["id"]
            tags = {}
            members = []

            for child in elem:
                if child.tag == "member":
                    members.append(child.attrib)
                elif child.tag == "tag":
                    tags[child.attrib["k"]] = child.attrib["v"]

            if tags.get("type", "") != "lanelet":
                continue

            lanelet = LaneletInfo(rel_id)
            lanelet.subtype = tags.get("subtype", "")
            lanelet.turn_direction = tags.get("turn_direction", "straight")

            for m in members:
                role = m.get("role", "")
                m_type = m.get("type", "")
                ref = m.get("ref", "")

                if m_type == "way" and role == "left":
                    lanelet.left_way_id = ref
                elif m_type == "way" and role == "right":
                    lanelet.right_way_id = ref
                elif role == "regulatory_element":
                    lanelet.regulatory_element_ids.append(ref)
                elif role == "follows":
                    lanelet.follows_ids.append(ref)

            if lanelet.left_way_id not in self.ways:
                continue
            if lanelet.right_way_id not in self.ways:
                continue

            lanelet.left_points = self.ways[lanelet.left_way_id]["points"]
            lanelet.right_points = self.ways[lanelet.right_way_id]["points"]

            if len(lanelet.left_points) < 2 or len(lanelet.right_points) < 2:
                continue

            lanelet.centerline = self.make_centerline(
                lanelet.left_points,
                lanelet.right_points
            )

            if len(lanelet.centerline) < 2:
                continue

            lanelet.heading = self.compute_polyline_heading(lanelet.centerline)

            lanelet.is_intersection = self.is_intersection_lanelet(lanelet)

            self.lanelets.append(lanelet)

        rospy.loginfo(
            "OSM loaded. nodes=%d, ways=%d, regs=%d, lanelets=%d",
            len(self.nodes),
            len(self.ways),
            len(self.regulatory_elements),
            len(self.lanelets)
        )

    def make_centerline(self, left_points, right_points):
        n = min(len(left_points), len(right_points))
        centerline = []

        for i in range(n):
            lx, ly = left_points[i]
            rx, ry = right_points[i]
            cx = 0.5 * (lx + rx)
            cy = 0.5 * (ly + ry)
            centerline.append((cx, cy))

        return centerline

    def compute_polyline_heading(self, polyline):
        if len(polyline) < 2:
            return 0.0

        p0 = polyline[0]
        p1 = polyline[-1]

        return math.atan2(p1[1] - p0[1], p1[0] - p0[0])

    def is_intersection_lanelet(self, lanelet):
        if lanelet.turn_direction in ["left", "right"]:
            return True

        for reg_id in lanelet.regulatory_element_ids:
            reg = self.regulatory_elements.get(reg_id)
            if reg is None:
                continue

            subtype = reg["tags"].get("subtype", "")
            if subtype == "traffic_light":
                return True

        return False

    # ============================================================
    # Callbacks
    # ============================================================
    def odom_callback(self, msg):
        self.odom_received = True
        self.ego_x = msg.pose.pose.position.x
        self.ego_y = msg.pose.pose.position.y
        self.ego_yaw = yaw_from_quaternion(msg.pose.pose.orientation)

    def objects_callback(self, msg):
        self.objects = msg.objects

    # ============================================================
    # Lanelet matching
    # ============================================================
    def nearest_point_on_polyline(self, px, py, polyline):
        if len(polyline) < 2:
            return None

        best = {
            "dist": float("inf"),
            "qx": 0.0,
            "qy": 0.0,
            "seg_idx": 0,
            "t": 0.0
        }

        for i in range(len(polyline) - 1):
            ax, ay = polyline[i]
            bx, by = polyline[i + 1]

            qx, qy, t, dist = project_point_to_segment(
                px, py, ax, ay, bx, by
            )

            if dist < best["dist"]:
                best["dist"] = dist
                best["qx"] = qx
                best["qy"] = qy
                best["seg_idx"] = i
                best["t"] = t

        return best

    def find_current_lanelet(self):
        if not self.odom_received:
            return None, float("inf")

        best_lanelet = None
        best_dist = float("inf")

        for lanelet in self.lanelets:
            projection = self.nearest_point_on_polyline(
                self.ego_x,
                self.ego_y,
                lanelet.centerline
            )

            if projection is None:
                continue

            dist = projection["dist"]

            # Prefer lanelets with similar heading
            heading_error = abs(normalize_angle(lanelet.heading - self.ego_yaw))

            if heading_error > math.pi * 0.75:
                dist += 5.0

            if dist < best_dist:
                best_dist = dist
                best_lanelet = lanelet

        return best_lanelet, best_dist

    # ============================================================
    # Route event
    # ============================================================
    def compute_intersection_event(self, current_lanelet, lane_status):
        if not lane_status:
            return self.default_dist_to_intersection, False

        in_intersection = False
        dist_to_intersection = self.default_dist_to_intersection

        if current_lanelet is not None and current_lanelet.is_intersection:
            proj = self.nearest_point_on_polyline(
                self.ego_x,
                self.ego_y,
                current_lanelet.centerline
            )
            if proj is not None and proj["dist"] <= self.intersection_radius:
                in_intersection = True
                dist_to_intersection = 0.0

        if in_intersection:
            return dist_to_intersection, True

        # Find nearest intersection lanelet ahead of ego
        for lanelet in self.lanelets:
            if not lanelet.is_intersection:
                continue

            proj = self.nearest_point_on_polyline(
                self.ego_x,
                self.ego_y,
                lanelet.centerline
            )

            if proj is None:
                continue

            vx = proj["qx"] - self.ego_x
            vy = proj["qy"] - self.ego_y

            forward = math.cos(self.ego_yaw) * vx + math.sin(self.ego_yaw) * vy
            lateral = -math.sin(self.ego_yaw) * vx + math.cos(self.ego_yaw) * vy

            if forward < 0.0:
                continue

            # 너무 옆에 있는 교차로 lanelet은 제외
            if abs(lateral) > self.lane_width * 3.0:
                continue

            euclidean_dist = math.sqrt(vx * vx + vy * vy)

            if euclidean_dist < dist_to_intersection:
                dist_to_intersection = euclidean_dist

        return dist_to_intersection, False

    # ============================================================
    # Lane change safety
    # ============================================================
    def has_adjacent_lane_candidate(self, current_lanelet, direction):
        if current_lanelet is None:
            return False

        sign = 1.0 if direction == "LEFT" else -1.0

        for lanelet in self.lanelets:
            if lanelet.id == current_lanelet.id:
                continue

            heading_error = abs(
                normalize_angle(lanelet.heading - current_lanelet.heading)
            )

            if heading_error > self.same_direction_yaw_threshold:
                continue

            proj = self.nearest_point_on_polyline(
                self.ego_x,
                self.ego_y,
                lanelet.centerline
            )

            if proj is None:
                continue

            vx = proj["qx"] - self.ego_x
            vy = proj["qy"] - self.ego_y

            lateral = (
                -math.sin(self.ego_yaw) * vx +
                math.cos(self.ego_yaw) * vy
            )

            forward = (
                math.cos(self.ego_yaw) * vx +
                math.sin(self.ego_yaw) * vy
            )

            if abs(forward) > 15.0:
                continue

            if sign * lateral < self.adjacent_lane_lateral_min:
                continue

            if sign * lateral > self.adjacent_lane_lateral_max:
                continue

            return True

        return False

    def is_adjacent_lane_clear(self, direction):
        sign = 1.0 if direction == "LEFT" else -1.0

        for obj in self.objects:
            ox = obj.pose.position.x
            oy = obj.pose.position.y

            dx = ox - self.ego_x
            dy = oy - self.ego_y

            local_x = math.cos(self.ego_yaw) * dx + math.sin(self.ego_yaw) * dy
            local_y = -math.sin(self.ego_yaw) * dx + math.cos(self.ego_yaw) * dy

            # Object must be in target adjacent lane band
            lateral_abs = sign * local_y

            if lateral_abs < self.adjacent_lane_lateral_min:
                continue

            if lateral_abs > self.adjacent_lane_lateral_max:
                continue

            # Front safety
            if 0.0 <= local_x <= self.LC_FRONT_SAFE_DIST:
                return False

            # Rear safety
            if -self.LC_REAR_SAFE_DIST <= local_x < 0.0:
                return False

        return True

    def compute_lane_change_decision(self, current_lanelet, lane_status):
        if not lane_status:
            return False, "NONE"

        left_exists = self.has_adjacent_lane_candidate(current_lanelet, "LEFT")
        right_exists = self.has_adjacent_lane_candidate(current_lanelet, "RIGHT")

        left_safe = left_exists and self.is_adjacent_lane_clear("LEFT")
        right_safe = right_exists and self.is_adjacent_lane_clear("RIGHT")

        if left_safe:
            return True, "LEFT"

        if right_safe:
            return True, "RIGHT"

        return False, "NONE"

    # ============================================================
    # Timer
    # ============================================================
    def timer_callback(self, event):
        if not self.odom_received or len(self.lanelets) == 0:
            self.publish_default()
            return

        current_lanelet, lane_dist = self.find_current_lanelet()

        lane_status = (
            current_lanelet is not None and
            lane_dist <= self.lane_match_threshold
        )

        dist_to_intersection, in_intersection = self.compute_intersection_event(
            current_lanelet,
            lane_status
        )

        intersection_exit_passed = (
            self.prev_in_intersection and
            not in_intersection
        )

        self.prev_in_intersection = in_intersection

        adjacent_lane_safe, recommended_direction = self.compute_lane_change_decision(
            current_lanelet,
            lane_status
        )

        self.pub_lane_status.publish(Bool(lane_status))
        self.pub_adjacent_lane_safe.publish(Bool(adjacent_lane_safe))
        self.pub_recommended_direction.publish(String(recommended_direction))

        self.pub_dist_to_intersection.publish(Float32(dist_to_intersection))
        self.pub_in_intersection.publish(Bool(in_intersection))
        self.pub_intersection_exit_passed.publish(Bool(intersection_exit_passed))

        # Parking zone is not implemented in this first version.
        self.pub_dist_to_parking_zone.publish(
            Float32(self.default_dist_to_parking_zone)
        )

        rospy.loginfo_throttle(
            1.0,
            "LaneletEval | lane_status=%s lane_dist=%.2f intersection_dist=%.2f "
            "in_intersection=%s lc_safe=%s direction=%s",
            lane_status,
            lane_dist,
            dist_to_intersection,
            in_intersection,
            adjacent_lane_safe,
            recommended_direction
        )

    def publish_default(self):
        self.pub_lane_status.publish(Bool(False))
        self.pub_adjacent_lane_safe.publish(Bool(False))
        self.pub_recommended_direction.publish(String("NONE"))

        self.pub_dist_to_intersection.publish(
            Float32(self.default_dist_to_intersection)
        )
        self.pub_in_intersection.publish(Bool(False))
        self.pub_intersection_exit_passed.publish(Bool(False))
        self.pub_dist_to_parking_zone.publish(
            Float32(self.default_dist_to_parking_zone)
        )


if __name__ == "__main__":
    try:
        node = Lanelet2EventEvaluatorNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
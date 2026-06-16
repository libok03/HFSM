# HFSM ROS Workspace

Hierarchical finite state machine (HFSM) decision-making stack for autonomous driving behavior selection.

This workspace contains the decision node, behavior command router, local planner, evaluator nodes, and dummy scenario publisher used to test state transitions.

## Packages

| Package | Role |
| --- | --- |
| `hfsm_decision_maker` | HFSM state machine, lanelet2 route evaluator, obstacle evaluator, global path publisher |
| `behavior_router` | Converts `/hfsm/current_state` into planner commands |
| `local_planner` | Generates local path and target speed from behavior commands |
| `hfsm_dummy_pub` | Publishes test topics for keyboard-driven HFSM scenarios |

## Main Nodes

| Node | Package | Purpose |
| --- | --- | --- |
| `hfsm_node.py` | `hfsm_decision_maker` | Publishes `/hfsm/current_state` |
| `lanelet2_event_evaluator_node.py` | `hfsm_decision_maker` | Publishes lane change and route event topics |
| `obstacle_evaluator_node.py` | `hfsm_decision_maker` | Publishes `/obstacle/*` and `/ttc` |
| `global_path_publisher_node.py` | `hfsm_decision_maker` | Publishes `/global_path` |
| `behavior_router_node.py` | `behavior_router` | Publishes `/behavior/*` planner command topics |
| `local_planner_node.py` | `local_planner` | Publishes `/local_trajectory/*` and `/local_planner/status` |
| `dummy_publisher_node.py` | `hfsm_dummy_pub` | Publishes dummy evaluator and ego-state inputs |

## Build

From the workspace root:

```bash
cd ~/HFSM
catkin_make
source devel/setup.bash
```

The top-level `src/CMakeLists.txt` is kept local to this repo so newer CMake versions can build legacy ROS Noetic catkin dependencies.

## Run Full HFSM Stack

```bash
cd ~/HFSM
source devel/setup.bash
roslaunch hfsm_decision_maker hfsm.launch
```

This launch starts:

```text
hfsm_decision_maker_node
lanelet2_event_evaluator_node
behavior_router_node
global_path_publisher_node
obstacle_evaluator_node
local_planner_node
```

## Run Dummy HFSM Test

Terminal 1:

```bash
cd ~/HFSM
source devel/setup.bash
roslaunch hfsm_dummy_pub hfsm_test.launch
```

Terminal 2:

```bash
cd ~/HFSM
source devel/setup.bash
rosrun hfsm_dummy_pub dummy_publisher_node.py
```

Terminal 3:

```bash
rostopic echo /hfsm/current_state
```

Use keys `1`-`9` and `0` in the dummy publisher terminal to switch scenarios.

## Key Topics

### HFSM Inputs

| Topic | Type | Purpose |
| --- | --- | --- |
| `/localization_kinematic_state` | `nav_msgs/Odometry` | Ego pose and speed |
| `/ttc` | `std_msgs/Float32` | Time to collision |
| `/lane_status` | `std_msgs/Bool` | Lane validity |
| `/obstacle/exists` | `std_msgs/Bool` | Obstacle existence |
| `/obstacle/front_distance` | `std_msgs/Float32` | Front obstacle distance |
| `/obstacle/in_ego_path` | `std_msgs/Bool` | Obstacle is inside ego path |
| `/obstacle/avoidance_possible` | `std_msgs/Bool` | Local avoidance availability |
| `/lane_change/adjacent_lane_safe` | `std_msgs/Bool` | Adjacent lane safety |
| `/lane_change/recommended_direction` | `std_msgs/String` | `LEFT`, `RIGHT`, or `NONE` |
| `/route_event/dist_to_intersection` | `std_msgs/Float32` | Distance to intersection |
| `/route_event/in_intersection` | `std_msgs/Bool` | Intersection occupancy |
| `/route_event/intersection_exit_passed` | `std_msgs/Bool` | Intersection exit flag |
| `/local_planner/status` | `std_msgs/String` | Planner completion feedback |

### HFSM Output

| Topic | Type | Values |
| --- | --- | --- |
| `/hfsm/current_state` | `std_msgs/String` | `INIT`, `BASIC_DRIVING`, `INTERSECTION`, `OBS_MANAGER`, `ESTOP`, `LANE_CHANGE`, `OBS_AVOIDANCE`, `LANE_FOLLOWING_PARKING`, `PARKING_MANEUVER` |

### Behavior Router Outputs

| Topic | Type | Purpose |
| --- | --- | --- |
| `/behavior/planner_mode` | `std_msgs/String` | `STOP`, `LANE_FOLLOW`, `LANE_CHANGE`, `AVOIDANCE` |
| `/behavior/target_speed` | `std_msgs/Float32` | Target speed in m/s |
| `/behavior/lane_change_direction` | `std_msgs/String` | Lane change direction |
| `/behavior/avoidance_enable` | `std_msgs/Bool` | Enables local avoidance offset |
| `/behavior/stop_request` | `std_msgs/Bool` | Requests full stop |

## State Transition Notes

The obstacle branch priority is:

```text
OBS_MANAGER
→ ESTOP
→ LANE_CHANGE
→ OBS_AVOIDANCE
→ fallback ESTOP
```

Lane change requires both:

```text
/lane_change/adjacent_lane_safe = True
/lane_change/recommended_direction in ["LEFT", "RIGHT"]
```

`OBS_AVOIDANCE_DONE` is published by `local_planner_node.py` only after the avoidance transition distance and hold distance are completed.

## Documentation

- Detailed architecture: [`src/hfsm_decision_maker/hfsm_architecture.md`](src/hfsm_decision_maker/hfsm_architecture.md)
- State transition logic: [`src/hfsm_decision_maker/hfsm_state_transition_logic.md`](src/hfsm_decision_maker/hfsm_state_transition_logic.md)

## Repository Hygiene

Generated ROS workspace outputs are intentionally excluded:

```text
build/
devel/
install/
log/
__pycache__/
```

Run `catkin_make` locally to regenerate `build/` and `devel/`.

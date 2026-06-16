# HFSM Node State Transition Logic

## 1. 개요

`hfsm_node.py`는 차량의 주행 상태를 판단하는 상위 의사결정 노드이다.
각종 입력 토픽을 받아 내부 변수인 `VehicleState`에 저장하고, 현재 state에서 조건을 검사하여 다음 state로 전이한다.

전체 HFSM 구조는 다음과 같다.

```text
INIT
  ↓
DRIVING
  ├── BASIC_DRIVING
  └── INTERSECTION

OBSTACLE
  ├── OBS_MANAGER
  ├── ESTOP
  ├── LANE_CHANGE
  └── OBS_AVOIDANCE

PARKING
  ├── LANE_FOLLOWING_PARKING
  └── PARKING_MANEUVER

FINISHED
```

단, `/hfsm/current_state`로 실제 publish되는 값은 하위 state 이름이다.

```text
INIT
BASIC_DRIVING
INTERSECTION
OBS_MANAGER
ESTOP
LANE_CHANGE
OBS_AVOIDANCE
LANE_FOLLOWING_PARKING
PARKING_MANEUVER
```

`DRIVING`, `OBSTACLE`, `PARKING`은 내부 계층 state이며, 일반적으로 `/hfsm/current_state`로 직접 publish되지는 않는다.

## 2. 주요 입력 토픽과 내부 변수

### 2.1 Ego Vehicle State

| Topic | Type | 내부 변수 | 의미 |
| --- | --- | --- | --- |
| `/localization_kinematic_state` | `Odometry` | `vs.odom_received` | 위치/속도 정보 수신 여부 |
| `/localization_kinematic_state` | `Odometry` | `vs.speed` | ego 차량 속도 |

`INIT` state에서 `/localization_kinematic_state`가 수신되면 `vs.odom_received = True`가 되고, HFSM은 `BASIC_DRIVING`으로 전이한다.

```text
/localization_kinematic_state 수신
→ vs.odom_received = True
→ INIT → BASIC_DRIVING
```

### 2.2 Obstacle State

| Topic | Type | 내부 변수 | 의미 |
| --- | --- | --- | --- |
| `/obstacle/exists` | `Bool` | `vs.obs_exists` | 장애물 존재 여부 |
| `/obstacle/front_distance` | `Float32` | `vs.obs_dist` | 전방 장애물 거리 |
| `/obstacle/in_ego_path` | `Bool` | `vs.obs_in_ego_path` | 장애물이 ego path 내부에 있는지 |
| `/obstacle/avoidance_possible` | `Bool` | `vs.obs_avoidance_possible` | local avoidance 가능 여부 |
| `/ttc` | `Float32` | `vs.ttc` | Time-to-Collision |

장애물 판단에 사용되는 주요 기준값은 다음과 같다.

```python
TTC_ESTOP = 1.5
TTC_LANE_CHANGE = 4.0
TTC_SAFE = 5.0

ESTOP_DIST = 5.0
OBSTACLE_SAFE_DIST = 15.0
```

```text
ttc < 1.5 s        → 긴급정지 후보
1.5 s ≤ ttc < 4 s → 차선 변경 후보
ttc < 5 s          → 일반 주행에는 위험

obs_dist < 5 m     → 긴급정지 후보
obs_dist < 15 m    → 장애물 대응 필요
```

### 2.3 Lane Change State

| Topic | Type | 내부 변수 | 의미 |
| --- | --- | --- | --- |
| `/lane_change/adjacent_lane_safe` | `Bool` | `vs.adjacent_lane_safe` | 인접 차선 안전 여부 |
| `/lane_change/recommended_direction` | `String` | `vs.recommended_direction` | 추천 차선 변경 방향 |

`recommended_direction`은 다음 값 중 하나로 정규화된다.

```text
LEFT
RIGHT
NONE
```

현재 구현에서는 `should_enter_lane_change()`가 다음 조건을 모두 확인한다.

```text
/lane_change/adjacent_lane_safe = True
AND
/lane_change/recommended_direction in ["LEFT", "RIGHT"]
```

따라서 방향이 `NONE`이면 인접 차선이 안전해도 `LANE_CHANGE`로 전이하지 않는다.

### 2.4 Intersection State

| Topic | Type | 내부 변수 | 의미 |
| --- | --- | --- | --- |
| `/route_event/dist_to_intersection` | `Float32` | `vs.dist_to_intersection` | 교차로까지 거리 |
| `/route_event/in_intersection` | `Bool` | `vs.in_intersection` | 현재 교차로 내부 여부 |
| `/route_event/intersection_exit_passed` | `Bool` | `vs.intersection_exit_passed` | 교차로 출구 통과 여부 |

교차로 접근 판단 기준은 다음과 같다.

```python
INTERSECTION_ENTER_DIST = 20.0
```

교차로까지 거리가 20 m 미만이거나, 현재 교차로 내부에 있으면 `INTERSECTION` state로 전이한다.

### 2.5 Lane Status

| Topic | Type | 내부 변수 | 의미 |
| --- | --- | --- | --- |
| `/lane_status` | `Bool` | `vs.lane_status_ok` | 차선 상태 정상 여부 |

`/lane_status = False`가 되면 기본 주행 중에는 장애물 대응 계층으로 전이한다.

```text
/lane_status = False
→ BASIC_DRIVING → OBS_MANAGER
```

또한 `should_enter_estop()` 조건에서도 `/lane_status = False`는 ESTOP 조건으로 사용된다.

### 2.6 Local Planner Feedback

| Topic | Type | 값 | 내부 변수 | 의미 |
| --- | --- | --- | --- | --- |
| `/local_planner/status` | `String` | `LANE_CHANGE_DONE` | `vs.lane_change_complete` | 차선 변경 완료 |
| `/local_planner/status` | `String` | `OBS_AVOIDANCE_DONE` | `vs.obs_avoidance_complete` | 회피 완료 |
| `/local_planner/status` | `String` | `PARKING_DONE` | `vs.parking_space_found` | 주차 관련 완료 신호 |

주요 전이에 직접 사용되는 값은 다음과 같다.

```text
LANE_CHANGE_DONE
→ LANE_CHANGE → BASIC_DRIVING

OBS_AVOIDANCE_DONE
→ OBS_AVOIDANCE → BASIC_DRIVING
```

`local_planner_node.py`는 `OBS_AVOIDANCE_DONE`을 회피 offset 도달 즉시 내지 않고, `transition_s + avoidance_hold_s` 거리 이후에 발행한다.

### 2.7 Parking State

| Topic | Type | 내부 변수 | 의미 |
| --- | --- | --- | --- |
| `/parking/target_on_route` | `Bool` | `vs.parking_target_on_route` | 경로상 주차 목표 존재 여부 |
| `/route_event/dist_to_parking_zone` | `Float32` | `vs.dist_to_parking_zone` | 주차 구역까지 거리 |
| `/parking/at_start_position` | `Bool` | `vs.at_parking_start` | 주차 시작 위치 도달 여부 |
| `/parking/dist_to_goal` | `Float32` | `vs.dist_to_parking_goal` | 주차 목표점까지 거리 |
| `/parking/yaw_error` | `Float32` | `vs.yaw_error_parking` | 주차 목표 yaw 오차 |

주차 관련 기준값은 다음과 같다.

```python
PARKING_APPROACH_DIST = 30.0
PARKING_READY_SPEED = 0.5
PARKING_POS_TOL = 0.5
PARKING_YAW_TOL = 5 deg
```

## 3. State별 전이 조건

### 3.1 INIT

Publish state:

```text
/hfsm/current_state = "INIT"
```

전이 조건:

```text
/localization_kinematic_state 수신
→ vs.odom_received = True
→ INIT → BASIC_DRIVING
```

유지 조건:

```text
/localization_kinematic_state 미수신
→ INIT 유지
```

### 3.2 BASIC_DRIVING

Publish state:

```text
/hfsm/current_state = "BASIC_DRIVING"
```

전이 우선순위:

```text
1. 차선 상태 불량
2. 교차로 접근 또는 진입
3. 장애물 감지
4. 주차 구역 접근
5. BASIC_DRIVING 유지
```

#### BASIC_DRIVING → OBS_MANAGER

조건 A: 차선 상태 불량

```text
/lane_status = False
→ BASIC_DRIVING → OBS_MANAGER
```

조건 B: 장애물 대응 필요

```text
/obstacle/exists = True
AND
(
    /obstacle/in_ego_path = True
    OR /ttc < 5.0
    OR /obstacle/front_distance < 15.0
)
→ BASIC_DRIVING → OBS_MANAGER
```

#### BASIC_DRIVING → INTERSECTION

```text
/route_event/in_intersection = True
OR
/route_event/dist_to_intersection < 20.0
→ BASIC_DRIVING → INTERSECTION
```

코드상 `/lane_status = False` 검사가 더 먼저 수행되므로, 차선 상태가 불량하면 교차로보다 장애물 대응이 우선된다.

#### BASIC_DRIVING → LANE_FOLLOWING_PARKING

```text
/parking/target_on_route = True
AND
/route_event/dist_to_parking_zone < 30.0
→ BASIC_DRIVING → LANE_FOLLOWING_PARKING
```

### 3.3 INTERSECTION

Publish state:

```text
/hfsm/current_state = "INTERSECTION"
```

전이 우선순위:

```text
1. 장애물 발생
2. 교차로 출구 통과
3. timeout
4. INTERSECTION 유지
```

#### INTERSECTION → OBS_MANAGER

```text
/obstacle/exists = True
AND
(
    /obstacle/in_ego_path = True
    OR /ttc < 5.0
    OR /obstacle/front_distance < 15.0
)
→ INTERSECTION → OBS_MANAGER
```

#### INTERSECTION → BASIC_DRIVING

```text
/route_event/intersection_exit_passed = True
AND
/lane_status = True
AND
heading_stable = True
→ INTERSECTION → BASIC_DRIVING
```

현재 `heading_stable`은 별도 토픽으로 갱신되지 않고 기본값 `True`로 유지된다.

#### INTERSECTION → BASIC_DRIVING by Timeout

```text
INTERSECTION 진입 후 30초 초과
AND
/route_event/in_intersection = False
→ INTERSECTION → BASIC_DRIVING
```

### 3.4 OBS_MANAGER

Publish state:

```text
/hfsm/current_state = "OBS_MANAGER"
```

`OBS_MANAGER`는 실제 주행 명령을 내리는 state라기보다, 장애물 상황을 분류하는 decision state이다.

전이 우선순위:

```text
1. ESTOP
2. LANE_CHANGE
3. OBS_AVOIDANCE
4. fallback ESTOP
```

#### OBS_MANAGER → ESTOP

```text
/obstacle/exists = True
AND
/obstacle/in_ego_path = True
AND
(
    /ttc < 1.5
    OR /obstacle/front_distance < 5.0
    OR /lane_status = False
    OR (
        /obstacle/avoidance_possible = False
        AND /lane_change/adjacent_lane_safe = False
    )
)
→ OBS_MANAGER → ESTOP
```

#### OBS_MANAGER → LANE_CHANGE

```text
/obstacle/exists = True
AND
/obstacle/in_ego_path = True
AND
/lane_change/adjacent_lane_safe = True
AND
/lane_change/recommended_direction in ["LEFT", "RIGHT"]
AND
(
    1.5 <= /ttc < 4.0
    OR
    (
        /ttc < 0.0
        AND 5.0 < /obstacle/front_distance < 15.0
    )
)
→ OBS_MANAGER → LANE_CHANGE
```

#### OBS_MANAGER → OBS_AVOIDANCE

```text
/obstacle/exists = True
AND
/obstacle/in_ego_path = True
AND
/obstacle/avoidance_possible = True
AND
(
    /obstacle/front_distance < 15.0
    OR /ttc < 5.0
)
→ OBS_MANAGER → OBS_AVOIDANCE
```

#### OBS_MANAGER → ESTOP by Fallback

```text
ESTOP 조건 False
LANE_CHANGE 조건 False
OBS_AVOIDANCE 조건 False
→ OBS_MANAGER → ESTOP
```

### 3.5 ESTOP

Publish state:

```text
/hfsm/current_state = "ESTOP"
```

`behavior_router_node.py`에서는 다음 명령으로 변환된다.

```text
/behavior/planner_mode = "STOP"
/behavior/target_speed = 0.0
/behavior/stop_request = True
```

#### ESTOP → BASIC_DRIVING

```text
is_obstacle_cleared() = True
AND
/lane_status = True
→ ESTOP → BASIC_DRIVING
```

`is_obstacle_cleared()`는 다음 중 하나를 만족하면 `True`가 된다.

```text
/obstacle/exists = False
→ obstacle cleared
```

또는:

```text
(
    /ttc invalid
    OR /ttc > 5.0
)
AND
(
    /obstacle/front_distance invalid
    OR /obstacle/front_distance > 15.0
)
→ obstacle cleared
```

#### ESTOP → LANE_CHANGE

```text
/obstacle/exists = True
AND
/obstacle/in_ego_path = True
AND
/lane_change/adjacent_lane_safe = True
AND
/lane_change/recommended_direction in ["LEFT", "RIGHT"]
AND
ego speed < 0.1
AND
/obstacle/front_distance >= 5.0
→ ESTOP → LANE_CHANGE
```

주의: 현재 코드의 `ESTOP → LANE_CHANGE` 분기에는 `recommended_direction` 검사가 아직 별도로 들어가 있지 않다. `should_enter_lane_change()`와 동일하게 보강하는 것이 좋다.

### 3.6 LANE_CHANGE

Publish state:

```text
/hfsm/current_state = "LANE_CHANGE"
```

`behavior_router_node.py`에서는 다음 명령으로 변환된다.

```text
/behavior/planner_mode = "LANE_CHANGE"
/behavior/target_speed = 7.0
/behavior/lane_change_direction = /lane_change/recommended_direction
/behavior/avoidance_enable = False
/behavior/stop_request = False
```

#### LANE_CHANGE → ESTOP

차선 변경 중 상황이 더 위험해지면 ESTOP으로 전이한다.

```text
/obstacle/exists = True
AND
/obstacle/in_ego_path = True
AND
(
    /ttc < 1.5
    OR /obstacle/front_distance < 5.0
    OR /lane_status = False
    OR (
        /obstacle/avoidance_possible = False
        AND /lane_change/adjacent_lane_safe = False
    )
)
→ LANE_CHANGE → ESTOP
```

#### LANE_CHANGE → BASIC_DRIVING

조건 A: Local Planner가 차선 변경 완료를 보고한 경우

```text
/local_planner/status = "LANE_CHANGE_DONE"
AND
yaw_stable = True
→ LANE_CHANGE → BASIC_DRIVING
```

현재 `yaw_stable`은 별도 토픽으로 갱신되지 않고 기본값 `True`로 유지된다.

조건 B: 장애물이 clear된 경우

```text
is_obstacle_cleared() = True
→ LANE_CHANGE → BASIC_DRIVING
```

### 3.7 OBS_AVOIDANCE

Publish state:

```text
/hfsm/current_state = "OBS_AVOIDANCE"
```

`behavior_router_node.py`에서는 다음 명령으로 변환된다.

```text
/behavior/planner_mode = "AVOIDANCE"
/behavior/target_speed = 5.0
/behavior/avoidance_enable = True
/behavior/stop_request = False
```

#### OBS_AVOIDANCE → ESTOP

```text
/obstacle/exists = True
AND
/obstacle/in_ego_path = True
AND
(
    /ttc < 1.5
    OR /obstacle/front_distance < 5.0
    OR /lane_status = False
    OR (
        /obstacle/avoidance_possible = False
        AND /lane_change/adjacent_lane_safe = False
    )
)
→ OBS_AVOIDANCE → ESTOP
```

#### OBS_AVOIDANCE → BASIC_DRIVING

조건 A: Local Planner가 회피 완료를 보고한 경우

```text
/local_planner/status = "OBS_AVOIDANCE_DONE"
AND
/lane_status = True
→ OBS_AVOIDANCE → BASIC_DRIVING
```

조건 B: 장애물이 clear된 경우

```text
is_obstacle_cleared() = True
→ OBS_AVOIDANCE → BASIC_DRIVING
```

### 3.8 LANE_FOLLOWING_PARKING

Publish state:

```text
/hfsm/current_state = "LANE_FOLLOWING_PARKING"
```

#### LANE_FOLLOWING_PARKING → PARKING_MANEUVER

```text
/parking/at_start_position = True
AND
ego speed < 0.5
AND
heading_stable = True
→ LANE_FOLLOWING_PARKING → PARKING_MANEUVER
```

현재 `heading_stable`은 별도 갱신되지 않으므로, 실질적으로는 다음 조건에 가깝다.

```text
/parking/at_start_position = True
AND
ego speed < 0.5
→ PARKING_MANEUVER
```

#### LANE_FOLLOWING_PARKING → BASIC_DRIVING

```text
/parking/target_on_route = False
→ LANE_FOLLOWING_PARKING → BASIC_DRIVING
```

### 3.9 PARKING_MANEUVER

Publish state:

```text
/hfsm/current_state = "PARKING_MANEUVER"
```

#### PARKING_MANEUVER → FINISHED

```text
/parking/dist_to_goal < 0.5
AND
abs(/parking/yaw_error) < 5 deg
AND
ego speed < 0.1
→ PARKING_MANEUVER → FINISHED
```

또는 parking maneuver 시간이 일정 시간 이상 초과되면 timeout으로 완료 처리한다.

```text
PARKING_MANEUVER 진입 후 60초 초과
→ FINISHED
```

현재 구조에서는 `FINISHED`가 top-level outcome으로 처리되며, 별도의 state로 계속 publish되는 구조는 아니다.

## 4. 전체 전이 흐름 요약

### 4.1 초기화

```text
/localization_kinematic_state 수신
→ INIT
→ BASIC_DRIVING
```

### 4.2 일반 주행 유지

```text
/lane_status = True
/obstacle/exists = False
/route_event/in_intersection = False
/route_event/dist_to_intersection >= 20.0
/parking/target_on_route = False
→ BASIC_DRIVING 유지
```

### 4.3 교차로 진입 및 탈출

```text
/route_event/dist_to_intersection < 20.0
OR /route_event/in_intersection = True
→ BASIC_DRIVING → INTERSECTION
```

```text
/route_event/intersection_exit_passed = True
AND /lane_status = True
→ INTERSECTION → BASIC_DRIVING
```

### 4.4 장애물 대응 진입

```text
/obstacle/exists = True
AND
(
    /obstacle/in_ego_path = True
    OR /ttc < 5.0
    OR /obstacle/front_distance < 15.0
)
→ BASIC_DRIVING 또는 INTERSECTION
→ OBS_MANAGER
```

### 4.5 장애물 대응 분기

```text
OBS_MANAGER
→ ESTOP
→ LANE_CHANGE
→ OBS_AVOIDANCE
```

분기 우선순위:

```text
1순위: ESTOP
2순위: LANE_CHANGE
3순위: OBS_AVOIDANCE
4순위: fallback ESTOP
```

### 4.6 주차 전이

```text
/parking/target_on_route = True
AND /route_event/dist_to_parking_zone < 30.0
→ BASIC_DRIVING → LANE_FOLLOWING_PARKING
```

```text
/parking/at_start_position = True
AND ego speed < 0.5
→ LANE_FOLLOWING_PARKING → PARKING_MANEUVER
```

```text
/parking/dist_to_goal < 0.5
AND abs(/parking/yaw_error) < 5 deg
AND ego speed < 0.1
→ PARKING_MANEUVER → FINISHED
```

## 5. `/hfsm/current_state`와 Behavior Router 관계

`hfsm_node.py`는 최종적으로 현재 state를 `/hfsm/current_state`로 publish한다.
이 값을 `behavior_router_node.py`가 받아 실제 planner 명령으로 변환한다.

| `/hfsm/current_state` | `/behavior/planner_mode` | `/behavior/target_speed` | `/behavior/stop_request` | 의미 |
| --- | --- | ---: | --- | --- |
| `INIT` | `STOP` | 0.0 | True | 초기 정지 |
| `BASIC_DRIVING` | `LANE_FOLLOW` | 10.0 | False | 일반 주행 |
| `INTERSECTION` | `LANE_FOLLOW` | 5.0 | False | 교차로 감속 주행 |
| `LANE_CHANGE` | `LANE_CHANGE` | 7.0 | False | 차선 변경 |
| `OBS_AVOIDANCE` | `AVOIDANCE` | 5.0 | False | local avoidance |
| `ESTOP` | `STOP` | 0.0 | True | 긴급정지 |

주의할 점은 `hfsm_node.py`가 `OBS_MANAGER`를 publish할 수 있다는 점이다.
현재 `behavior_router_node.py`는 `OBS_MANAGER`를 명시적으로 처리하지 않으므로 unknown state로 간주되어 기본 STOP 명령이 나간다.

또한 주차 관련 state인 다음 두 state도 behavior router에 별도 처리가 없으면 STOP으로 처리될 수 있다.

```text
LANE_FOLLOWING_PARKING
PARKING_MANEUVER
```

## 6. 현재 구조의 핵심 주의점

### 6.1 `recommended_direction` 검증

현재 `should_enter_lane_change()`에는 다음 검사가 반영되어 있다.

```python
if vs.recommended_direction not in ["LEFT", "RIGHT"]:
    return False
```

따라서 인접 차선이 안전하더라도 방향이 `NONE`이면 `LANE_CHANGE`로 전이하지 않는다.

단, `ESTOP → LANE_CHANGE` 직접 전이 조건에도 같은 방향 검증을 추가하는 것이 더 일관적이다.

### 6.2 `OBS_MANAGER`에 대한 Behavior Router 처리

`OBS_MANAGER`는 decision state이지만 `/hfsm/current_state`로 publish된다.
현재 router에서는 unknown state fallback으로 STOP이 나간다.
보수적인 동작으로는 가능하지만, 로그 경고를 줄이고 의도를 명확히 하려면 다음 mapping을 추가하는 것이 좋다.

```text
OBS_MANAGER → STOP
```

### 6.3 Parking State에 대한 Behavior Router 처리

`LANE_FOLLOWING_PARKING`, `PARKING_MANEUVER` state가 존재하지만, behavior router에서 해당 state를 별도로 처리하지 않으면 실제 주차 behavior로 연결되지 않는다.

필요한 처리는 다음과 같다.

```text
LANE_FOLLOWING_PARKING → LANE_FOLLOW + 저속
PARKING_MANEUVER → PARKING planner mode
```

### 6.4 `heading_stable`, `yaw_stable`이 사실상 항상 True

현재 코드에서 `heading_stable`, `yaw_stable`은 초기값이 `True`이고, 별도 토픽으로 갱신되지 않는다.

따라서 다음 판단은 실제 안정성 검사가 아니라 항상 통과하는 조건에 가깝다.

```text
교차로 탈출 후 heading 안정성
차선 변경 완료 후 yaw 안정성
주차 시작 전 heading 안정성
```

정확한 판단을 위해서는 yaw rate, heading error, lateral error 등을 기반으로 갱신하는 로직이 필요하다.

## 7. 결론

`hfsm_node.py`의 핵심 역할은 다음과 같다.

```text
센서 및 evaluator 토픽 수신
→ VehicleState 갱신
→ 현재 state에서 조건 검사
→ 다음 state 결정
→ /hfsm/current_state publish
→ behavior_router가 planner command로 변환
```

장애물 대응의 핵심 분기 구조는 다음과 같다.

```text
장애물 감지
→ OBS_MANAGER
→ 1순위 ESTOP
→ 2순위 LANE_CHANGE
→ 3순위 OBS_AVOIDANCE
→ fallback ESTOP
```

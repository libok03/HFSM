# HFSM Architecture & Logic Reference

이 문서는 자율주행 의사결정 시스템인 `hfsm_decision_maker` 패키지와 그 주변 시스템(`behavior_router`, `local_planner` 등)이 주고받는 토픽들과 상태 천이 로직을 종합적으로 정리한 가이드입니다.

---

## 1. 시스템 아키텍처 개요

시스템은 크게 **[인지/평가] ➔ [판단(HFSM)] ➔ [명령 라우팅] ➔ [로컬 제어]** 의 4단계 구조로 이루어져 있습니다.

1. **Evaluator Nodes (평가기):** 센서 및 맵 데이터를 분석하여 위험도와 주행 이벤트를 평가합니다. (`lanelet2_event_evaluator_node.py` 및 외부 장애물 인지 노드)
2. **HFSM Node (판단기):** 평가된 데이터를 바탕으로 현재 차량이 취해야 할 최상위 상태(State)를 결정합니다. (`hfsm_node.py`)
3. **Behavior Router (명령어 라우터):** HFSM의 State를 해석하여 하위 제어기들이 알아들을 수 있는 행동 지시로 번역합니다. (`behavior_router_node.py`)
4. **Local Planner (로컬 제어):** 라우터의 지시를 받아 실제 차량이 따라가야 할 궤적과 속도를 그려냅니다. (`local_planner_node.py`)

---

## 2. 입출력 토픽 (Topics) 정리

### 📥 HFSM이 외부로부터 받는 토픽 (Inputs)
| 토픽 이름 | 메시지 타입 | 용도 / 보내는 곳 |
|---|---|---|
| `/localization_kinematic_state` | `nav_msgs/Odometry` | 차량의 현재 위치, 속도 파악 |
| `/ttc` | `std_msgs/Float32` | 충돌 예상 시간 (외부 장애물 판단 노드) |
| `/lane_status` | `std_msgs/Bool` | 차선 인식 및 맵 매칭 정상 여부 |
| `/obstacle/exists`, `/obstacle/front_distance`, `/obstacle/in_ego_path`, `/obstacle/avoidance_possible` | `Bool` / `Float32` | 장애물 존재 여부 및 회피/제동 판단 데이터 |
| `/lane_change/adjacent_lane_safe`, `/lane_change/recommended_direction` | `Bool` / `String` | 인접 차선 안전 여부 및 방향 (`lanelet2_event_evaluator`) |
| `/route_event/dist_to_intersection`, `/route_event/in_intersection`, `/route_event/intersection_exit_passed` | `Float32` / `Bool` | 교차로 진입 및 통과 여부 (`lanelet2_event_evaluator`) |
| `/route_event/dist_to_parking_zone` | `Float32` | 주차장까지의 거리 |
| `/local_planner/status` | `std_msgs/String` | 로컬 플래너의 임무 완료 피드백 (예: `"LANE_CHANGE_DONE"`) |

### 📤 HFSM이 하위 시스템으로 내리는 토픽 (Outputs)
HFSM 노드는 오직 **단 하나의 상태 출력**만 내보냅니다.
* **`/hfsm/current_state`** (`std_msgs/String`) : 현재 결정된 최상위 행동 상태.

### 🔄 Behavior Router가 로컬 플래너로 전달하는 명령 토픽
HFSM의 State를 받아 아래의 세부 명령으로 쪼개서 `local_planner`에 전달합니다.
* **`/behavior/planner_mode`** (`String`): `"STOP"`, `"LANE_FOLLOW"`, `"LANE_CHANGE"`, `"AVOIDANCE"`
* **`/behavior/target_speed`** (`Float32`): 각 모드에 알맞은 목표 속도 (예: 10.0, 5.0, 0.0)
* **`/behavior/lane_change_direction`** (`String`): `"LEFT"`, `"RIGHT"`, `"NONE"`
* **`/behavior/avoidance_enable`** (`Bool`): 회피 궤적 생성 활성화 여부
* **`/behavior/stop_request`** (`Bool`): 비상 정지 및 정차 요구

---

## 3. HFSM 상태(State) 천이 로직 상세

HFSM은 크게 `INIT`, `DRIVING`, `OBSTACLE`, `PARKING` 4가지의 거대한 상위 State를 가지며, 그 내부에 세부 State들이 존재합니다.

### 🟢 INIT (초기화)
* **목적:** 센서 데이터(Odometry)가 정상적으로 수신될 때까지 대기합니다.
* **로직:** Odom이 수신되면 `DRIVING` 상태로 넘어갑니다.

### 🔵 DRIVING (주행 모드)
차량이 정상적으로 경로를 따라가는 상태입니다.

1. **`BASIC_DRIVING`**
   * **기본:** 목표 속도(`10.0m/s`), 차선 유지(`LANE_FOLLOW`)
   * **천이 조건:**
     * 전방에 교차로가 가까워지면 ➔ `INTERSECTION`
     * 전방에 주차장이 가까워지면 ➔ `PARKING`
     * 장애물이 나타나거나 TTC가 위험해지면 ➔ `OBSTACLE` (우선순위 최고)
2. **`INTERSECTION`**
   * **기본:** 속도를 줄임(`5.0m/s`), 차선 유지(`LANE_FOLLOW`)
   * **천이 조건:**
     * 교차로를 완전히 빠져나가면(`intersection_exit_passed == True`) ➔ `BASIC_DRIVING` 복귀
     * 30초 이상 교차로에 머무르면(Timeout) ➔ `BASIC_DRIVING` 강제 복귀

### 🔴 OBSTACLE (장애물 대응 모드)
장애물이 발견되어 회피하거나 정지해야 하는 최우선 순위 상태입니다. 내부에 `OBS_MANAGER`라는 중재자가 상황을 판단하여 아래 3가지 중 하나로 분기시킵니다.

1. **`ESTOP` (긴급 제동)**
   * **조건:** 장애물이 내 경로상에 있고 TTC가 매우 짧거나(`TTC_ESTOP` 미만), 회피가 불가능할 때.
   * **행동:** 즉시 속도 `0.0m/s`, 정지 요청(`stop_request=True`)
2. **`LANE_CHANGE` (차선 변경)**
   * **조건:** TTC가 회피할 수 있는 여유 범위 내에 있고 옆 차선이 안전할 때.
   * **행동:** `lane_change_direction`을 설정하고 로컬 플래너에게 다항식 스플라인 회피 궤적(`LANE_CHANGE`)을 요청. 플래너가 완료(`LANE_CHANGE_DONE`) 상태를 주면 `BASIC_DRIVING`으로 복귀.
3. **`OBS_AVOIDANCE` (단순 측면 회피)**
   * **조건:** 차선 변경까지는 필요 없고 약간 비켜서 갈 수 있을 때.
   * **행동:** `AVOIDANCE` 모드를 켜서 로컬 플래너가 1.5m 정도 측면으로 비켜가는 궤적을 그리도록 유도.

### 🟣 PARKING (주차 모드)
1. **`LANE_FOLLOWING_PARKING`**
   * 주차장에 진입하여 주차 시작 지점까지 서행(`3.0m/s`)으로 이동.
2. **`PARKING_MANEUVER`**
   * 실제 주차 지점에 도착하여 정밀하게 주차 각도와 거리를 맞추고 최종 정지.

---

## 4. 확장 가이드 (차후 업데이트 필요 사항)

* **신호등(Traffic Light) 연동:**
  현재 `INTERSECTION` 상태에서는 단순히 속도만 줄이도록 되어 있습니다. 비전 인식 노드와 연동하여 빨간불일 경우 `INTERSECTION_STOP` 이라는 상태를 추가하고, 정지선까지 남은 거리를 바탕으로 로컬 플래너가 차를 세우도록 확장해야 합니다.
* **주차 공간 판단 (Parking Zone Parsing):**
  현재 `lanelet2_event_evaluator`에서 주차장 구역을 찾는 로직이 빠져 있습니다. OSM 맵에서 주차장 속성을 추출하는 코드가 추가되어야 합니다.

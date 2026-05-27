# Day4_5_Scenario_1.slx — Signal & Block Mapping

이 문서는 `Models/Day4_5/Day4_5_Scenario_1.slx`의 현재 신호 라우팅과 블록 구성을 정리한 것이다. From-scratch 재구현 시 새 `planner_top` 모듈의 입력/출력 인터페이스를 설계하는 근거로 사용한다.

원본 모델 inspection은 MATLAB의 `find_system` + Stateflow API + `.slx` (ZIP) 내부 `system_root.xml` 직접 grep 으로 수행했다.

## 1. CarMaker quantity 매핑

### 입력 (Read CM Dict1)
| Signal | CarMaker quantity | 의미 |
| ------ | ----------------- | ---- |
| ego_x   | `Car.Fr1.tx` | ego 차량 글로벌 X (Frame 1 = front-1 axle? Or vehicle ref) |
| ego_y   | `Car.Fr1.ty` | ego 차량 글로벌 Y |
| ego_yaw | `Car.Fr1.rz` | ego 차량 yaw (rad) |
| ego_v   | `Car.vx`     | ego 종방향 속도 (m/s) |

### 출력 (Write CM Dict ×3)
| Signal | CarMaker quantity | 의미 |
| ------ | ----------------- | ---- |
| steer_fl    | `Car.CFL.rz_ext`     | 전륜 좌측 스티어링 외부 명령 (rad) |
| steer_fr    | `Car.CFR.rz_ext`     | 전륜 우측 스티어링 외부 명령 (rad) |
| desired_ax  | `AccelCtrl.DesiredAx` | 종방향 가속도 명령 (m/s²) |

## 2. 모델 내부 신호 (Goto/From 태그)

| Tag | Source (Goto) | Sink (From) | 의미 |
| --- | ------------- | ----------- | ---- |
| `Start_Point`   | `Goto4` ← Mux(Constant2=0, Constant3=-20) | `From7` → MATLAB Function/start_point | (0, -20) |
| `Finish_Point`  | `Goto1` ← Mux(Constant4=80, Constant5=-3) | `From6` → MATLAB Function/finish_point | (80, -3) |
| `Map_Boundary`  | `Goto6` ← Mux(Constant9=-100, 11=100, 13=100, 14=-100, ...) | `From` → MATLAB Function1/map_boundary | x∈[-100,100], y∈[-100,100] (현재) |
| `Traffic_size`  | `Goto2` ← Mux(Constant10=2.48, Constant1=11.5) | `From2/5` → MATLAB Function1·2/traffic_size | Volvo 트럭 [W, L] |
| `Traffic_Info`  | `Goto9` ← Subsystem/Traffic_Info | `From3/4` → MATLAB Function1·2/traffic_info | 21×1 (T01~T07) |
| `Map`           | `Goto` ← MATLAB Function1/mapMatrix | `From1` → MATLAB Function2/map | 100×100 baseline map |
| `y`             | `Goto3` ← MATLAB Function2/y | `From8` → MATLAB Function/occ_map | 100×100 occupancy map |
| `goal_yaw`      | Constant `goal_yaw = pi/2` | 직결 → MATLAB Function/goal_yaw | π/2 (90°), T00 yaw와 동일 |

**핵심 관찰**: `Finish_Point`와 `goal_yaw`는 **Constant 블록으로 하드코딩**되어 있고, **T00의 동적 좌표는 모델 어디에서도 입력되지 않는다.** 사용자 의도에 따라 T00 좌표/방향을 동적으로 받으려면 `Read CM Dict`에 `Traffic.0.tx, Traffic.0.ty, Traffic.0.Yaw` (또는 그에 상응하는 CarMaker quantity)를 추가해야 한다.

## 3. 블록 구성

### 최상위 (Day4_5_Scenario_1)
54개 블록. 주요 항목:
- **Read CM Dict1** — CarMaker → Simulink (ego 상태 4개 quantity)
- **Write CM Dict ×3** — Simulink → CarMaker (steer FL, steer FR, desired Ax)
- **MATLAB Function**  (= 메인 `Parking` 함수, Stateflow EMChart)
- **MATLAB Function1** (= `generate_map_`, Stateflow EMChart)
- **MATLAB Function2** (= `add_obstacle_`, Stateflow EMChart)
- **Subsystem** (Traffic Object 1~7 → traffic_info 21×1 형성)
- Constant 14개, Mux 8개, From/Goto 다수, ToWorkspace 5개

### MATLAB Function / `Parking` (chart_18)
```
Inports : ego_x, ego_y, ego_yaw, ego_v, start_point, finish_point, goal_yaw, occ_map
Outports: desired_ax, steer_fl, steer_fr, path_x_dbg, path_y_dbg, path_len_dbg
```
- 내부는 **grid A\* + path smoothing** (`grid_astar_plan`, `safe_smooth_path`, `path_is_collision_free`)이지 Hybrid A\*가 아님. (외부 `chart18_current.m` 백업과는 다른 버전이다.)
- 시작 좌표 clamp: `plan_sx ∈ [1, 99]`, `plan_sy ∈ [-99, -1]` → 주차장 영역 x∈[0,100], y∈[-100,0] 안으로 강제.
- 첫 경로점을 `ego` 위치로 강제 후 충돌 재검증, 실패 시 stay-put fallback.
- `persistent`: `path_x, path_y, path_len, plan_ready, tick, last_goal_x/y, hold_count, prev_steer`.
- 427 라인 내외, codegen 호환 (`%#codegen`).

### MATLAB Function1 / `generate_map_`
```
Inports : map_boundary, traffic_info, traffic_size
Outports: mapMatrix
```
- 현재 **100×100 셀**, `res=1.0`, `x_min=0.0`, `y_max=0.0` → **물리 영역 x∈[0,100], y∈[-100,0]** (100m × 100m).
- `map_boundary`를 받아 bounding box 추출 후, 그 외부 셀을 1로 마킹 (off-road 경계).
- **사용자 항목 1의 "기존 100m×100m, 1m/cell, 100×100 cells" 가 chart 내부 코드와 정확히 일치.**

### MATLAB Function2 / `add_obstacle_`
```
Inports : map, traffic_info, traffic_size
Outports: y
```
- **100×100 셀**, 동일한 영역/해상도.
- 주석에 명시: `% CarMaker traffic position is assumed to be the rear bumper center.`
- 실제 코드는 `(rear_x, rear_y) → (px, py) = (rear_x + L/2·cos(yaw), rear_y + L/2·sin(yaw))` 로 **차량 중심 변환 후 좌우대칭 footprint** (`|local_x|≤half_l, |local_y|≤half_w`).
- inflation = `ego_w*0.5 + safety_margin = 1.9*0.5 + 0.3 = 1.25 m`.
- T01~T07 7대만 처리 (`num_traffic = 7`).
- **사용자 항목 2의 의도와 등가** (수학적으로 동일). 새 구현은 사용자 의사코드대로 "변환 없이 비대칭 footprint" (`0 ≤ local_x ≤ veh_l`)로 작성하면 더 직관적.

### Subsystem (Traffic_Info 형성)
- 내부 블록: Traffic Object 1~7 (7개 Reference) + BusSelector 7개 + Mux + Constant 7개 + Goto/From + Terminator.
- 각 Traffic Object i 의 bus에서 (x, y, yaw) 추출 → Mux → 21×1 → Outport `Traffic_Info`.
- **T00용 Reference는 없음.** → 기존 외부 코드 `chart45_with_t00.m`의 주석 ("Subsystem only routes T01..T07") 그대로.

## 4. New `planner_top` 인터페이스 설계 (재구현 시)

설계 요구사항 1~5와 위 매핑을 종합한 권장 인터페이스:

```matlab
function [steer_fl, steer_fr, desired_ax, ...
          path_x_dbg, path_y_dbg, path_len_dbg, occ_dbg] = planner_top( ...
              ego_x, ego_y, ego_yaw, ego_v, ...
              t00_x, t00_y, t00_yaw, ...      % 신규: T00 goal pose (장애물 아님)
              traffic_info, traffic_size)
% traffic_info : 21x1  (T01~T07 rear bumper x,y,yaw)
% traffic_size : 1x2   ([W L] of Volvo truck)
```

내부 호출 순서:
1. `occ_grid = add_obstacle(traffic_info, traffic_size)` — 200×200 (0.5m/cell, 100m×100m, 항목 1)
2. `[path_x, path_y, path_yaw, path_len] = hybrid_astar_plan(ego_pose, t00_pose, occ_grid)`
3. `steer_cmd = pure_pursuit(ego_pose, ego_v, path_x, path_y, path_len)`
4. `desired_ax = pd_speed(v_des_profile(ego_pose, t00_pose), ego_v)`
5. `steer_fl = steer_cmd; steer_fr = steer_cmd;` (또는 Ackermann 보정)

**Constant block 변경 필요 항목 (.slx 통합 단계)**:
- Constant9/11/13/14 (map_boundary 4 corners): -100/100 → -50/50 (100m×100m로 축소) **또는** baseline map 생성 함수를 새로 짜고 map_boundary 입력을 제거.
- Constant4 (=80), Constant5 (=-3), `goal_yaw` (=π/2): T00 goal pose 동적 입력으로 교체.
- Constant10 (=2.48), Constant1 (=11.5): 그대로 유지 (Volvo 트럭 사이즈).

**Read CM Dict1 변경 필요 사항**:
- 현재 quantity: `Car.Fr1.tx, Car.Fr1.ty, Car.Fr1.rz, Car.vx` (ego 4개).
- 추가 필요: T00 글로벌 좌표·yaw. CarMaker quantity 후보:
  - `Traffic.T00.tx, Traffic.T00.ty, Traffic.T00.Yaw` (CarMaker traffic object quantity 컨벤션)
  - 정확한 이름은 CarMaker GUI 또는 `cmrepl` 등으로 확인 (현재 도구로는 검증 불가).

**미해결**: T00 goal pose 파생 방식 (옵션 A: T00 rear bumper 그대로 / 옵션 B: T00 옆 별도 좌표) — Hybrid A\* 검증 단계에서 결정.

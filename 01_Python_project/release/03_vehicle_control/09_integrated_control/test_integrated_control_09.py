"""Integrated control regression — behavioral spec (requirements level).

알고리즘 형태 (각 sub-controller 내부 / decision latch 구현) 는 자유.
인터페이스만 맞으면 OK — 두 가지 spec 으로 합격 판정:
  1. LongitudinalDecision latch — 침범 후 mode 가 timegap 으로 latch 되어 유지.
  2. closed-loop 통합 — ego 가 lane2 추종 유지 + target 침범 후 timegap follow + 충돌 X.

Sub-controller (PP / Stanley / PIDFF) 의 단위 수식 검증은 06/07/08 에 위임 —
여기선 adapter 통합 작동만 closed-loop 통해서 간접 검증.
"""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "05_frame_transform"))
from control_pipeline import (
    ControlPipeline,
    EgoState,
    LongitudinalDecision,
    Road,
    TargetState,
)
from frame_transform import Global2Local, PolynomialFitting, PolynomialValue
from lateral_controller import LateralController, PurePursuit
from longitudinal_controller import LongitudinalController
from vehicle_combined import VehicleCombined

DT = 0.1
DEGREE = 3
NUM_POINT = 10
X_LOCAL = np.arange(0.0, 13.0, 0.5)
SAMPLE_XS = np.arange(NUM_POINT) * 1.2


def _target_state(t: float, road: Road, vx_t: float = 8.0,
                  X0: float = 50.0, t_invasion: float = 10.0,
                  T_invasion: float = 5.0) -> TargetState:
    X = X0 + vx_t * t
    if t < t_invasion:
        Y = road.lane1_center(X)
    elif t < t_invasion + T_invasion:
        phase = (t - t_invasion) / T_invasion
        blend = 0.5 * (1.0 - np.cos(np.pi * phase))
        Y = (1.0 - blend) * road.lane1_center(X) + blend * road.lane2_center(X)
    else:
        Y = road.lane2_center(X)
    return TargetState(X=float(X), Y=float(Y), vx=vx_t)


def test_long_decision_latches_timegap_after_invasion():
    """침범 감지 후 mode 는 timegap 으로 latch — 이후 target 이 원래 차로로 돌아가도 유지."""
    road = Road(R=200.0)
    dec = LongitudinalDecision(road)
    ego = EgoState(X=0.0, Y=-1.75, Yaw=0.0, vx=10.0)
    t1 = TargetState(X=50.0, Y=float(road.lane1_center(50.0)), vx=8.0)
    assert dec.long_mode(0.0, ego, t1) == "speed"
    t2 = TargetState(X=70.0, Y=float(road.lane2_center(70.0)), vx=8.0)
    assert dec.long_mode(1.0, ego, t2) == "timegap"
    t3 = TargetState(X=80.0, Y=float(road.lane1_center(80.0)), vx=8.0)
    assert dec.long_mode(2.0, ego, t3) == "timegap"  # latch 유지


def test_closed_loop_integration_within_spec():
    """sim 45 s 통합 폐루프:
       - lane2 추종 (마지막 5 s tail 평균 |lat err| < 0.4 m)
       - target invasion 후 timegap latch
       - 충돌 없음 (min gap > 2 m)
       - 감속 발생 (vx 가 v_des 10 보다 작아짐), 단 target_vx 8 까지 과감속 X
    """
    road = Road(R=200.0)
    pp = PurePursuit(L=4.0, lookahead_time=1.0)
    lat_ctrl = LateralController(pp, lookahead_x_fn=lambda vx: vx * pp.lookahead_time)
    long_ctrl = LongitudinalController(dt=DT, kp_v=0.5, kd_v=0.0, kp_g=2.0, kd_g=3.0, tau_gap=1.5)
    decision = LongitudinalDecision(road)
    pipe = ControlPipeline(
        g2l=Global2Local(NUM_POINT),
        fitter=PolynomialFitting(DEGREE, NUM_POINT),
        ev=PolynomialValue(DEGREE, int(X_LOCAL.size)),
        lat_ctrl=lat_ctrl,
        long_ctrl=long_ctrl,
        decision=decision,
        ref_y_fn=road.lane2_center,
        sample_xs=SAMPLE_XS,
        x_local=X_LOCAL,
        v_des=10.0,
    )
    plant = VehicleCombined(dt=DT, vx0=10.0, X0=0.0, Y0=float(road.lane2_center(0.0)), Yaw0=0.0)
    sim_time = 45.0
    steps = int(sim_time / DT)
    modes, Y_ego, X_ego, X_tgt, vx_ego = [], [], [], [], []
    for i in range(steps):
        t = i * DT
        ego = EgoState(X=plant.X, Y=plant.Y, Yaw=plant.Yaw, vx=plant.vx)
        tgt = _target_state(t, road)
        out = pipe.step(t, ego, tgt)
        plant.step(out.delta, out.ax)
        modes.append(out.long_mode)
        Y_ego.append(plant.Y); X_ego.append(plant.X); X_tgt.append(tgt.X)
        vx_ego.append(plant.vx)

    # 침범 latch
    assert decision.invaded, "target invasion 후 timegap mode latch 안 됨"
    assert "timegap" in modes

    # tail lane2 추종 (마지막 5 s)
    n_last = int(5.0 / DT)
    Y_last = np.array(Y_ego[-n_last:])
    X_last = np.array(X_ego[-n_last:])
    lat_err_tail = np.abs(Y_last - road.lane2_center(X_last))
    tail_mae = float(np.mean(lat_err_tail))
    assert tail_mae < 0.4, f"lane2 추종 tail MAE {tail_mae:.4f} m 임계값 초과"

    # 감속 발생, 단 과감속 X
    vx_last_mean = float(np.mean(vx_ego[-n_last:]))
    assert vx_last_mean < 9.5, f"감속 부족: vx tail mean {vx_last_mean:.2f} (v_des 10)"
    assert vx_last_mean > 7.0, f"과감속: vx tail mean {vx_last_mean:.2f} (target vx 8)"

    # 충돌 없음
    gaps = np.array(X_tgt) - np.array(X_ego)
    assert gaps.min() > 2.0, f"min gap {gaps.min():.2f} m 임계값 미달 (충돌 위험)"

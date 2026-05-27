"""LatPIDFF regression — behavioral spec (requirements level).

알고리즘 형태 (정통 PID + FF / 다른 처리) 는 자유.
인터페이스만 맞으면 OK — pipeline 폐루프 lateral 추종의 평균·최대 오차로 합격 판정.
FF 효과는 PID-only 대비 상대 비교로 검증.
"""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "05_frame_transform"))
from frame_transform import Global2Local, PolynomialFitting, PolynomialValue
from lat_pid_ff import LatPIDFF
from lateral_pipeline_pid_ff import LateralPipeline
from vehicle_lat_pid import VehicleLat

DT = 0.1
DEGREE = 3
NUM_POINT = 5
X_LOCAL = np.arange(0.0, 10.0, 0.5)
SAMPLE_XS = np.arange(NUM_POINT) * 1.0


def _ref_y(x):
    L_straight = 40.0
    if np.isscalar(x):
        return 0.0 if x < L_straight else 2.0 * (np.cos((x - L_straight) / 14.0) - 1.0)
    return np.where(x < L_straight, 0.0, 2.0 * (np.cos((x - L_straight) / 14.0) - 1.0))


def _make_pipeline(controller: LatPIDFF) -> LateralPipeline:
    return LateralPipeline(
        Global2Local(NUM_POINT),
        PolynomialFitting(DEGREE, NUM_POINT),
        PolynomialValue(DEGREE, np.size(X_LOCAL)),
        controller, SAMPLE_XS, X_LOCAL,
    )


def _run_closed_loop(controller: LatPIDFF, vx: float, sim_time: float) -> list[float]:
    pipe = _make_pipeline(controller)
    plant = VehicleLat(dt=DT, vx=vx, Y0=1.0)
    errs = []
    lookahead_x = vx * controller.lookahead_time
    for _ in range(int(sim_time / DT)):
        out = pipe.step(plant.X, plant.Y, plant.Yaw, vx, _ref_y, lookahead_x=lookahead_x)
        plant.step(out.delta, vx)
        errs.append(abs(plant.Y - float(_ref_y(plant.X))))
    return errs


def test_low_speed_tracking_within_spec():
    """vx=3, PID-only (kff=0), pipeline 30 s: tail 평균 |lateral err| < 0.3, peak < 1.2.

    저속 + 작은 곡률이라 FF 없이도 추종 가능.
    """
    controller = LatPIDFF(kp=0.2, kd=0.1, ki=0.0, kff=0.0, dt=DT)
    errs = np.array(_run_closed_loop(controller, vx=3.0, sim_time=30.0))
    tail_mae = float(np.mean(errs[len(errs) // 2:]))
    peak = float(np.max(errs))
    assert tail_mae < 0.3, f"tail lat MAE {tail_mae:.4f} m 임계값 초과"
    assert peak < 1.2, f"peak |lat error| {peak:.4f} m 임계값 초과"


def test_high_speed_ff_improves_tracking():
    """vx=10: kff>0 가 kff=0 대비 평균 |lateral err| 15% 이상 감소.

    고속·곡률 구간에서 FF 가 PID-only 대비 의미있는 개선이어야 — FF 무효 구현 차단.
    """
    pid_only = LatPIDFF(kp=0.2, kd=0.1, ki=0.0, kff=0.0, dt=DT)
    pid_ff = LatPIDFF(kp=0.2, kd=0.1, ki=0.0, kff=0.1, dt=DT)
    errs_no_ff = np.array(_run_closed_loop(pid_only, vx=10.0, sim_time=15.0))
    errs_ff = np.array(_run_closed_loop(pid_ff, vx=10.0, sim_time=15.0))
    mean_no_ff = float(np.mean(errs_no_ff))
    mean_ff = float(np.mean(errs_ff))
    assert mean_ff < 0.85 * mean_no_ff, (
        f"FF 효과 부족 — no_ff MAE={mean_no_ff:.4f}, ff MAE={mean_ff:.4f}, "
        f"감소비 {mean_ff / mean_no_ff:.3f} (임계값 < 0.85)"
    )

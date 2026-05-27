"""TimeGap PID regression — behavioral spec (requirements level).

알고리즘 형태 (정통 PID / 다른 gap 정의) 는 자유.
인터페이스만 맞으면 OK — 1초 시간 간격 추종의 충돌 회피 + 정상상태 정확도로 합격 판정.
"""
import numpy as np
from time_gap_pid import TimeGapPID
from vehicle_long_tg import VehicleLong

DT = 0.1
TIME_GAP = 1.0
KP, KD, KI = 0.3, 1.0, 0.0


def _maneuvering_a_t(t: float) -> float:
    if t < 20:
        return 0.0
    elif t < 40:
        return 1.5
    elif t < 60:
        return -1.5
    else:
        return 0.0


def test_stationary_target_time_gap_within_spec():
    """정속 target 50 s: tail 평균 |time_gap - 1.0| < 0.1 s, peak < 0.3 s.

    초기 조건 = ACC 인계 시점 (gap0 = 정상 time-gap 거리).
    """
    sim_time = 50.0
    target = VehicleLong(dt=DT, Ca=0.0, x0=10.0, vx0=10.0)
    ego = VehicleLong(dt=DT, Ca=0.5, x0=0.0, vx0=10.0)
    pid = TimeGapPID(kp=KP, kd=KD, ki=KI, dt=DT, time_gap=TIME_GAP)
    steps = int(sim_time / DT)
    tg_errs = np.zeros(steps)
    for k in range(steps):
        u = pid.step(target_x=target.x, ego_x=ego.x, ego_vx=ego.vx)
        ego.step(u)
        target.step(0.0)
        tg_errs[k] = abs((target.x - ego.x) / ego.vx - TIME_GAP)
    tail_mae = float(np.mean(tg_errs[steps // 2:]))
    peak = float(np.max(tg_errs))
    assert tail_mae < 0.1, f"tail time_gap MAE {tail_mae:.4f} s 임계값 초과"
    assert peak < 0.3, f"peak |time_gap err| {peak:.4f} s 임계값 초과"


def test_maneuvering_target_safe_and_reconverges():
    """가/감속 target 80 s: 충돌 없음 + 정속 복귀 후 1초 time_gap 재수렴.

    동일 게인. min gap > 0 (충돌 X), 마지막 |time_gap - 1.0| < 0.5 s.
    """
    sim_time = 80.0
    target = VehicleLong(dt=DT, Ca=0.0, x0=10.0, vx0=10.0)
    ego = VehicleLong(dt=DT, Ca=0.5, x0=0.0, vx0=10.0)
    pid = TimeGapPID(kp=KP, kd=KD, ki=KI, dt=DT, time_gap=TIME_GAP)
    steps = int(sim_time / DT)
    min_gap = float("inf")
    for i in range(steps):
        t = i * DT
        u = pid.step(target_x=target.x, ego_x=ego.x, ego_vx=ego.vx)
        ego.step(u)
        target.step(_maneuvering_a_t(t))
        gap = target.x - ego.x
        if gap < min_gap:
            min_gap = gap
    final_time_gap = (target.x - ego.x) / ego.vx
    assert min_gap > 0.0, f"충돌 발생; min gap = {min_gap:.2f} m"
    assert abs(final_time_gap - TIME_GAP) < 0.5, (
        f"final time_gap {final_time_gap:.2f} s 임계값 초과"
    )

"""ConstantSpace PID regression — behavioral spec (requirements level).

알고리즘 형태 (정통 PID / 다른 처리) 는 자유.
인터페이스만 맞으면 OK — 일정 간격 추종의 충돌 회피 + gap boundedness 로 합격 판정.
"""
import numpy as np
from constant_space_pid import ConstantSpacePID
from vehicle_long_space import VehicleLong

DT = 0.1
SIM_TIME = 80.0
TARGET_SPACE = 20.0
KP, KD, KI = 0.08, 0.4, 0.0  # PD-only — drag 외란으로 정상상태 offset 존재 (학습 포인트)


def test_following_safe_and_bounded():
    """PD-only 종방향 추종 80 s:
       - 충돌 없음 (min gap > 5 m)
       - gap 발산 없음 (peak |gap - target_space| < 12 m)
       - 정상상태 gap 이 target_space 근방 (tail MAE < 3 m — drag 로 일정 offset 허용)

    PD-only 의 잔류 offset 자체가 학습 포인트. 충돌·발산만 막으면 합격.
    """
    target = VehicleLong(dt=DT, m=500.0, Ca=0.0, x0=30.0, vx0=10.0)
    ego = VehicleLong(dt=DT, m=500.0, Ca=0.5, x0=0.0, vx0=10.0)
    ctrl = ConstantSpacePID(kp=KP, kd=KD, ki=KI, dt=DT, target_space=TARGET_SPACE)
    steps = int(SIM_TIME / DT)
    gaps = np.zeros(steps)
    for k in range(steps):
        u = ctrl.step(target_x=target.x, ego_x=ego.x)
        ego.step(u)
        target.step(0.0)
        gaps[k] = target.x - ego.x

    gap_err = np.abs(gaps - TARGET_SPACE)
    min_gap = float(np.min(gaps))
    tail_mae = float(np.mean(gap_err[steps // 2:]))
    peak = float(np.max(gap_err))
    assert min_gap > 5.0, f"min gap {min_gap:.2f} m 임계값 미달 (충돌 위험)"
    assert tail_mae < 3.0, f"tail gap MAE {tail_mae:.2f} m 임계값 초과"
    assert peak < 12.0, f"peak |gap - target_space| {peak:.2f} m 임계값 초과"

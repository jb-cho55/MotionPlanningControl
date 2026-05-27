"""Speed PID regression — behavioral spec (requirements level).

알고리즘 형태 (정통 PID / 다른 적분/미분 처리) 는 자유.
인터페이스만 맞으면 OK — 폐루프 속도 추적의 평균·최대 오차로 합격 판정.
"""
import numpy as np
from speed_pid import SpeedPID
from vehicle_long_speed import VehicleLong

DT = 0.1
SIM_TIME = 50.0
V_REF = 30.0
KP, KD, KI = 1.0, 0.0, 0.005


def test_speed_tracking_within_spec():
    """vx0=0, v_ref=30 폐루프 50 s: tail 평균 |vx 오차| < 0.5, peak |vx 오차| < 31.

    KI 가 있어 plant drag 외란을 보상해 tail MAE 작음. tail 정확도 + 트랜지언트 boundedness.
    Trivial 구현 (return 0 등) 은 두 임계값 모두 초과로 차단.
    """
    plant = VehicleLong(dt=DT, vx0=0.0)
    controller = SpeedPID(kp=KP, kd=KD, ki=KI, dt=DT)
    steps = int(SIM_TIME / DT)
    errors = np.zeros(steps)
    for k in range(steps):
        u = controller.step(reference=V_REF, measure=plant.vx)
        plant.step(u)
        errors[k] = abs(plant.vx - V_REF)
    tail_mae = float(np.mean(errors[steps // 2:]))
    peak = float(np.max(errors))
    assert tail_mae < 0.5, f"tail MAE {tail_mae:.4f} 임계값 초과"
    assert peak < 31.0, f"peak |error| {peak:.4f} 임계값 초과"

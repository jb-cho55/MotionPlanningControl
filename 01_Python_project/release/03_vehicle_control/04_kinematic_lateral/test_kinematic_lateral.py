"""KinematicLateral PID regression — behavioral spec (requirements level).

알고리즘 형태 (정통 PID / 다른 처리) 는 자유.
인터페이스만 맞으면 OK — 폐루프 lateral 추종의 평균·최대 오차로 합격 판정.
"""
import numpy as np
from kinematic_lateral_pid import KinematicLateralPID
from vehicle_lat_kinematic import VehicleLat

DT = 0.1
SIM_TIME = 30.0
VX = 3.0
Y_REF = 4.0
KP, KD, KI = 0.2, 0.8, 0.0


def test_lateral_tracking_within_spec():
    """vx=3, Y_ref=4 폐루프 30 s: tail 평균 |Y 오차| < 0.2 m, peak |Y 오차| < 4.2 m."""
    plant = VehicleLat(dt=DT, vx=VX)
    pid = KinematicLateralPID(kp=KP, kd=KD, ki=KI, dt=DT)
    steps = int(SIM_TIME / DT)
    errors = np.zeros(steps)
    for k in range(steps):
        delta = pid.step(reference_Y=Y_REF, ego_Y=plant.Y)
        plant.step(delta, VX)
        errors[k] = abs(plant.Y - Y_REF)
    tail_mae = float(np.mean(errors[steps // 2:]))
    peak = float(np.max(errors))
    assert tail_mae < 0.2, f"tail Y MAE {tail_mae:.4f} m 임계값 초과"
    assert peak < 4.2, f"peak |Y error| {peak:.4f} m 임계값 초과"

"""PID Controller regression — behavioral spec (requirements level).

알고리즘 형태 (정통 PID / 다른 integral 누적 방식) 는 자유.
인터페이스만 맞으면 OK — 외란 하 폐루프 평균·최대 오차로 합격 판정.
"""
import numpy as np
from pid_controller import PIDController
from plant_pid import Plant

DT = 0.1
SIM_TIME = 60.0
Y0 = 1.0
DISTURBANCE = 0.5
KP, KD, KI = 2.0, 1.0, 0.5


def test_closed_loop_under_disturbance_within_spec():
    """y0=1, target=0, 외란 0.5 폐루프 60 s: tail 평균 |오차| < 0.05, peak |오차| < 1.5.

    I 항이 외란을 적분 보상해야 tail MAE 가 작아짐 (PD-only 면 정상상태 오차 > 0.2 남음).
    Trivial / I 항 누락 구현은 tail MAE 임계값 초과로 차단.
    """
    plant = Plant(DT, y0=Y0, disturbance=DISTURBANCE)
    controller = PIDController(kp=KP, kd=KD, ki=KI, dt=DT)
    steps = int(SIM_TIME / DT)
    errors = np.zeros(steps)
    for k in range(steps):
        u = controller.step(reference=0.0, measure=plant.y)
        errors[k] = abs(plant.step(u))
    tail_mae = float(np.mean(errors[steps // 2:]))
    peak = float(np.max(errors))
    assert tail_mae < 0.05, f"tail MAE {tail_mae:.4f} 임계값 초과"
    assert peak < 1.5, f"peak |error| {peak:.4f} 임계값 초과"

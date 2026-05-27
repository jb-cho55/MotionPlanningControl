"""PID Tuning regression — behavioral spec (requirements level).

학생이 채운 KP/KD/KI 만으로 평가 — 게인 부호/크기 제약은 두지 않음.
외란 + 약한 액추에이션 하에서 평균·최대 오차 + 제어 입력 boundedness 로 합격 판정.
"""
import numpy as np
from pid_controller import PIDController
from plant_pid_tuning import Plant
from tuning import KD, KI, KP

DT = 0.1
SIM_TIME = 60.0
DISTURBANCE = 0.3
ACTUATION_GAIN = 0.5
Y0 = 1.0


def test_closed_loop_error_within_spec():
    """학생 게인 폐루프 60 s: tail 평균 |오차| < 0.1, peak |오차| < 1.5."""
    plant = Plant(DT, y0=Y0, disturbance=DISTURBANCE, actuation_gain=ACTUATION_GAIN)
    controller = PIDController(kp=KP, kd=KD, ki=KI, dt=DT)
    steps = int(SIM_TIME / DT)
    errors = np.zeros(steps)
    for k in range(steps):
        u = controller.step(reference=0.0, measure=plant.y)
        errors[k] = abs(plant.step(u))
    tail_mae = float(np.mean(errors[steps // 2:]))
    peak = float(np.max(errors))
    assert tail_mae < 0.1, f"tail MAE {tail_mae:.4f} 임계값 초과"
    assert peak < 1.5, f"peak |error| {peak:.4f} 임계값 초과"


def test_control_input_bounded():
    """제어 입력 boundedness — 발산형 튜닝 (극단적 KP 등) 차단."""
    plant = Plant(DT, y0=Y0, disturbance=DISTURBANCE, actuation_gain=ACTUATION_GAIN)
    controller = PIDController(kp=KP, kd=KD, ki=KI, dt=DT)
    steps = int(SIM_TIME / DT)
    max_abs_u = 0.0
    for _ in range(steps):
        u = controller.step(reference=0.0, measure=plant.y)
        max_abs_u = max(max_abs_u, abs(u))
        plant.step(u)
    assert max_abs_u < 50.0, f"peak |u| {max_abs_u:.2f} 임계값 초과"

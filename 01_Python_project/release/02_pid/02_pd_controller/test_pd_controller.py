"""PD Controller regression — behavioral spec (requirements level).

알고리즘 형태 (정통 PD / 다른 derivative 처리) 는 자유.
인터페이스만 맞으면 OK — 폐루프 평균·최대 오차로 합격 판정.
"""
import numpy as np
from pd_controller import PDController
from plant_pd import Plant

DT = 0.1
SIM_TIME = 30.0
Y0 = 1.0
KP = 2.0
KD = 1.0


def test_closed_loop_error_within_spec():
    """y0=1, target=0, KP=2/KD=1 폐루프 30 s: tail 평균 |오차| < 0.01, peak |오차| < 1.2.

    PD 의 D 항이 P 단독 대비 트랜지언트를 더 빨리 잠재워 tail MAE 가 훨씬 작아야 함.
    Trivial 구현 (return 0 / kp 만) 은 두 임계값 모두 초과로 차단.
    """
    plant = Plant(DT, y0=Y0)
    controller = PDController(kp=KP, kd=KD, dt=DT)
    steps = int(SIM_TIME / DT)
    errors = np.zeros(steps)
    for k in range(steps):
        u = controller.step(reference=0.0, measure=plant.y)
        errors[k] = abs(plant.step(u))
    tail_mae = float(np.mean(errors[steps // 2:]))
    peak = float(np.max(errors))
    assert tail_mae < 0.01, f"tail MAE {tail_mae:.4f} 임계값 초과"
    assert peak < 1.2, f"peak |error| {peak:.4f} 임계값 초과"

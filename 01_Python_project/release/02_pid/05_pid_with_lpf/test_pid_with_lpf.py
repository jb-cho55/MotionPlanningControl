"""PID + LPF integration regression — behavioral spec (requirements level).

closed_loop_step 의 내부 순서 (measure → filter → control → actuate) 형태는 자유.
인터페이스 (4-tuple 반환) + 외란/노이즈 시나리오의 평균·최대 오차로 합격 판정.
"""
import numpy as np
from closed_loop_lpf import closed_loop_step
from low_pass_filter import LowPassFilter
from pid_controller import PIDController
from plant_lpf import Plant

DT = 0.1
SIM_TIME = 60.0
DISTURBANCE = 0.1
NOISE_STD = 0.25
SEED = 42
KP, KD, KI = 2.0, 2.0, 0.5
ALPHA = 0.9


def _make_plant(seed: int) -> Plant:
    return Plant(
        dt=DT, y0=1.0,
        disturbance=DISTURBANCE,
        measurement_noise_std=NOISE_STD,
        rng=np.random.default_rng(seed),
    )


def _run_filtered() -> tuple[list[float], list[float]]:
    plant = _make_plant(SEED)
    lpf = LowPassFilter(alpha=ALPHA)
    pid = PIDController(kp=KP, kd=KD, ki=KI, dt=DT)
    steps = int(SIM_TIME / DT)
    y_trues, us = [], []
    for _ in range(steps):
        y_true, _, _, u = closed_loop_step(plant, lpf, pid, target=0.0)
        y_trues.append(y_true)
        us.append(u)
    return y_trues, us


def test_filtered_loop_error_within_spec():
    """LPF 폐루프 60 s: tail 평균 |오차| < 0.15, peak |오차| < 1.5."""
    y_trues, _ = _run_filtered()
    errs = np.abs(y_trues)
    tail_mae = float(np.mean(errs[len(errs) // 2:]))
    peak = float(np.max(errs))
    assert tail_mae < 0.15, f"tail MAE {tail_mae:.4f} 임계값 초과"
    assert peak < 1.5, f"peak |error| {peak:.4f} 임계값 초과"


def test_lpf_reduces_control_variance_vs_raw():
    """LPF 의 진짜 가치: D 항 노이즈 증폭 차단 → 제어 입력 std 5× 이상 감소.

    추적 오차는 raw 와 비슷하지만 actuator 출렁임이 한 자리수 이상 줄어들어야.
    LPF 효과 없는 구현 (필터 우회 등) 은 ratio < 5 로 차단.
    """
    _, u_filtered = _run_filtered()
    plant_r = _make_plant(SEED)
    pid_r = PIDController(kp=KP, kd=KD, ki=KI, dt=DT)
    steps = int(SIM_TIME / DT)
    u_raw = []
    for _ in range(steps):
        m = plant_r.measure()
        u = pid_r.step(0.0, m)
        plant_r.step(u)
        u_raw.append(u)

    tail = steps // 2
    std_filtered = float(np.std(u_filtered[tail:]))
    std_raw = float(np.std(u_raw[tail:]))
    assert std_filtered * 5.0 < std_raw, (
        f"LPF 미적용/약함 의심 — filtered std={std_filtered:.4f}, raw std={std_raw:.4f}, "
        f"감쇠비 {std_raw / std_filtered:.2f}× (임계값 5×)"
    )

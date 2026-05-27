"""PID + KF integration regression — behavioral spec (requirements level).

closed_loop_step 내부 순서/prev_u 전파 형태는 자유.
인터페이스 + 외란/노이즈 시나리오의 평균·최대 오차로 합격 판정.
"""
import numpy as np
from closed_loop_kf import closed_loop_step
from kalman_filter_2d import KalmanFilter2D
from pid_controller import PIDController
from plant_kf import Plant

DT = 0.1
SIM_TIME = 60.0
DISTURBANCE = 0.1
NOISE_STD = 0.25
SEED = 42
KP, KD, KI = 2.0, 2.0, 0.5
DAMPING = 1.0
M = 1.0


def _make_plant(seed: int) -> Plant:
    return Plant(
        dt=DT, y0=1.0,
        damping=DAMPING, m=M,
        disturbance=DISTURBANCE,
        measurement_noise_std=NOISE_STD,
        rng=np.random.default_rng(seed),
    )


def _make_kf() -> KalmanFilter2D:
    A = np.array([[1.0, DT], [0.0, 1.0 - DAMPING * DT / M]])
    B = np.array([0.0, DT / M])
    C = np.array([1.0, 0.0])
    Q = np.diag([1e-3, 1e-3])
    R = NOISE_STD ** 2
    x0 = np.array([1.0, 0.0])
    P0 = 10.0 * np.eye(2)
    return KalmanFilter2D(A=A, B=B, C=C, Q=Q, R=R, x0=x0, P0=P0)


def _run_loop() -> tuple[list[float], list[float]]:
    """Driver loop with prev_u tracking. Returns (y_trues, estimation_errors)."""
    plant = _make_plant(SEED)
    kf = _make_kf()
    pid = PIDController(kp=KP, kd=KD, ki=KI, dt=DT)
    steps = int(SIM_TIME / DT)
    y_trues, errs = [], []
    prev_u = 0.0
    for _ in range(steps):
        y_true, _, y_est, u = closed_loop_step(plant, kf, pid, target=0.0, prev_u=prev_u)
        y_trues.append(y_true)
        errs.append(abs(y_est - y_true))
        prev_u = u
    return y_trues, errs


def test_closed_loop_error_within_spec():
    """KF + PID 폐루프 60 s: tail 평균 |오차| < 0.15, peak |오차| < 1.5."""
    y_trues, _ = _run_loop()
    errs = np.abs(y_trues)
    tail_mae = float(np.mean(errs[len(errs) // 2:]))
    peak = float(np.max(errs))
    assert tail_mae < 0.15, f"tail MAE {tail_mae:.4f} 임계값 초과"
    assert peak < 1.5, f"peak |error| {peak:.4f} 임계값 초과"


def test_kf_estimation_better_than_raw_noise():
    """KF 추정 평균 |오차| < 0.10 — raw 측정 노이즈 (std 0.25, 평균 |오차| ≈ 0.2) 의 절반 이하."""
    _, errs = _run_loop()
    tail_est_mae = float(np.mean(errs[len(errs) // 2:]))
    assert tail_est_mae < 0.10, f"KF 추정 tail MAE {tail_est_mae:.4f} 임계값 초과"

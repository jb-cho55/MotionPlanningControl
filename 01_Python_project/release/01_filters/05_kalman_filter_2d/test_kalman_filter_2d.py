"""KalmanFilter2D regression — behavioral spec (requirements level).

알고리즘 형태 (정통 Kalman / 단순 추정기 / 다른 방식) 는 자유.
인터페이스만 맞으면 OK — 위치·속도 RMS 오차로 합격 판정.
"""
import numpy as np
from kalman_filter_2d import KalmanFilter2D


def _cv_matrices(dt=0.1):
    A = np.array([[1.0, dt], [0.0, 1.0]])
    B = np.array([0.0, dt])
    C = np.array([1.0, 0.0])
    return A, B, C


def test_constant_velocity_tracking_rms_within_spec():
    """등속 truth (v=2.0) + 위치만 N(0, 0.5) 노이즈, 300 step: warm-up 이후
    위치 RMS < 0.5, 속도 RMS < 0.3.

    RMS = sqrt(bias² + variance) — bias 와 variance 모두 한 번에 잡힘.
    상수 / 패스스루 류 trivial 구현은 임계값 초과로 차단.
    """
    rng = np.random.default_rng(0)
    dt = 0.1
    n = 300
    v_truth = 2.0
    A, B, C = _cv_matrices(dt=dt)
    Q = np.diag([0.01, 0.05])
    R = 0.5
    kf = KalmanFilter2D(A=A, B=B, C=C, Q=Q, R=R)

    truth_pos = v_truth * np.arange(n) * dt
    pos_est = np.zeros(n)
    vel_est = np.zeros(n)
    for k in range(n):
        z = truth_pos[k] + rng.normal(0, 0.5)
        state = kf.step(measurement=z, control_input=0.0)
        pos_est[k] = state[0]
        vel_est[k] = state[1]

    warm = 100
    pos_rms = float(np.sqrt(np.mean((pos_est[warm:] - truth_pos[warm:]) ** 2)))
    vel_rms = float(np.sqrt(np.mean((vel_est[warm:] - v_truth) ** 2)))
    assert pos_rms < 0.5
    assert vel_rms < 0.3

"""KalmanFilter regression — behavioral spec (requirements level).

알고리즘 형태 (정통 Kalman / 단순 LPF 흉내 / 다른 추정기) 는 자유.
인터페이스만 맞으면 OK — RMS 오차로 합격 판정.
"""
import numpy as np
from kalman_filter import KalmanFilter


def test_constant_signal_is_stable():
    """무노이즈 측정 + 모델 정합 feedforward: 출력이 truth 근방에서 안정."""
    truth = 5.0
    kf = KalmanFilter(m=1.0, dt=0.01, q=0.01, r=1.0, p0=10.0)
    u_ff = -truth  # A·truth + B·u = truth 정합 (모델 drift 상쇄)
    out = 0.0
    for _ in range(2000):
        out = kf.step(measurement=truth, control_input=u_ff)
    assert abs(out - truth) < 0.1


def test_noisy_tracking_rms_within_spec():
    """상수 truth=5 + N(0, 1) 노이즈, feedforward u=-truth: warm-up 이후 RMS < 0.3.

    RMS = sqrt(bias² + variance) — bias 와 variance 모두 한 번에 잡힘.
    'return 0' (bias 임계값 초과) / 'return x' (variance 임계값 초과) 모두 차단.
    """
    rng = np.random.default_rng(0)
    truth = 5.0
    kf = KalmanFilter(m=1.0, dt=0.01, q=0.01, r=1.0, p0=10.0)
    u_ff = -truth
    estimates = []
    for _ in range(10000):
        z = truth + rng.normal(0, 1.0)
        estimates.append(kf.step(z, control_input=u_ff))
    estimates = np.array(estimates)
    rms = float(np.sqrt(np.mean((estimates[1000:] - truth) ** 2)))
    assert rms < 0.3

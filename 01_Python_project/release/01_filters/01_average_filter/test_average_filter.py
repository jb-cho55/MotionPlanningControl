"""AverageFilter regression — behavioral spec (requirements level).

알고리즘 형태 (재귀 누적 / list mean / 다른 방식) 는 자유.
인터페이스만 맞으면 OK — RMS 오차로 합격 판정.
"""
import numpy as np
from average_filter import AverageFilter


def test_constant_signal_is_stable():
    """상수 입력에 출력이 흔들리지 않음."""
    af = AverageFilter()
    out = 0.0
    for _ in range(100):
        out = af.step(2.0)
    assert abs(out - 2.0) < 1e-6


def test_noisy_tracking_rms_within_spec():
    """상수 truth=5 + N(0, 1) 노이즈 1만 표본: warm-up 이후 RMS < 0.1.

    RMS = sqrt(bias² + variance) — bias 와 variance 모두 한 번에 잡힘.
    'return 0' (bias 임계값 초과) / 'return x' (variance 임계값 초과) 모두 차단.
    """
    rng = np.random.default_rng(0)
    truth = 5.0
    af = AverageFilter()
    samples = rng.normal(truth, 1.0, size=10000)
    estimates = np.array([af.step(s) for s in samples])
    rms = float(np.sqrt(np.mean((estimates[100:] - truth) ** 2)))
    assert rms < 0.1

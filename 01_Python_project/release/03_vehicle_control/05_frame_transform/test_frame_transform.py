"""Frame Transform regression — requirements level (수치 정확성).

이 모듈은 06/07/08/09 의 pipeline 빌딩 블록 (순수 수학 유틸리티).
'평균 오차 < X' 시나리오가 아니라 'tolerance 이내 수치 정답' 으로 검증.
구현 형태 (numpy / scipy / pure python) 는 자유.
"""
import numpy as np
from frame_transform import Global2Local, PolynomialFitting, PolynomialValue


def test_global2local_translation_and_rotation():
    """Yaw=π/2 (북향) + ego at (10, 5): global (11, 5) → local (0, -1).

    회전 + 평행이동이 동시에 작동하는지 한 step 으로 검증.
    """
    points = np.array([[11.0, 5.0]])
    g2l = Global2Local(num_points=1)
    out = g2l.convert(points, yaw_ego=np.pi / 2, x_ego=10.0, y_ego=5.0)
    assert np.allclose(out, [[0.0, -1.0]], atol=1e-12)


def test_polynomial_fit_recovers_known_coeffs():
    """y = 2x³ - x² + 3x - 1 의 sample 들을 fit → 계수 정확 (tolerance 이내)."""
    coeff_truth = np.array([[2.0], [-1.0], [3.0], [-1.0]])
    xs = np.array([0.0, 1.0, 2.0, 3.0])
    ys = 2 * xs**3 - xs**2 + 3 * xs - 1
    points = np.column_stack([xs, ys])
    fitter = PolynomialFitting(degree=3, num_points=4)
    coeff = fitter.fit(points)
    assert np.allclose(coeff, coeff_truth, atol=1e-9)


def test_polynomial_value_evaluates_correctly():
    """coeff = [0, 0, 2, -1] (degree 3) → y = 2x - 1 (모든 x 에서)."""
    coeff = np.array([[0.0], [0.0], [2.0], [-1.0]])
    xs = np.array([0.0, 1.0, 2.0, 3.0])
    ev = PolynomialValue(degree=3, num_x=4)
    y = ev.calculate(coeff, xs)
    expected = 2 * xs - 1
    assert np.allclose(y.flatten(), expected)

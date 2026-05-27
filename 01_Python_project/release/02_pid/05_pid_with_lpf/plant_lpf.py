"""Plant — 1-D damped point mass with disturbance + Gaussian measurement noise.

이 파일은 검증 환경의 일부입니다. 수정하지 마세요.

Dynamics:
    v' = (u + disturbance - c*v) / m
    y' = v
    y_measure = y_true + N(0, R²)
"""
from __future__ import annotations

import numpy as np


class Plant:
    def __init__(
        self,
        dt: float = 0.1,
        y0: float = 1.0,
        v0: float = 0.0,
        damping: float = 1.0,
        m: float = 1.0,
        disturbance: float = 0.0,
        measurement_noise_std: float = 0.0,
        rng: np.random.Generator | None = None,
    ):
        self.dt = dt
        self.y = y0
        self.v = v0
        self.damping = damping
        self.m = m
        self.disturbance = disturbance
        self.measurement_noise_std = measurement_noise_std
        self.rng = rng if rng is not None else np.random.default_rng()

    def step(self, u: float) -> float:
        a = (u + self.disturbance - self.damping * self.v) / self.m
        self.v += a * self.dt
        self.y += self.v * self.dt
        return self.y

    def measure(self) -> float:
        return float(self.y + self.rng.normal(0.0, self.measurement_noise_std))

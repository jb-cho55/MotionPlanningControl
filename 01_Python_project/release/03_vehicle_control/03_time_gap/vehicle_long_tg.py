"""VehicleLong — 종방향 점질량 + drag + grade + ax limit. Fixed simulation environment.

이 파일은 검증 환경의 일부입니다. 수정하지 마세요.
(01/02 의 동일 모델 사본)

Dynamics (per step):
    x_{k+1}  = x_k + dt · vx_k + 0.5 · dt² · ax_k
    vx_{k+1} = vx_k + dt · ax_k
    ax_{k+1} = clip(u - C · vx_{k+1}² - g · sin(θ),  -ax_limit, +ax_limit)
"""
from __future__ import annotations

import numpy as np


class VehicleLong:
    def __init__(
        self,
        dt: float = 0.1,
        m: float = 500.0,
        Ca: float = 0.5,
        x0: float = 0.0,
        vx0: float = 0.0,
        grade: float = 0.0,
        ax_limit: float = 2.0,
    ):
        self.dt = dt
        self.m = m
        self.g = 9.81
        self.C = Ca / m
        self.theta = grade
        self.ax_limit = ax_limit
        self.x = x0
        self.vx = vx0
        self.ax = 0.0

    def step(self, ax_cmd: float) -> None:
        self.x = self.x + self.dt * self.vx + 0.5 * self.dt**2 * self.ax
        self.vx = self.vx + self.dt * self.ax
        self.ax = float(np.clip(
            ax_cmd - self.C * self.vx**2 - self.g * np.sin(self.theta),
            -self.ax_limit, self.ax_limit,
        ))

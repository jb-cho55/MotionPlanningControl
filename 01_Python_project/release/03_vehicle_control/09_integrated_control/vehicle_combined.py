"""VehicleCombined — kinematic bicycle plant 로 종+횡 동시 제어.

04/06/07/08 의 VehicleLat 는 vx 가 외부 입력 (lateral-only). 09 는 종 제어도 학습 대상이라
ax 입력 + vx 가 자체 상태.

이 파일은 검증 환경의 일부입니다. 수정하지 마세요.
"""
from __future__ import annotations

import numpy as np


class VehicleCombined:
    def __init__(
        self,
        dt: float,
        vx0: float = 10.0,
        m: float = 500.0,
        L: float = 4.0,
        kv: float = 0.005,
        X0: float = 0.0,
        Y0: float = 0.0,
        Yaw0: float = 0.0,
        ax_min: float = -5.0,
        ax_max: float = 3.0,
    ):
        self.dt = dt
        self.m = m
        self.L = L
        self.kv = kv
        self.ax_min = ax_min
        self.ax_max = ax_max
        self.vx = vx0
        self.X = X0
        self.Y = Y0
        self.Yaw = Yaw0
        self.delta = 0.0
        self.ax = 0.0
        self.yawrate = 0.0

    def step(self, delta: float, ax: float) -> None:
        self.delta = float(np.clip(delta, -0.5, 0.5))
        self.ax = float(np.clip(ax, self.ax_min, self.ax_max))
        self.vx = max(0.0, self.vx + self.ax * self.dt)
        self.yawrate = self.vx / (self.L + self.kv * self.vx**2) * self.delta
        self.Yaw = self.Yaw + self.dt * self.yawrate
        self.X = self.X + self.vx * self.dt * np.cos(self.Yaw)
        self.Y = self.Y + self.vx * self.dt * np.sin(self.Yaw)

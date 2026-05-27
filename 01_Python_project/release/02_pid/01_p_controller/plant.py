"""Plant — discrete 1-D damped point mass. Fixed simulation environment.

이 파일은 검증 환경의 일부입니다. 수정하지 마세요.

Dynamics:
    v' = (u - c*v) / m
    y' = v
"""
from __future__ import annotations


class Plant:
    def __init__(
        self,
        dt: float = 0.1,
        y0: float = 1.0,
        v0: float = 0.0,
        damping: float = 1.0,
        m: float = 1.0,
    ):
        self.dt = dt
        self.y = y0
        self.v = v0
        self.damping = damping
        self.m = m

    def step(self, u: float) -> float:
        a = (u - self.damping * self.v) / self.m
        self.v += a * self.dt
        self.y += self.v * self.dt
        return self.y

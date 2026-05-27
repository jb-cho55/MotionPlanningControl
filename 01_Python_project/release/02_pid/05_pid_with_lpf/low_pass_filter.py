"""Low-Pass Filter — 01_filters/03_low_pass_filter 의 정답 사본 (재구현 대상 X).

본 모듈에서는 학생이 LPF 를 다시 구현하지 않습니다 — fixture 로 제공.
"""
from __future__ import annotations


class LowPassFilter:
    def __init__(self, alpha: float = 0.9):
        self.alpha = alpha
        self.y: float | None = None

    def step(self, x: float) -> float:
        if self.y is None:
            self.y = x
        else:
            self.y = self.alpha * self.y + (1.0 - self.alpha) * x
        return self.y

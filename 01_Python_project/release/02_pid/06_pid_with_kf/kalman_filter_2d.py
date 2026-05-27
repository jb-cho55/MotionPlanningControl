"""Kalman Filter 2D — 01_filters/05_kalman_filter_2d 의 정답 사본 (재구현 대상 X).

본 모듈에서는 학생이 KF 를 다시 구현하지 않습니다 — fixture 로 제공.
"""
from __future__ import annotations

import numpy as np


class KalmanFilter2D:
    def __init__(self, A: np.ndarray, B: np.ndarray, C: np.ndarray,
                 Q: np.ndarray, R: float,
                 x0: np.ndarray | None = None,
                 P0: np.ndarray | None = None):
        self.A = A
        self.B = B
        self.C = C
        self.Q = Q
        self.R = R
        self.x = x0 if x0 is not None else np.zeros(2)
        self.P = P0 if P0 is not None else 10.0 * np.eye(2)

    def step(self, measurement: float, control_input: float) -> np.ndarray:
        x_pred = self.A @ self.x + self.B * control_input
        P_pred = self.A @ self.P @ self.A.T + self.Q
        S = self.C @ P_pred @ self.C + self.R
        K = (P_pred @ self.C) / S
        innovation = measurement - self.C @ x_pred
        self.x = x_pred + K * innovation
        self.P = (np.eye(2) - np.outer(K, self.C)) @ P_pred
        return self.x

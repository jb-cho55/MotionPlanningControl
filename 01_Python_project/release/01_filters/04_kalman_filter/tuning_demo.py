"""Kalman Filter 튜닝 데모 — 4 가지 (q, r) 조합 비교 (plotly, 브라우저 렌더).

같은 KalmanFilter 클래스를 다양한 (q, r) 조합으로 적용하여 추정 결과를 비교.
- (q=0.01, r=0.1):  측정에 매우 빠르게 추종 (gain ↑)
- (q=0.01, r=10):   측정 영향 미미, 모델에 의존 (gain ↓)
- (q=10, r=0.01):   gain 매우 큼 → 측정 거의 그대로
- (q=0.01, r=1.0):  균형 (04 의 기본값과 유사)

실행 전 `kalman_filter.py` 의 `# TODO` 를 구현해야 동작합니다.
"""
from __future__ import annotations

import numpy as np
import plotly.graph_objects as go
from kalman_filter import KalmanFilter


def main() -> None:
    rng = np.random.default_rng(0)
    n = 300
    truth = 5.0
    samples = truth + rng.normal(loc=0.0, scale=1.0, size=n)
    u_ff = -truth  # feedforward to match A·truth + B·u = truth

    presets = [
        {"label": "q=0.01, r=0.1 (측정 추종)", "q": 0.01, "r": 0.1, "color": "red"},
        {"label": "q=0.01, r=10  (모델 의존)", "q": 0.01, "r": 10.0, "color": "blue"},
        {"label": "q=10,   r=0.01 (측정 그대로)", "q": 10.0, "r": 0.01, "color": "green"},
        {"label": "q=0.01, r=1.0 (균형)", "q": 0.01, "r": 1.0, "color": "purple"},
    ]

    fig = go.Figure()
    fig.add_hline(y=truth, line=dict(color="black", dash="dash"), annotation_text="truth")
    fig.add_trace(go.Scatter(y=samples, mode="markers", name="measurements",
                             marker=dict(color="gray", opacity=0.3, size=4)))

    for p in presets:
        kf = KalmanFilter(q=p["q"], r=p["r"])
        estimate = np.array([kf.step(z, u_ff) for z in samples])
        fig.add_trace(go.Scatter(y=estimate, mode="lines", name=p["label"],
                                 line=dict(color=p["color"], width=2)))

    fig.update_layout(
        title="Kalman Filter — q/r 튜닝 비교",
        xaxis_title="step",
        yaxis_title="value",
        template="plotly_white",
    )
    fig.show()


if __name__ == "__main__":
    main()

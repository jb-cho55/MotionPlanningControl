# ruff: noqa: E402, I001
"""4 필터 비교 데모 — Average / Moving Average / Low-Pass / Kalman.

같은 신호 (truth + 노이즈) 에 4 개 필터를 동시에 적용하여 plotly 그래프로 비교.

실행 전 4 개 필터의 `# TODO` 가 모두 구현되어 있어야 합니다 (01-04 과제 완료 후).
하나라도 미구현이면 첫 step 호출에서 `NotImplementedError` 발생.
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import plotly.graph_objects as go

HERE = Path(__file__).parent
for sub in ["01_average_filter", "02_moving_average_filter",
            "03_low_pass_filter", "04_kalman_filter"]:
    sys.path.insert(0, str(HERE / sub))

from average_filter import AverageFilter
from moving_average_filter import MovingAverageFilter
from low_pass_filter import LowPassFilter
from kalman_filter import KalmanFilter


def main() -> None:
    rng = np.random.default_rng(0)
    n = 500
    t = np.arange(n)
    truth = 5.0 + np.sin(0.03 * t)
    samples = truth + rng.normal(loc=0.0, scale=0.5, size=n)

    af = AverageFilter()
    maf = MovingAverageFilter(window=20)
    lpf = LowPassFilter(alpha=0.9)
    kf = KalmanFilter(m=1.0, dt=0.01, q=0.1, r=0.5, p0=10.0)

    y_af = np.array([af.step(s) for s in samples])
    y_maf = np.array([maf.step(s) for s in samples])
    y_lpf = np.array([lpf.step(s) for s in samples])
    y_kf = np.array([kf.step(s, control_input=0.0) for s in samples])

    fig = go.Figure()
    fig.add_trace(go.Scatter(x=t, y=truth, mode="lines", name="truth",
                             line=dict(color="black", dash="dash")))
    fig.add_trace(go.Scatter(x=t, y=samples, mode="markers", name="measurements",
                             marker=dict(color="gray", opacity=0.3, size=4)))
    fig.add_trace(go.Scatter(x=t, y=y_af, mode="lines", name="Average",
                             line=dict(color="magenta", width=2)))
    fig.add_trace(go.Scatter(x=t, y=y_maf, mode="lines", name="Moving Average (W=20)",
                             line=dict(color="blue", width=2)))
    fig.add_trace(go.Scatter(x=t, y=y_lpf, mode="lines", name="Low-Pass (α=0.9)",
                             line=dict(color="cyan", width=2)))
    fig.add_trace(go.Scatter(x=t, y=y_kf, mode="lines", name="Kalman",
                             line=dict(color="red", width=2)))
    fig.update_layout(
        title="4 필터 비교 — truth = 5 + sin(0.03·t), noise N(0, 0.5)",
        xaxis_title="step",
        yaxis_title="value",
        template="plotly_white",
    )
    fig.show()


if __name__ == "__main__":
    main()

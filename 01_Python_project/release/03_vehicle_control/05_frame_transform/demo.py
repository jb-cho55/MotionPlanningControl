"""Frame Transform — global path + ego pose → local frame fit/evaluate.

본 모듈은 시간축이 없는 **정적 데모** — Rerun 시간축 재생 가치가 없어
plotly 2-패널 (global / local) 로 좌표 변환의 전/후를 직접 보여준다.
다른 03_vehicle_control 모듈이 `record_gen.py` + simulator 로 시간축 재생하는
것과 다른 점 (§release_pipeline §record_gen.py 패턴의 정적-데모 예외).

실행 전 `frame_transform.py` 의 `# TODO` 를 구현해야 동작합니다 — 구현 전이면 NotImplementedError.
"""
from __future__ import annotations

import numpy as np
import plotly.graph_objects as go
from frame_transform import Global2Local, PolynomialFitting, PolynomialValue
from plotly.subplots import make_subplots


def _vehicle_box(x: float, y: float, yaw: float,
                 L: float = 1.0, W: float = 0.6) -> tuple[np.ndarray, np.ndarray]:
    """차량 직사각형 5 꼭짓점 (close polygon) — yaw 회전 적용. 진행방향 = +x_local."""
    hL, hW = L / 2.0, W / 2.0
    corners = np.array([(+hL, +hW), (+hL, -hW), (-hL, -hW), (-hL, +hW), (+hL, +hW)])
    c, s = np.cos(yaw), np.sin(yaw)
    rot = np.array([[c, -s], [s, c]])
    rotated = (rot @ corners.T).T
    return rotated[:, 0] + x, rotated[:, 1] + y


def _vehicle_nose(x: float, y: float, yaw: float,
                  L: float = 1.0, W: float = 0.6) -> tuple[np.ndarray, np.ndarray]:
    """전방 삼각형 — box 의 앞쪽 절반에 inscribed. 진행방향 강조."""
    hL, hW = L / 2.0, W / 2.0
    corners = np.array([(+hL, 0.0), (0.0, +hW), (0.0, -hW), (+hL, 0.0)])
    c, s = np.cos(yaw), np.sin(yaw)
    rot = np.array([[c, -s], [s, c]])
    rotated = (rot @ corners.T).T
    return rotated[:, 0] + x, rotated[:, 1] + y


def main() -> None:
    num_degree = 3
    num_point = 4
    points = np.array([[1.0, 2.0], [3.0, 3.0], [4.0, 4.0], [5.0, 5.0]])
    x_ego = 2.0
    y_ego = 0.0
    yaw_ego = np.pi / 4
    x_local = np.arange(0.0, 10.0, 0.5)

    g2l = Global2Local(num_point)
    fitter = PolynomialFitting(num_degree, num_point)
    ev = PolynomialValue(num_degree, np.size(x_local))

    g2l.convert(points, yaw_ego, x_ego, y_ego)
    fitter.fit(g2l.local_points)
    ev.calculate(fitter.coeff, x_local)

    fig = make_subplots(rows=1, cols=2,
                        subplot_titles=("Global Frame (path + ego pose)",
                                        "Local Frame (path 변환 + polynomial fit)"))
    fig.add_trace(go.Scatter(x=points[:, 0], y=points[:, 1], mode="markers",
                             name="path points (global)",
                             marker=dict(color="blue", size=8)), row=1, col=1)
    ex_g, ey_g = _vehicle_box(x_ego, y_ego, yaw_ego)
    nx_g, ny_g = _vehicle_nose(x_ego, y_ego, yaw_ego)
    fig.add_trace(go.Scatter(x=ex_g, y=ey_g, mode="lines", fill="toself", name="ego",
                             line=dict(color="red", width=2),
                             fillcolor="rgba(255,0,0,0.15)"), row=1, col=1)
    fig.add_trace(go.Scatter(x=nx_g, y=ny_g, mode="lines", fill="toself",
                             line=dict(color="red", width=2),
                             fillcolor="rgba(255,0,0,0.15)",
                             showlegend=False), row=1, col=1)
    fig.add_trace(go.Scatter(x=g2l.local_points[:, 0], y=g2l.local_points[:, 1],
                             mode="markers", name="path points (local)",
                             marker=dict(color="blue", size=8)), row=1, col=2)
    fig.add_trace(go.Scatter(x=ev.points[:, 0], y=ev.points[:, 1],
                             mode="lines", name="polynomial fit",
                             line=dict(color="orange", width=2, dash="dot")),
                  row=1, col=2)
    ex_l, ey_l = _vehicle_box(0.0, 0.0, 0.0)
    nx_l, ny_l = _vehicle_nose(0.0, 0.0, 0.0)
    fig.add_trace(go.Scatter(x=ex_l, y=ey_l, mode="lines", fill="toself",
                             name="ego (origin)", line=dict(color="red", width=2),
                             fillcolor="rgba(255,0,0,0.15)"), row=1, col=2)
    fig.add_trace(go.Scatter(x=nx_l, y=ny_l, mode="lines", fill="toself",
                             line=dict(color="red", width=2),
                             fillcolor="rgba(255,0,0,0.15)",
                             showlegend=False), row=1, col=2)
    fig.update_xaxes(title_text="X (m)", row=1, col=1)
    fig.update_xaxes(title_text="x (m, local)", range=[-5, 5], row=1, col=2)
    fig.update_yaxes(title_text="Y (m)", scaleanchor="x", scaleratio=1, row=1, col=1)
    fig.update_yaxes(title_text="y (m, local)", range=[-5, 5],
                     scaleanchor="x2", scaleratio=1, row=1, col=2)
    fig.update_layout(template="plotly_white",
                      title="Frame Transform — global ↔ local + polynomial fit")
    fig.show()


if __name__ == "__main__":
    main()

# 과제 — Kalman Filter (scalar)

## 목표
1차원 선형 시스템의 **칼만 필터** 를 구현한다. 모델 기반 예측 (Predict) 과 측정 기반 보정 (Update) 을 결합해, 이전 모듈 (Average / Moving Average / Low-Pass) 보다 한 단계 위 추정을 학습한다.

## 시스템 모델 (참고)
```
state    : x_{k+1} = A · x_k + B · u_k + w_k,    w ~ N(0, Q)
measure  : z_k     = C · x_k + v_k,              v ~ N(0, R)
```
여기서 `A = 1 + dt`, `B = dt / m`, `C = 1` (단순 1D 적분기). `__init__` 에서 자동 설정됨.

## 알고리즘 (구현해야 할 6줄)
**Predict:**
```
x_pred = A · x + B · u
P_pred = A² · P + Q
```
**Update:**
```
K = P_pred · C / (C² · P_pred + R)
x = x_pred + K · (z - C · x_pred)
P = (1 - K · C) · P_pred
```
`self.x`, `self.P` 매 step 갱신, `self.x` 반환.

## 인터페이스 계약
**이 시그니처는 변경하지 마세요.** 채점/테스트가 이 형태에 의존합니다.

```python
class KalmanFilter:
    def __init__(self, m: float = 1.0, dt: float = 0.01,
                 q: float = 0.1, r: float = 0.9, p0: float = 10.0): ...
    def step(self, measurement: float, control_input: float) -> float
```

- `__init__` 은 이미 작성되어 있음 — `A`, `B`, `C`, `Q`, `R`, `self.x = 0`, `self.P = p0` 가 자동 설정
- 학생이 작성할 부분은 `step` 의 6 줄 (predict 2 + update 3 + return 1)

## 구현 위치
`01_Python_project/release/01_filters/04_kalman_filter/kalman_filter.py` 의 `step` 메소드 안 `# TODO:` 블록.

## 실행

> 환경 셋업 (1회) 과 명령 실행 규칙은 [`../../README.md`](../../README.md) 참조. **git root 에서 실행.**

테스트 (합격 검증):
```bash
uv run pytest 01_Python_project/release/01_filters/04_kalman_filter/ -v
```

데모 (시각 확인, 선택 — 구현 후):
```bash
uv run python 01_Python_project/release/01_filters/04_kalman_filter/demo.py
```
→ 상수 truth (5.0) + N(0, 1) 노이즈 측정 500 표본을 칼만으로 추정한 결과 plotly 그래프.

튜닝 데모 (선택 — 구현 후):
```bash
uv run python 01_Python_project/release/01_filters/04_kalman_filter/tuning_demo.py
```
→ 4 가지 (q, r) 조합 비교: 측정 추종 / 모델 의존 / 측정 그대로 / 균형.

## 합격 기준 (`pytest` 통과)
학생이 푼 알고리즘 형태 (정통 Kalman / 단순 LPF 흉내 / 다른 추정기) 는 제약 X — **behavioral spec** 만 본다.

1. **무노이즈 + feedforward 안정성** — `q=0.01, r=1.0, u=-truth` 정합 setup, 2000 step 후 `|x - truth| < 0.1`
2. **노이즈 추적 RMS** — 위와 같은 setup 에 N(0, 1) 노이즈 측정 1만 step, warm-up 이후 RMS 오차 `< 0.3`

> RMS = √(bias² + variance). trivial 구현 (`return 0` / `return measurement`) 은 두 임계값 중 하나 이상 초과로 차단.

## 힌트
- 위 알고리즘 6줄을 그대로 옮기면 됩니다 — 표준 칼만이라 정답 식 자체는 노출 OK. 핵심은 정확히 코드로 옮기는 연습.
- 임시 변수 `x_pred`, `p_pred`, `k` 를 만들어 가독성 확보 권장
- `self.x` 는 step 끝에서 한 번만 갱신 (`x_pred` 를 곧장 self.x 로 박아두면 update 가 망가짐 — 두 단계가 분리되어야 함)
- `(1 - K·C)` 부호 주의 — 잘못 쓰면 P 발산
- 합격 기준 4 의 `u=-5.0` 은 `A·truth + B·u = truth` 가 성립하도록 한 feedforward (모델 drift 상쇄). 학생이 이걸 손으로 유도할 필요는 없고, 테스트가 적절히 셋업해줍니다.

## 게인/파라미터 튜닝 위치

라이브러리 코드 (`.py` 안의 클래스·함수) 는 **시그니처만** 정의 — kp/kd/ki, window_size, R/Q, lookahead 등은 매개변수로만 받는다. 실제 *값* 은 두 곳에서 명시:

- **시각화/실행 (자유롭게 변경 OK, **release 기본값은 모두 0**)**: 같은 폴더의 `record_gen.py` / `demo.py` (시나리오 여럿이면 `record_gen_<scenario>.py`) 안의 게인/파라미터가 0 으로 초기화되어 있음 → **학생이 직접 채워야 응답이 나옴**. 0 인 채로 실행하면 controller 출력 0, 응답 없음 (또는 NaN/division 에러). 값을 바꿔 다시 실행하며 응답 변화 비교.
- **합격 기준 검증 (변경 금지)**: `test_*.py` 안에 박혀 있음. pytest 가 이 값으로 통과 여부를 본다 — 임의로 바꾸면 검증 의미가 사라짐.

즉 "다른 게인은 어떻게 동작하지?" 는 producer 만 바꾸고, "내 구현이 spec 을 통과하는가?" 는 test 그대로 두고 `pytest` 만 돌리면 된다.

## 문제별 추가 제약
(공통 제약은 [`../../README.md`](../../README.md) 의 "AI 도구 사용 가이드" 참조)

이 문제에 한정한 추가 사항은 없습니다.

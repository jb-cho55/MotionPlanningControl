# 과제 — PID Controller (비례 + 적분 + 미분 제어)

## 목표
PD 에 적분(I) 항을 더해 외란(disturbance)이 있는 상황에서도 정상상태 오차를 0 으로 수렴시킨다.
`Ki = 0` 으로 두면 PD 와 동등 — 알고리즘 일관성도 확인한다.

## 인터페이스 계약
**이 시그니처는 변경하지 마세요.**

```python
class PIDController:
    def __init__(self, kp: float, kd: float, ki: float, dt: float): ...
    def step(self, reference: float, measure: float) -> float
```

- 내부 상태: `prev_error` (D 항용), `error_sum` (I 항용 누적합).
- 첫 호출 시 D 항 처리 정책은 PD 과제와 동일 (이전 오차 없으면 D 기여 0).

## 구현 위치
`01_Python_project/release/02_pid/03_pid_controller/pid_controller.py` 의 `step` 메소드 안 `# TODO:` 블록.

## 실행

> 환경 셋업은 [`../../README.md`](../../README.md) 참조. **git root 에서 실행.**

테스트:
```bash
uv run pytest 01_Python_project/release/02_pid/03_pid_controller/ -v
```

시나리오 실행 → `record.json` 생성 + Rerun viewer 자동 띄움:
```bash
uv run python 01_Python_project/release/02_pid/03_pid_controller/record_gen.py
```
→ 외란 0.5 하에서 두 차량 동시 시뮬: PD (회색) 는 차로 중앙 위쪽으로 offset, PID (파랑) 는 중앙으로 수렴 — I 항의 외란 보상 효과 3D 시각화.

> JSON 만 만들고 viewer 안 띄우려면 record_gen 명령에 `--no-viewer` 옵션 추가.

Rerun viewer 로 재생:
```bash
uv run python 01_Python_project/release/02_pid/simulator_pid.py 01_Python_project/release/02_pid/03_pid_controller/
```

> **시뮬레이터는 챕터 전체용** — 인자 없이 실행하면 `02_pid/` 하위 모든 시나리오를 한 viewer 에 별도 recording 으로 멀티 로드, viewer 좌측 Recordings 패널에서 클릭 전환. `--camera follow|fixed` 로 초기 카메라 (기본 `follow`).

## 합격 기준 (`pytest` 통과)
학생이 푼 알고리즘 형태 (정통 PID / 다른 integral 누적 방식) 는 제약 X — **behavioral spec** 만 본다.

1. **외란 하 폐루프 추적 오차** — 외란 0.5, `kp=2.0, kd=1.0, ki=0.5`, 60 초 시뮬, tail 평균 `|error| < 0.05`, peak `|error| < 1.5`

> I 항이 외란을 적분 보상해야 tail MAE 가 작음 (PD-only 면 잔류 오차 > 0.2 남음). I 항 누락 구현은 tail 임계값 초과로 차단.

## 힌트
- 식: `u = Kp · 오차 + Kd · 오차의 미분 + Ki · 오차의 누적합`
- 누적합: `error_sum += 오차 · dt` (**`dt` 곱 누락 자주 발생**)
- 외란이 있어 평형 상태에서 `u_ss ≠ 0` 이 필요 — I 항이 `Ki · error_sum` 을 키워 이를 만든다.
- D 항 처리는 PD 과제와 동일 (첫 호출 0, 이후 미분 정규화).

## 게인/파라미터 튜닝 위치

라이브러리 코드 (`.py` 안의 클래스·함수) 는 **시그니처만** 정의 — kp/kd/ki, window_size, R/Q, lookahead 등은 매개변수로만 받는다. 실제 *값* 은 두 곳에서 명시:

- **시각화/실행 (자유롭게 변경 OK, **release 기본값은 모두 0**)**: 같은 폴더의 `record_gen.py` / `demo.py` (시나리오 여럿이면 `record_gen_<scenario>.py`) 안의 게인/파라미터가 0 으로 초기화되어 있음 → **학생이 직접 채워야 응답이 나옴**. 0 인 채로 실행하면 controller 출력 0, 응답 없음 (또는 NaN/division 에러). 값을 바꿔 다시 실행하며 응답 변화 비교.
- **합격 기준 검증 (변경 금지)**: `test_*.py` 안에 박혀 있음. pytest 가 이 값으로 통과 여부를 본다 — 임의로 바꾸면 검증 의미가 사라짐.

즉 "다른 게인은 어떻게 동작하지?" 는 producer 만 바꾸고, "내 구현이 spec 을 통과하는가?" 는 test 그대로 두고 `pytest` 만 돌리면 된다.

## 문제별 추가 제약
- **`plant_pid.py` 절대 수정 금지**.
- I 항의 anti-windup, leak 등 추가 처리는 본 과제 범위 밖 (단순 Σ 만).

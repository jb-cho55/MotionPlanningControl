# 과제 — P Controller (비례 제어)

## 목표
오차(목표값 − 현재값) 에 비례하는 제어 입력을 생성하는 비례 제어기를 구현한다.
폐루프 시뮬레이션에서 점질량 시스템이 목표 위치로 수렴하는 것을 확인한다.

## 인터페이스 계약
**이 시그니처는 변경하지 마세요.** 채점/테스트가 이 형태에 의존합니다.

```python
class PController:
    def __init__(self, kp: float): ...
    def step(self, reference: float, measure: float) -> float
```

- `step(ref, m)` — 매 호출마다 새 제어 입력을 반환. **내부 상태 없음** (P 제어기는 memoryless).

## 구현 위치
`01_Python_project/release/02_pid/01_p_controller/p_controller.py` 의 `step` 메소드 안 `# TODO:` 블록.

## 실행

> 환경 셋업 (1회) 과 명령 실행 규칙은 [`../../README.md`](../../README.md) 참조. **git root 에서 실행.**

테스트 (합격 검증):
```bash
uv run pytest 01_Python_project/release/02_pid/01_p_controller/ -v
```

시나리오 실행 → `record.json` 생성 + Rerun viewer 자동 띄움:
```bash
uv run python 01_Python_project/release/02_pid/01_p_controller/record_gen.py
```
→ 파란 차량이 차로 중앙 (Y=0) 으로 수렴 (P 잔류 진동 약간). X 는 시각용 일정 vx=5 m/s 전진.

> JSON 만 만들고 viewer 안 띄우려면 record_gen 명령에 `--no-viewer` 옵션 추가.

Rerun viewer 로 재생:
```bash
uv run python 01_Python_project/release/02_pid/simulator_pid.py 01_Python_project/release/02_pid/01_p_controller/
```

> **시뮬레이터는 챕터 전체용** — 인자 없이 실행하면 `02_pid/` 하위 모든 시나리오를 한 viewer 에 별도 recording 으로 멀티 로드, viewer 좌측 Recordings 패널에서 클릭 전환. `--camera follow|fixed` 로 초기 카메라 (기본 `follow`).

## 합격 기준 (`pytest` 통과)
학생이 푼 알고리즘 형태 (정통 비례 제어 / 다른 방식) 는 제약 X — **behavioral spec** 만 본다.

1. **폐루프 추적 오차** — `y0=1.0, target=0.0, kp=2.0`, 30 초 시뮬, tail 평균 `|error| < 0.05`, peak `|error| < 1.2`

> tail MAE = 정상상태 정확도, peak = 트랜지언트 발산/오버슈트 boundedness. trivial 구현 (`return 0` 등) 두 임계값 모두 초과로 차단.

## 힌트
- 일반 형태: `u = Kp * error`, 여기서 `error = reference − measure`
- 부호 주의: `measure − reference` 순서로 쓰면 양의 피드백이 되어 발산
- P 제어기는 메모리리스 — `__init__` 에서 이전 측정값을 저장할 필요 X

## 게인/파라미터 튜닝 위치

라이브러리 코드 (`.py` 안의 클래스·함수) 는 **시그니처만** 정의 — kp/kd/ki, window_size, R/Q, lookahead 등은 매개변수로만 받는다. 실제 *값* 은 두 곳에서 명시:

- **시각화/실행 (자유롭게 변경 OK, **release 기본값은 모두 0**)**: 같은 폴더의 `record_gen.py` / `demo.py` (시나리오 여럿이면 `record_gen_<scenario>.py`) 안의 게인/파라미터가 0 으로 초기화되어 있음 → **학생이 직접 채워야 응답이 나옴**. 0 인 채로 실행하면 controller 출력 0, 응답 없음 (또는 NaN/division 에러). 값을 바꿔 다시 실행하며 응답 변화 비교.
- **합격 기준 검증 (변경 금지)**: `test_*.py` 안에 박혀 있음. pytest 가 이 값으로 통과 여부를 본다 — 임의로 바꾸면 검증 의미가 사라짐.

즉 "다른 게인은 어떻게 동작하지?" 는 producer 만 바꾸고, "내 구현이 spec 을 통과하는가?" 는 test 그대로 두고 `pytest` 만 돌리면 된다.

## 문제별 추가 제약
(공통 제약은 [`../../README.md`](../../README.md) 의 "AI 도구 사용 가이드" 참조 — 시그니처 유지, `# TODO:` 위 안내문 유지 등)

- **`plant.py` 절대 수정 금지** — 검증 환경(시뮬레이션 plant)의 일부.

# 과제 — Average Filter

## 목표
연속적으로 들어오는 측정값의 누적 평균을 **재귀 형태**로 계산하는 필터를 구현한다.
필터를 거치면 측정 노이즈가 줄어 신호가 안정화되어야 한다.

## 인터페이스 계약
**이 시그니처는 변경하지 마세요.** 채점/테스트가 이 형태에 의존합니다.

```python
class AverageFilter:
    def __init__(self): ...
    def step(self, x: float) -> float
```

- `step(x)` 는 새 측정값 `x` 를 받아 갱신된 누적 평균을 반환
- 내부에 표본 수와 현재 평균만 보관 (전체 입력 배열 저장 금지)

## 구현 위치
`01_Python_project_refactored/release/01_filters/01_average_filter/average_filter.py` 의 `step` 메소드 안 `# TODO:` 블록.
다른 파일/시그니처는 건드리지 않습니다.

## 실행

> 환경 셋업 (1회) 과 명령 실행 규칙은 [`../../README.md`](../../README.md) 참조. **git root 에서 실행.**

테스트 (합격 검증):
```bash
uv run pytest 01_Python_project_refactored/release/01_filters/01_average_filter/ -v
```

데모 (시각 확인, 선택 — 구현 후):
```bash
uv run python 01_Python_project_refactored/release/01_filters/01_average_filter/demo.py
```
→ 기본 브라우저에 N(5, 1) 노이즈 샘플과 추정 평균을 plotly 그래프로 표시.

## 합격 기준 (`pytest` 통과)
학생이 푼 알고리즘 형태 (재귀 누적 / list mean / 다른 방식) 는 제약 X — **behavioral spec** 만 본다.

1. **상수 입력 안정성** — `step(2.0)` 을 100 회 반복 호출 시 출력 `2.0` 유지
2. **노이즈 추적 RMS** — 상수 truth=5 + N(0, 1) 노이즈 1만 표본, warm-up (100 표본) 이후 RMS 오차 `< 0.1`

> RMS = √(bias² + variance). `return 0` (bias 임계값 초과) / `return x` (variance 임계값 초과) 모두 차단.

## 힌트
- 재귀 형태: `다음 평균 = 이전 평균 + (새 측정값 - 이전 평균) / 표본수`
- 표본 수를 갱신하는 위치(분모로 쓰기 전/후)에 주의

## 게인/파라미터 튜닝 위치

라이브러리 코드 (`.py` 안의 클래스·함수) 는 **시그니처만** 정의 — kp/kd/ki, window_size, R/Q, lookahead 등은 매개변수로만 받는다. 실제 *값* 은 두 곳에서 명시:

- **시각화/실행 (자유롭게 변경 OK, **release 기본값은 모두 0**)**: 같은 폴더의 `record_gen.py` / `demo.py` (시나리오 여럿이면 `record_gen_<scenario>.py`) 안의 게인/파라미터가 0 으로 초기화되어 있음 → **학생이 직접 채워야 응답이 나옴**. 0 인 채로 실행하면 controller 출력 0, 응답 없음 (또는 NaN/division 에러). 값을 바꿔 다시 실행하며 응답 변화 비교.
- **합격 기준 검증 (변경 금지)**: `test_*.py` 안에 박혀 있음. pytest 가 이 값으로 통과 여부를 본다 — 임의로 바꾸면 검증 의미가 사라짐.

즉 "다른 게인은 어떻게 동작하지?" 는 producer 만 바꾸고, "내 구현이 spec 을 통과하는가?" 는 test 그대로 두고 `pytest` 만 돌리면 된다.

## 문제별 추가 제약
(공통 제약은 [`../../README.md`](../../README.md) 의 "AI 도구 사용 가이드" 참조 — 시그니처 유지, `# TODO:` 위 안내문 유지 등)

이 문제에 한정한 추가 사항은 없습니다.

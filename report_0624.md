# 작업 보고서 (2026-06-24)

## 배경

Isaac Sim 설치 없이(PC 용량 한계) `assignment2/palletizing_simulator` 과제를 진행하기 위해,
`assignment2/NO_ISAAC_SIM.md` 가이드에 따라 `assignment2/templete code`에서
알고리즘 개발 및 검증 작업을 수행했다. 물리 시뮬레이션 기반의 최종 검증은
추후 Isaac Sim 환경에서 별도로 진행해야 한다.

작업 대상 디렉토리: `assignment2/templete code/`

---

## 1. 베이스라인 현황 파악

`python main.py`로 기존 baseline을 실행한 결과, 박스 250개 중 단 20개만
적재되는 문제를 발견했다.

- 원인: `algorithm.py`의 `Palletizer.run()` 내부에 다음과 같은 코드가 있었음
  ```python
  if len(self.sequence) >= 20:
      self.finished_by_user = True
      break
  ```
  README에는 언급되지 않은 인위적 제한으로, 평가 스펙과 무관한 디버그성
  잔재로 판단되어 제거함.
- 베이스라인 적재율: **12.74% / 17.76%** (box_sequence_0 / box_sequence_1)

---

## 2. `algorithm.py` 알고리즘 개선

기존의 단순 cursor 기반(X 채우기 → row → layer 이동) 배치 로직을 다음과 같이 교체했다.

### 2.1 Extreme-point 기반 배치 탐색
- 박스를 하나 놓을 때마다 그 박스의 우측 모서리 `(x+dx, y)`와 뒤쪽 모서리
  `(x, y+dy)`를 새로운 배치 후보 좌표로 추가.
- 매 스텝마다 모든 후보 좌표 × 회전(0°/90°) 조합을 평가하여, 다음 두 조건을
  만족하는 가장 낮고 앞쪽인(`z`, `y`, `x` 오름차순) 위치를 선택:
  - 팔레트 범위 내 (X/Y/Z 모두)
  - 지지율(support ratio) 70% 이상

### 2.2 버퍼(buffer) 활용
- 기존 코드는 버퍼 안의 박스를 순서대로 확인하다 처음 맞는 것을 즉시 적재했음.
- 개선 후에는 버퍼 안의 모든 박스에 대해 최적 배치를 계산한 뒤, 그중
  **가장 낮고 효율적인 배치를 가진 박스를 먼저 적재**하도록 변경.
- `config/algorithm_config.yaml`의 `buffer.size`를 `0 → 4`로 변경하여
  버퍼 기반 선택이 실제로 동작하도록 함.

### 2.3 충돌/지지 계산 방식 — 격자(grid) 근사에서 정확한 연속 좌표 계산으로 전환
- 처음에는 1cm 격자의 heightmap으로 점유 상태를 근사 관리했으나,
  `physics_checker.py`로 검증하는 과정에서 격자 인덱스 변환 시 좌표가
  최대 격자 폭의 절반(0.5cm)까지 스냅되어 **실제로 겹치는 두 박스가
  "충돌 없음"으로 잘못 판정되는 버그**를 발견했다.
- 이를 보수적(floor/ceil)으로 고쳐보았으나, 격자 해상도(1cm)가 박스 크기
  대비 작지 않아 박스마다 최대 2cm씩 여유 공간이 부풀려지며 적재율이
  오히려 0.7~15%까지 급락하는 부작용이 발생했다.
- 최종적으로 격자 근사를 완전히 제거하고, 적재된 박스들의 **정확한
  AABB(바운딩 박스)를 numpy 배열로 유지**하면서 XY 겹침 영역으로부터
  바로 높이(z)와 지지율을 계산하는 방식으로 교체했다. 이를 통해 격자
  해상도에 따른 오차 없이 정확한 충돌/지지 판정이 가능해졌다.

### 2.4 결과
| 파일 | 적재 박스 수 | 적재율 |
|---|---|---|
| box_sequence_0 | 95 / 250 | **66.47%** |
| box_sequence_1 | 68 / 250 | **54.17%** |

(베이스라인 12.74% / 17.76% 대비 큰 개선)

---

## 3. `validator.py` 작성

`algorithm_results/*.json` 출력이 README에 정의된 스키마를 만족하는지
검사하는 스크립트를 작성했다.

검사 항목:
- 최상위 필수 필드: `buffer_size`, `sequence`, `terminated`, `terminated_step`, `finished_by_user`
- 각 박스 항목의 필수 필드: `step`, `id`, `size`, `mass`, `position`, `rotation`
- 타입 검사 (`size`/`position`은 길이 3의 숫자 리스트 등)
- `rotation`이 `0` 또는 `90`인지
- `id` 중복 여부

실행 결과: 두 파일 모두 스키마 검증 통과.

```
python validator.py
```

---

## 4. `physics_checker.py` 작성

Isaac Sim 없이도 명백한 물리적 문제를 걸러내기 위한 근사("physics-less")
검사 스크립트를 작성했다.

검사 항목:
1. **팔레트 영역 이탈** — 각 박스의 AABB가 팔레트 길이/폭/높이를 벗어나는지
2. **박스 간 충돌** — 모든 박스 쌍에 대해 AABB 겹침 여부 (얇은 면 접촉은
   출력 JSON의 소수점 3자리 반올림으로 인한 노이즈이므로 1.5mm 허용 오차로 구분)
3. **지지 안정성** — 박스 바닥면을 10×10 격자로 샘플링하여, 자신보다 먼저
   적재된 박스들의 윗면(또는 바닥) 중 자신의 바닥 높이와 맞닿은 비율이
   70% 미만이면 불안정 적재로 표시

실행 결과: 두 파일 모두 충돌/지지 문제 없음 (`[OK]`).

```
python physics_checker.py
```

---

## 5. 변경된 파일 목록

| 파일 | 변경 내용 |
|---|---|
| `templete code/algorithm.py` | 적재 알고리즘 전면 재작성 (extreme-point + 정확한 AABB 기반 충돌/지지 계산, 버퍼 기반 박스 선택) |
| `templete code/config/algorithm_config.yaml` | `buffer.size`를 0 → 4로 변경 |
| `templete code/validator.py` | 신규 작성 — 출력 스키마 검증기 |
| `templete code/physics_checker.py` | 신규 작성 — 충돌/지지 근사 검사기 |

`BoxInput`, `PlacedBox`, `RunResult`, `PalletConfig`, `run()` 함수 시그니처 등
수정 금지 항목은 변경하지 않았다.

---

## 6. 다음 단계 (TODO)

- `NO_ISAAC_SIM.md`의 6~7단계에 따라 `algorithm_results/`와 사용한
  `config/algorithm_config.yaml`을 함께 패키징
- Isaac Sim이 설치된 환경(연구실 서버/클라우드/동료 PC)에서 실제 물리
  시뮬레이션으로 최종 검증 (특히 지지율 임계값 70%가 실제 물리 환경에서도
  안정적인지 확인 필요)
- 필요 시 `_SUPPORT_THRESHOLD`, 후보 좌표 탐색 전략(예: 더 다양한 extreme
  point, skyline 등) 추가 튜닝

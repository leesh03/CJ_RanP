## Isaac Sim 없이 개발/검증하기 (가이드)

목적
- Isaac Sim 설치 없이도 알고리즘을 개발하고 제출 준비를 진행할 수 있도록 단계별 워크플로와 실용적인 대안을 정리합니다. 이 문서는 다른 학생이나 자동화된 도구(또는 AI)에게도 쉽게 제시할 수 있도록 작성되었습니다.

전제/가정
- 레포에 포함된 `templete code`(알고리즘 실행기)와 `algorithm_results` 출력 포맷을 사용할 수 있음.
- 최종 평가나 물리 기반 검증(충돌/물리 시뮬레이션)은 Isaac Sim(또는 동등한 환경)에서 수행되어야 함.

결론(요약)
- 전체 기능 검증(물리 시뮬레이션, 시각화, 정확한 물리 반응)은 Isaac Sim이 필요합니다.
- 하지만 알고리즘 구현, 결과 파일 생성, 형식 검증, 기본 논리/충돌 가능성 검사 등은 로컬에서 충분히 진행할 수 있습니다.

권장 워크플로 (순서별)
1. 목표 정의 및 입력/출력 명세 확인
   - `box_sequence` 형식과 `algorithm_results`에 어떤 정보(박스 위치/회전 등)를 써야 하는지 파악합니다.
   - 레포의 `templete code`와 `README.md`에서 요구되는 파일/폴더 구조를 확인하세요.

2. 알고리즘 개발: 시뮬레이터 없이 알고리즘 로직 작성
   - `templete code`의 `algorithm.py`을 사용해 로컬에서 작동하는 버전을 만듭니다.
   - 물리 시뮬레이션 없이도 배치 결정(각 박스의 목표 위치/방향, 스택 순서)을 출력하도록 구현합니다.
   - 입력은 `box_sequence/*.json`, 출력은 `algorithm_results/*.json` 형식을 따르세요.

3. 출력 형식/내용 자동 검증기 작성
   - 결과 JSON이 요구 스키마(필수 필드, 타입)를 만족하는지 확인하는 스크립트를 만듭니다.
   - 예: `validator.py`는 각 박스에 `id`, `position`, `orientation`, `placed_time` 같은 기본 필드를 검사합니다.

4. 물리적 충돌 가능성(간단 검사)
   - 정확한 물리 시뮬레이션 없이 근사 검사만 수행합니다:
     - 박스들의 바운딩 박스(또는 원) 간 중첩 검사로 명백한 충돌을 탐지.
     - 스택 높이/중심 범위 등을 이용해 불안정 배치 의심을 표기.
   - 이 단계는 `physics-less checker`로 구현 가능합니다.

5. 단위/통합 테스트 추가
   - 대표적인 `box_sequence` 케이스(빈 공간, 꽉 찬 공간, 다양한 크기)를 모의 입력으로 테스트 케이스를 만듭니다.
   - CI나 로컬 테스트로 알고리즘의 안정성(예외 처리 등)을 확인하세요.

6. 결과 패키징 및 제출 준비
   - 평가자가 Isaac Sim에서 결과를 재현할 수 있도록 `algorithm_results`와 사용한 `config`(예: `algorithm_config.yaml`)를 함께 제공합니다.
   - README에 실행 방법, 의존성, 출력 포맷 설명을 추가하세요.

7. 원격/클라우드에서 최종 검증(가능하면)
   - Isaac Sim이 설치된 원격 머신(연구실 서버, 동료 PC, 클라우드 인스턴스)에 결과를 옮겨 물리 시뮬레이션으로 최종 검증합니다.

디스크 공간이 부족할 때 가능한 전략
- 외장 드라이브에 Isaac Sim을 설치하고 심볼릭 링크 사용(Windows 예: `mklink /D <link> <target>`)
- 임시로 로컬에서 Isaac Sim을 설치하지 않고 개발은 위 워크플로로 진행한 뒤, 원격에서 한 번만 전체 시뮬레이션 실행
- 불필요한 대용량 파일 정리: Conda envs, Docker 이미지, 다운로드 캐시 등

간단한 예시 명령(Windows PowerShell)
```powershell
# templete code 실행(예시)
python "templete code\main.py" --input "templete code\box_sequence\box_sequence_0.json" --output "algorithm_results/result_0.json"

# 심볼릭 링크 예시(관리자 권한 필요)
mklink /D C:\isaac-sim D:\isaac-sim
```

권장 스크립트/도구 목록 (간단 템플릿)
- `validator.py`: 출력 JSON 스키마 검사
- `physics_checker.py`: 축약된 충돌/스택 안정성 검사
- `run_local.sh` / `run_local.ps1`: 알고리즘 실행 + 검증을 연속 실행하는 스크립트

AI에 전달할 때의 권장 패키지(간단 안내)
- 필요한 파일: `algorithm_results/*.json`, 사용한 `box_sequence/*.json`, `algorithm_config.yaml`, `templete code` 실행 명령
- 설명서: 간단한 실행 명령과 기대 출력 예시(샘플 JSON)를 함께 첨부

추가 제안
- 원하면 제가 `validator.py`와 `physics_checker.py`의 간단한 템플릿을 작성해 드리겠습니다.
- 또한 이 문서를 영어 버전으로 변환하거나, 자동화된 CI 실행 스크립트로 확장할 수 있습니다.

문의사항이나 원하는 템플릿(예: `validator.py`)을 지정해 주세요. 기꺼이 바로 만들어 드립니다.

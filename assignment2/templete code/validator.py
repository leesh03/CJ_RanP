"""
algorithm_results/*.json 출력 형식을 README 스펙대로 검사하는 스크립트.

Isaac Sim 없이도 결과 파일이 평가 시스템이 기대하는 스키마를 만족하는지
빠르게 확인할 수 있다 (NO_ISAAC_SIM.md 3단계).

사용법:
    python validator.py
    python validator.py --dir algorithm_results
    python validator.py --file algorithm_results/box_sequence_0.json
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Dict, List


REQUIRED_TOP_KEYS = ("buffer_size", "sequence", "terminated", "terminated_step", "finished_by_user")
REQUIRED_BOX_KEYS = ("step", "id", "size", "mass", "position", "rotation")
ALLOWED_ROTATIONS = (0, 90)


def validate_box(box: Dict, index: int) -> List[str]:
    errors = []

    for key in REQUIRED_BOX_KEYS:
        if key not in box:
            errors.append(f"sequence[{index}]: 필수 필드 누락 '{key}'")

    if errors:
        return errors

    if not isinstance(box["step"], int):
        errors.append(f"sequence[{index}]: 'step'은 int여야 함")

    if not isinstance(box["id"], int):
        errors.append(f"sequence[{index}]: 'id'는 int여야 함")

    size = box["size"]
    if not isinstance(size, list) or len(size) != 3 or not all(isinstance(v, (int, float)) for v in size):
        errors.append(f"sequence[{index}]: 'size'는 길이 3의 숫자 리스트여야 함")

    if not isinstance(box["mass"], (int, float)):
        errors.append(f"sequence[{index}]: 'mass'는 숫자여야 함")

    position = box["position"]
    if not isinstance(position, list) or len(position) != 3 or not all(isinstance(v, (int, float)) for v in position):
        errors.append(f"sequence[{index}]: 'position'는 길이 3의 숫자 리스트여야 함")

    rotation = box["rotation"]
    if rotation not in ALLOWED_ROTATIONS:
        errors.append(f"sequence[{index}]: 'rotation'은 {ALLOWED_ROTATIONS} 중 하나여야 함 (받은 값: {rotation})")

    return errors


def validate_result(data: Dict) -> List[str]:
    errors = []

    for key in REQUIRED_TOP_KEYS:
        if key not in data:
            errors.append(f"최상위 필수 필드 누락: '{key}'")

    if errors:
        return errors

    if not isinstance(data["buffer_size"], int):
        errors.append("'buffer_size'는 int여야 함")

    if not isinstance(data["sequence"], list):
        errors.append("'sequence'는 list여야 함")
    else:
        seen_ids = set()
        for i, box in enumerate(data["sequence"]):
            errors.extend(validate_box(box, i))

            box_id = box.get("id")
            if box_id in seen_ids:
                errors.append(f"sequence[{i}]: 중복된 id={box_id}")
            seen_ids.add(box_id)

    if not isinstance(data["terminated"], bool):
        errors.append("'terminated'는 bool이어야 함")

    if data["terminated_step"] is not None and not isinstance(data["terminated_step"], int):
        errors.append("'terminated_step'는 int 또는 null이어야 함")

    if not isinstance(data["finished_by_user"], bool):
        errors.append("'finished_by_user'는 bool이어야 함")

    return errors


def validate_file(path: Path) -> List[str]:
    try:
        with path.open("r", encoding="utf-8") as f:
            data = json.load(f)
    except json.JSONDecodeError as e:
        return [f"JSON 파싱 실패: {e}"]

    return validate_result(data)


def main() -> int:
    parser = argparse.ArgumentParser(description="algorithm_results 출력 스키마 검증기")
    parser.add_argument("--dir", default="algorithm_results", help="결과 디렉토리 (기본: algorithm_results)")
    parser.add_argument("--file", default=None, help="단일 결과 파일 경로")
    args = parser.parse_args()

    here = Path(__file__).resolve().parent

    if args.file:
        files = [Path(args.file)]
    else:
        result_dir = here / args.dir
        files = sorted(p for p in result_dir.glob("*.json"))

    if not files:
        print("[ERROR] 검사할 결과 파일을 찾지 못했습니다.")
        return 1

    total_errors = 0

    for path in files:
        errors = validate_file(path)

        if errors:
            print(f"[FAIL] {path.name}: {len(errors)}개 오류")
            for e in errors:
                print(f"    - {e}")
        else:
            print(f"[OK]   {path.name}: 스키마 검증 통과")

        total_errors += len(errors)

    print()
    print(f"[SUMMARY] 검사 파일 {len(files)}개, 오류 {total_errors}개")

    return 1 if total_errors > 0 else 0


if __name__ == "__main__":
    sys.exit(main())

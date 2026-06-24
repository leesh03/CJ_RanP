"""
Isaac Sim 없이 algorithm_results/*.json 의 적재 결과를 근사적으로 검사하는
"physics-less checker" (NO_ISAAC_SIM.md 4단계).

실제 물리 시뮬레이션 없이 다음을 확인한다:
  1. 팔레트 영역(bounds) 이탈 여부
  2. 박스끼리의 AABB(바운딩 박스) 충돌 여부
  3. 박스 바닥면의 지지율(support ratio) - 바닥/하위 박스 위에 충분히
     얹혀 있는지 (얹힌 비율이 낮으면 불안정 적재로 표시)

이 검사는 실제 물리 엔진을 대체하지 못하며, 명백한 문제를 빠르게
걸러내기 위한 근사 검사임을 유의해야 한다. 최종 검증은 Isaac Sim에서
수행해야 한다.

사용법:
    python physics_checker.py
    python physics_checker.py --dir algorithm_results --config config/algorithm_config.yaml
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Dict, List, Tuple

import yaml

EPS = 0.001                 # 출력 JSON이 소수점 3자리로 저장되어 생기는 반올림 오차 허용치
COLLISION_TOL = 0.0015      # 면 접촉(터치)은 충돌로 보지 않기 위한 허용 오차
SUPPORT_EPS = 0.003         # 지지면으로 인정할 높이 오차
SUPPORT_THRESHOLD = 0.70    # 바닥면 지지 비율 기준
GRID_N = 10                 # 지지율 추정을 위한 footprint 내부 샘플 그리드 한 변

AABB = Tuple[float, float, float, float, float, float]


def box_aabb(box: Dict) -> AABB:
    cx, cy, cz = box["position"]
    dx, dy, dz = box["size"]
    return (
        cx - dx / 2.0, cy - dy / 2.0, cz - dz / 2.0,
        cx + dx / 2.0, cy + dy / 2.0, cz + dz / 2.0,
    )


def overlap_1d(a0: float, a1: float, b0: float, b1: float) -> float:
    return max(0.0, min(a1, b1) - max(a0, b0))


def check_bounds(boxes: List[Dict], pallet: Dict) -> List[str]:
    errors = []
    length, width, height = pallet["length"], pallet["width"], pallet["height"]

    for box in boxes:
        x0, y0, z0, x1, y1, z1 = box_aabb(box)

        if x0 < -EPS or x1 > length + EPS:
            errors.append(f"id={box['id']}: X 범위 이탈 [{x0:.4f}, {x1:.4f}] (팔레트 길이 {length})")

        if y0 < -EPS or y1 > width + EPS:
            errors.append(f"id={box['id']}: Y 범위 이탈 [{y0:.4f}, {y1:.4f}] (팔레트 폭 {width})")

        if z0 < -EPS or z1 > height + EPS:
            errors.append(f"id={box['id']}: Z 범위 이탈 [{z0:.4f}, {z1:.4f}] (팔레트 높이 {height})")

    return errors


def check_collisions(boxes: List[Dict]) -> List[str]:
    errors = []
    aabbs = [box_aabb(b) for b in boxes]

    for i in range(len(boxes)):
        for j in range(i + 1, len(boxes)):
            a, b = aabbs[i], aabbs[j]

            ox = overlap_1d(a[0], a[3], b[0], b[3])
            oy = overlap_1d(a[1], a[4], b[1], b[4])
            oz = overlap_1d(a[2], a[5], b[2], b[5])

            if ox > COLLISION_TOL and oy > COLLISION_TOL and oz > COLLISION_TOL:
                errors.append(
                    f"id={boxes[i]['id']} <-> id={boxes[j]['id']}: 충돌 (겹침 {ox:.4f}m x {oy:.4f}m x {oz:.4f}m)"
                )

    return errors


def check_support(boxes: List[Dict]) -> List[str]:
    """
    각 박스보다 먼저 적재된(앞선 인덱스) 박스들만 지지대 후보로 본다.
    footprint를 GRID_N x GRID_N 격자로 나눠 표본점마다 아래에서 가장 높은
    표면(바닥 또는 하위 박스 top)을 찾고, 박스 바닥(z0)과 같은 높이로
    지지되는 표본 비율을 계산한다.
    """
    errors = []
    aabbs = [box_aabb(b) for b in boxes]

    for i, box in enumerate(boxes):
        x0, y0, z0, x1, y1, _ = aabbs[i]

        if z0 <= SUPPORT_EPS:
            continue  # 바닥에 닿아 있음

        dx = x1 - x0
        dy = y1 - y0

        supported = 0
        total = GRID_N * GRID_N

        for gi in range(GRID_N):
            for gj in range(GRID_N):
                px = x0 + dx * (gi + 0.5) / GRID_N
                py = y0 + dy * (gj + 0.5) / GRID_N

                top_below = 0.0
                for k in range(i):
                    bx0, by0, _, bx1, by1, bz1 = aabbs[k]

                    if bx0 - EPS <= px <= bx1 + EPS and by0 - EPS <= py <= by1 + EPS:
                        top_below = max(top_below, bz1)

                if abs(top_below - z0) <= SUPPORT_EPS:
                    supported += 1

        ratio = supported / total

        if ratio < SUPPORT_THRESHOLD - EPS:
            errors.append(
                f"id={box['id']}: 지지율 부족 ({ratio * 100:.1f}% < {SUPPORT_THRESHOLD * 100:.0f}%), "
                f"불안정 적재 의심 (z={z0:.3f})"
            )

    return errors


def check_result(data: Dict, pallet: Dict) -> List[str]:
    boxes = data.get("sequence", [])

    errors: List[str] = []
    errors.extend(check_bounds(boxes, pallet))
    errors.extend(check_collisions(boxes))
    errors.extend(check_support(boxes))

    return errors


def load_pallet_config(config_path: Path) -> Dict:
    with config_path.open("r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f)

    return cfg["pallet"]


def main() -> int:
    parser = argparse.ArgumentParser(description="물리 시뮬레이션 없이 충돌/지지 안정성을 근사 검사")
    parser.add_argument("--dir", default="algorithm_results", help="결과 디렉토리 (기본: algorithm_results)")
    parser.add_argument("--file", default=None, help="단일 결과 파일 경로")
    parser.add_argument("--config", default="config/algorithm_config.yaml", help="팔레트 크기를 읽어올 설정 파일")
    args = parser.parse_args()

    here = Path(__file__).resolve().parent
    pallet = load_pallet_config(here / args.config)

    if args.file:
        files = [Path(args.file)]
    else:
        files = sorted((here / args.dir).glob("*.json"))

    if not files:
        print("[ERROR] 검사할 결과 파일을 찾지 못했습니다.")
        return 1

    total_errors = 0

    for path in files:
        with path.open("r", encoding="utf-8") as f:
            data = json.load(f)

        errors = check_result(data, pallet)

        if errors:
            print(f"[FAIL] {path.name}: {len(errors)}개 문제")
            for e in errors:
                print(f"    - {e}")
        else:
            print(f"[OK]   {path.name}: 충돌/지지 근사 검사 통과 (박스 {len(data.get('sequence', []))}개)")

        total_errors += len(errors)

    print()
    print(f"[SUMMARY] 검사 파일 {len(files)}개, 문제 {total_errors}개")

    return 1 if total_errors > 0 else 0


if __name__ == "__main__":
    sys.exit(main())

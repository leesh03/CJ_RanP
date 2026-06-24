from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Tuple, TypedDict

import numpy as np

from buffer_manager import BufferManager


# ---------------------------------------------------------------------------
# 입출력 스키마  (수정 금지)
# ---------------------------------------------------------------------------

class BoxInput(TypedDict):
    step: int
    id: int
    size: List[float]   # [length, width, height]
    mass: float


class PlacedBox(TypedDict):
    step: int
    id: int
    size: List[float]
    mass: float
    position: List[float]
    rotation: int       # 0 또는 90


class RunResult(TypedDict):
    buffer_size: int
    sequence: List[PlacedBox]

    # 더 이상 적재 가능한 박스를 찾지 못해 자동 종료된 경우 True
    terminated: bool
    terminated_step: Optional[int]

    # 참가자 알고리즘이 명시적으로 적재 종료를 선언한 경우 True
    finished_by_user: bool


# ---------------------------------------------------------------------------
# 설정 dataclass  (수정 금지)
# ---------------------------------------------------------------------------

@dataclass
class PalletConfig:
    length: float
    width: float
    height: float


# ---------------------------------------------------------------------------
# 참가자 개발 영역
# ---------------------------------------------------------------------------

@dataclass
class AlgorithmConfig:
    allow_rotation: bool
    buffer_size: int


_SUPPORT_THRESHOLD = 0.70   # 박스 바닥면이 이 비율 이상 지지되어야 안정적으로 간주
_EPS = 1e-6


@dataclass
class _Placement:
    score: Tuple[float, float, float]   # (z, y, x) - 작을수록 우선
    x: float
    y: float
    z: float
    dims: Tuple[float, float, float]
    rotation: int


class Palletizer:
    """
    Extreme-point 기반 팔레타이저.

    동작 방식:
      - 이미 적재된 박스들의 정확한(연속값) AABB를 그대로 유지한다 (격자 근사 없음).
      - 박스를 내려놓을 수 있는 (x, y) 후보 지점을 extreme-point 방식으로 관리한다
        (각 박스를 놓은 뒤 그 박스의 우측/뒤쪽 모서리를 새 후보로 추가).
      - 각 후보 지점에서 그 footprint와 XY가 겹치는 기존 박스들의 top z 중
        최댓값을 "내려놓일 높이(z)"로 삼고, 그 높이에 실제로 맞닿아 있는
        면적의 비율을 지지율(support ratio)로 계산한다.
      - buffer 안의 박스들 중에서 (z, y, x)가 가장 작은(가장 낮고 앞쪽인) 배치를
        선택해 적재한다. 즉 buffer_size > 1이면 여러 박스를 미리 보고
        그중 가장 효율적으로 놓을 수 있는 박스를 먼저 적재한다.
      - 적재 가능한 박스가 하나도 없으면 자동 종료한다.
    """

    def __init__(self, pallet_cfg: PalletConfig, algo_cfg: AlgorithmConfig) -> None:
        self.pallet = pallet_cfg
        self.algo = algo_cfg
        self._reset_state()

    def _reset_state(self) -> None:
        # 적재된 박스들의 정확한 AABB: 각 행 = (x0, y0, z0, x1, y1, z1)
        self._placed_bounds = np.empty((0, 6), dtype=np.float64)

        # extreme-point 후보 (x, y) 집합. 시작은 원점뿐이다.
        self._candidates = {(0.0, 0.0)}

        self.sequence: List[PlacedBox] = []

        self.finished = False
        self.terminated_step: Optional[int] = None
        self.finished_by_user = False

    # -----------------------------------------------------------------------
    # 참가자 수정 가능 함수
    # -----------------------------------------------------------------------

    def should_finish(self, current_buffer: List[BoxInput]) -> bool:
        """
        [참가자 수정 가능]
        현재 버퍼 상태를 보고 적재를 명시적으로 종료할지 결정한다.
        기본 구현은 항상 계속 진행한다 (False 반환).
        """
        return False

    # -----------------------------------------------------------------------
    # 충돌/지지 조회 (격자 근사 없이 연속 좌표로 직접 계산)
    # -----------------------------------------------------------------------

    def _query(self, x: float, y: float, dx: float, dy: float) -> Tuple[float, float]:
        """
        주어진 footprint(x, y, dx, dy)에서 (내려놓일 높이 z, 지지율)을 반환한다.

        z는 이 footprint와 XY가 겹치는 기존 박스들의 top z 중 최댓값이다
        (겹치는 박스가 없으면 바닥인 0). 지지율은 그 높이 z에 실제로 맞닿은
        면적 비율이다 (z보다 낮은 곳에서 멈춰 있는 박스 위 영역은 들뜬 공간이므로
        지지로 인정하지 않는다).
        """
        arr = self._placed_bounds

        if arr.shape[0] == 0:
            return 0.0, 1.0

        ox = np.minimum(arr[:, 3], x + dx) - np.maximum(arr[:, 0], x)
        oy = np.minimum(arr[:, 4], y + dy) - np.maximum(arr[:, 1], y)

        overlap_mask = (ox > _EPS) & (oy > _EPS)

        if not overlap_mask.any():
            return 0.0, 1.0

        z = float(arr[overlap_mask, 5].max())

        support_mask = overlap_mask & (np.abs(arr[:, 5] - z) <= _EPS)
        support_area = float(np.sum(ox[support_mask] * oy[support_mask]))

        footprint_area = dx * dy
        support_ratio = min(1.0, support_area / footprint_area) if footprint_area > _EPS else 1.0

        return z, support_ratio

    def _stamp(self, x: float, y: float, z: float, dx: float, dy: float, dz: float) -> None:
        row = np.array([[x, y, z, x + dx, y + dy, z + dz]], dtype=np.float64)
        self._placed_bounds = np.vstack([self._placed_bounds, row])

    def _add_candidates(self, x: float, y: float, dx: float, dy: float) -> None:
        self._candidates.add((round(x + dx, 4), round(y, 4)))
        self._candidates.add((round(x, 4), round(y + dy, 4)))

    # -----------------------------------------------------------------------
    # 배치 탐색
    # -----------------------------------------------------------------------

    def _candidate_orientations(
        self,
        size: List[float],
    ) -> List[Tuple[Tuple[float, float, float], int]]:
        sx, sy, sz = float(size[0]), float(size[1]), float(size[2])

        if not self.algo.allow_rotation:
            return [((sx, sy, sz), 0)]

        return [
            ((sx, sy, sz), 0),
            ((sy, sx, sz), 90),
        ]

    def _best_placement_for_box(self, box: BoxInput) -> Optional[_Placement]:
        best: Optional[_Placement] = None

        for dims, rotation in self._candidate_orientations(box["size"]):
            dx, dy, dz = dims

            if dx > self.pallet.length + _EPS or dy > self.pallet.width + _EPS:
                continue

            for (cx, cy) in self._candidates:
                if cx + dx > self.pallet.length + _EPS:
                    continue

                if cy + dy > self.pallet.width + _EPS:
                    continue

                z, support_ratio = self._query(cx, cy, dx, dy)

                if z + dz > self.pallet.height + _EPS:
                    continue

                if support_ratio < _SUPPORT_THRESHOLD - _EPS:
                    continue

                score = (round(z, 4), round(cy, 4), round(cx, 4))

                if best is None or score < best.score:
                    best = _Placement(
                        score=score,
                        x=cx,
                        y=cy,
                        z=z,
                        dims=dims,
                        rotation=rotation,
                    )

        return best

    # -----------------------------------------------------------------------
    # 적재 적용
    # -----------------------------------------------------------------------

    def _append_placed(self, box: BoxInput, placement: _Placement) -> None:
        dx, dy, dz = placement.dims
        x, y, z = placement.x, placement.y, placement.z

        self.sequence.append({
            "step": int(box["step"]),
            "id": int(box["id"]),
            "size": [
                round(dx, 3),
                round(dy, 3),
                round(dz, 3),
            ],
            "mass": float(box["mass"]),
            "position": [
                round(x + dx / 2.0, 3),
                round(y + dy / 2.0, 3),
                round(z + dz / 2.0, 3),
            ],
            "rotation": int(placement.rotation),
        })

        self._stamp(x, y, z, dx, dy, dz)
        self._add_candidates(x, y, dx, dy)

    def run(self, boxes: List[BoxInput]) -> RunResult:
        self._reset_state()

        buf = BufferManager(self.algo.buffer_size)
        buf.reset(boxes)

        while buf.has_pending():
            if self.algo.buffer_size == 0:
                current = [buf.peek_next()]
            else:
                current = buf.get_buffer()

            if self.should_finish(current):
                self.finished_by_user = True
                break

            best_index: Optional[int] = None
            best_placement: Optional[_Placement] = None

            for idx, box in enumerate(current):
                placement = self._best_placement_for_box(box)

                if placement is None:
                    continue

                if best_placement is None or placement.score < best_placement.score:
                    best_placement = placement
                    best_index = idx

            if best_placement is None or best_index is None:
                self.finished = True

                if current:
                    self.terminated_step = int(current[0]["step"])

                break

            box = current[best_index]
            self._append_placed(box, best_placement)

            if self.algo.buffer_size == 0:
                buf.pop_next()
            else:
                buf.pop_selected(best_index)

        return {
            "buffer_size": self.algo.buffer_size,
            "sequence": self.sequence,
            "terminated": self.finished,
            "terminated_step": self.terminated_step,
            "finished_by_user": self.finished_by_user,
        }

#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Any, Callable

from ..type.models import PathSeedCandidate, JointPath
from ..tools.metrics import validate_joint_path


DecodeFn = Callable[[str, list[float], list[float]], JointPath]
PlanFn = Callable[[JointPath, list[float], list[float]], tuple[JointPath, float]]


@dataclass
class PathSeedPlannerOptions:
    use_path_seed_client: bool = False
    use_fallback_decode: bool = True
    use_identity_plan: bool = True


class PathSeedPlanner:
    """
    plannerは選ばない．

    役割は以下だけ．
    1．候補pathseedをdecodeして decoded_path を作る
    2．decoded_pathをplan，または補正して modified_path を作る
    3．stomp_time_sec，またはplan_time_secをcandidateに書き込む

    最終的な選択は selector が evaluator を使って行う．
    """

    def __init__(
        self,
        node: Any | None = None,
        decode_fn: DecodeFn | None = None,
        plan_fn: PlanFn | None = None,
        options: PathSeedPlannerOptions | None = None,
    ) -> None:
        self.node = node
        self.decode_fn = decode_fn
        self.plan_fn = plan_fn
        self.options = options or PathSeedPlannerOptions()

        self._path_seed_client = None
        if self.options.use_path_seed_client:
            self._path_seed_client = self._create_path_seed_client()

    def run(
        self,
        candidates: list[PathSeedCandidate],
        start_joints: list[float],
        goal_joints: list[float],
    ) -> list[PathSeedCandidate]:
        planned_candidates: list[PathSeedCandidate] = []

        for candidate in candidates:
            try:
                planned = self.run_candidate(
                    candidate=candidate,
                    start_joints=start_joints,
                    goal_joints=goal_joints,
                )
                planned_candidates.append(planned)

            except Exception as e:
                self._warn(
                    f"[PathSeedPlanner] candidate failed, seed_id={candidate.record.seed_id}, error={e}"
                )

        return planned_candidates

    def run_candidate(
        self,
        candidate: PathSeedCandidate,
        start_joints: list[float],
        goal_joints: list[float],
    ) -> PathSeedCandidate:
        decoded_path = self.decode_candidate(
            candidate=candidate,
            start_joints=start_joints,
            goal_joints=goal_joints,
        )

        modified_path, plan_time_sec = self.plan_candidate(
            decoded_path=decoded_path,
            start_joints=start_joints,
            goal_joints=goal_joints,
        )

        candidate.decoded_path = decoded_path
        candidate.modified_path = modified_path
        candidate.stomp_time_sec = plan_time_sec

        return candidate

    def decode_candidate(
        self,
        candidate: PathSeedCandidate,
        start_joints: list[float],
        goal_joints: list[float],
    ) -> JointPath:
        pathseed_path = str(candidate.absolute_path)

        if self.decode_fn is not None:
            decoded = self.decode_fn(pathseed_path, start_joints, goal_joints)
            validate_joint_path(decoded)
            return decoded

        if self._path_seed_client is not None:
            decoded = self._decode_with_path_seed_client(
                pathseed_path=pathseed_path,
                start_joints=start_joints,
                goal_joints=goal_joints,
            )
            validate_joint_path(decoded)
            return decoded

        if self.options.use_fallback_decode:
            decoded = self._make_linear_demo_path(
                start_joints=start_joints,
                goal_joints=goal_joints,
            )
            validate_joint_path(decoded)
            return decoded

        raise RuntimeError("No decode method is available.")

    def plan_candidate(
        self,
        decoded_path: JointPath,
        start_joints: list[float],
        goal_joints: list[float],
    ) -> tuple[JointPath, float]:
        if self.plan_fn is not None:
            modified_path, plan_time_sec = self.plan_fn(
                decoded_path,
                start_joints,
                goal_joints,
            )
            validate_joint_path(modified_path)
            return modified_path, float(plan_time_sec)

        if self.options.use_identity_plan:
            t0 = time.time()
            modified_path = [list(q) for q in decoded_path]
            plan_time_sec = time.time() - t0
            validate_joint_path(modified_path)
            return modified_path, plan_time_sec

        raise RuntimeError("No plan method is available.")

    def _create_path_seed_client(self) -> Any | None:
        try:
            from path_reuse_method.path_seed_client import PathSeedClient

            return PathSeedClient()

        except Exception as e:
            self._warn(f"[PathSeedPlanner] PathSeedClient is not available, error={e}")
            return None

    def _decode_with_path_seed_client(
        self,
        pathseed_path: str,
        start_joints: list[float],
        goal_joints: list[float],
    ) -> JointPath:
        decoded = self._path_seed_client.send_decode_path_seed(
            pathseed_path,
            start_joints,
            goal_joints,
        )

        return self._normalize_decoded_path(decoded)

    def _normalize_decoded_path(self, decoded: Any) -> JointPath:
        if decoded is None:
            raise RuntimeError("decoded path is None.")

        if isinstance(decoded, list):
            if not decoded:
                raise RuntimeError("decoded path is empty.")

            if isinstance(decoded[0], list):
                return [[float(x) for x in q] for q in decoded]

            return [[float(x) for x in decoded]]

        rows = getattr(decoded, "rows", None)
        cols = getattr(decoded, "cols", None)
        data = getattr(decoded, "data", None)

        if rows is not None and cols is not None and data is not None:
            rows = int(rows)
            cols = int(cols)
            values = [float(x) for x in list(data)]

            if rows <= 0 or cols <= 0:
                raise RuntimeError(f"decoded path has invalid shape, rows={rows}, cols={cols}")

            if len(values) != rows * cols:
                raise RuntimeError(
                    f"decoded path data size mismatch, len={len(values)}, rows={rows}, cols={cols}"
                )

            return [
                values[i * cols : (i + 1) * cols]
                for i in range(rows)
            ]

        raise RuntimeError(f"unsupported decoded path type: {type(decoded)}")

    def _make_linear_demo_path(
        self,
        start_joints: list[float],
        goal_joints: list[float],
    ) -> JointPath:
        mid = [
            0.5 * (s + g)
            for s, g in zip(start_joints, goal_joints)
        ]

        return [
            list(start_joints),
            mid,
            list(goal_joints),
        ]

    def _warn(self, msg: str) -> None:
        if self.node is not None:
            try:
                self.node.get_logger().warn(msg)
                return
            except Exception:
                pass
        print(msg)

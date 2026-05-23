#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import annotations

import math
from pathlib import Path

from ..type.models import PathSeedCandidate, JointPath
from .registry import PathSeedRegistry
from .evaluator import PathSeedEvaluator, EvaluationWeights
from .validator import PathSeedValidator


class PathSeedSelector:
    """パスシードを選択する"""
    def __init__(
        self,
        registry_path: str | Path,
        library_root: str | Path,
        weights: EvaluationWeights | None = None,
    ) -> None:
        self.registry = PathSeedRegistry(
            registry_path=registry_path,
            library_root=library_root,
        )
        self.evaluator = PathSeedEvaluator(weights=weights)
        self.validator = PathSeedValidator()

    def build_candidates(
        self,
        environment_id: str,
        skill_name: str | None = None,
    ) -> list[PathSeedCandidate]:
        candidates: list[PathSeedCandidate] = []

        for record in self.registry.find_records(
            environment_id=environment_id,
            skill_name=skill_name,
        ):
            absolute_path = self.registry.resolve_path(record)
            if not self.validator.validate_file_exists(absolute_path):
                continue

            candidates.append(
                PathSeedCandidate(
                    record=record,
                    absolute_path=absolute_path,
                )
            )

        return candidates

    def evaluate_and_select(
        self,
        candidates: list[PathSeedCandidate],
        top_k: int = 3,
    ) -> dict[str, Any]:
        ready_candidates = [
            c for c in candidates if self.validator.validate_candidate_ready(c)
        ]

        evaluations = self.evaluator.evaluate(ready_candidates)

        return {
            "selected_seed": evaluations[0].__dict__ if evaluations else None,
            "evaluations": [e.__dict__ for e in evaluations[:top_k]],
        }

    def select_from_evaluated_paths(
        self,
        environment_id: str,
        evaluated_paths: dict[str, dict[str, Any]],
        skill_name: str | None = None,
        top_k: int = 3,
    ) -> dict[str, Any]:
        candidates = self.build_candidates(
            environment_id=environment_id,
            skill_name=skill_name,
        )

        for candidate in candidates:
            seed_id = candidate.record.seed_id
            item = evaluated_paths.get(seed_id)
            if not item:
                continue

            candidate.decoded_path = item.get("decoded_path")
            candidate.modified_path = item.get("modified_path")
            candidate.stomp_time_sec = item.get("stomp_time_sec")

        return self.evaluate_and_select(candidates, top_k=top_k)

    def register_success(self, seed_id: str) -> bool:
        for record in self.registry.records:
            if record.seed_id == seed_id:
                record.success_count += 1
                self.registry.save()
                return True
        return False

    def select_nearest_by_start_goal(
        self,
        environment_id: str,
        target_start_joints: list[float],
        target_goal_joints: list[float],
        skill_name: str | None = None,
        top_k: int = 3,
    ) -> dict[str, Any]:
        # 候補seedを取得
        candidates = self.build_candidates(
            environment_id=environment_id,
            skill_name=skill_name,
        )

        # 距離を計算（start距離 + goal距離）
        scored: list[dict[str, Any]] = []
        for candidate in candidates:
            # 比較用のstart/goalを取得
            start_ref, goal_ref = self._reference_start_goal(candidate)

            # L2距離
            start_dist = self._l2_distance(start_ref, target_start_joints)
            goal_dist = self._l2_distance(goal_ref, target_goal_joints)
            total_dist = start_dist + goal_dist

            scored.append(
                {
                    "seed_id": candidate.record.seed_id,
                    "environment_id": candidate.record.environment_id,
                    "skill_name": candidate.record.skill_name,
                    "relative_path": candidate.record.relative_path,
                    "absolute_path": str(candidate.absolute_path),
                    "start_distance": start_dist,
                    "goal_distance": goal_dist,
                    "distance": total_dist,
                    "success_count": candidate.record.success_count,
                }
            )

        # 近い順にソート
        scored.sort(key=lambda x: x["distance"])
        return {
            # 1位
            "selected_seed": scored[0] if scored else None,
            # 上位k件
            "evaluations": scored[:top_k],
        }

    @staticmethod
    def _reference_start_goal(candidate: PathSeedCandidate) -> tuple[list[float], list[float]]:
        return (
            [float(x) for x in candidate.record.start_joints],
            [float(x) for x in candidate.record.goal_joints],
        )

    @staticmethod
    def _l2_distance(a: list[float], b: list[float]) -> float:
        # 要素数チェック
        if len(a) != len(b):
            raise ValueError(f"joint length mismatch: {len(a)} != {len(b)}")
        # L2距離
        return math.sqrt(sum((float(x) - float(y)) ** 2 for x, y in zip(a, b)))

    @staticmethod
    def make_demo_path(
        start_joints: list[float],
        goal_joints: list[float],
        offset: float = 0.0,
    ) -> JointPath:
        """s,gの中間地点を生成
        """
        mid = [
            0.5 * (s + g) + offset
            for s, g in zip(start_joints, goal_joints)
        ]
        return [
            list(start_joints),
            mid,
            list(goal_joints),
        ]

#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import annotations

from pathlib import Path
from typing import Any

from ..type.models import PathSeedCandidate, JointPath
from .registry import PathSeedRegistry
from .evaluator import PathSeedEvaluator, EvaluationWeights
from .validator import PathSeedValidator


class PathSeedSelector:
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

    @staticmethod
    def make_demo_path(
        start_joints: list[float],
        goal_joints: list[float],
        offset: float = 0.0,
    ) -> JointPath:
        mid = [
            0.5 * (s + g) + offset
            for s, g in zip(start_joints, goal_joints)
        ]
        return [
            list(start_joints),
            mid,
            list(goal_joints),
        ]

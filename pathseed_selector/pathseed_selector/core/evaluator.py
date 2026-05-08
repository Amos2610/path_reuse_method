#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import annotations

from dataclasses import dataclass

from ..type.models import PathSeedCandidate, PathSeedEvaluation
from ..tools.metrics import diff_norm, path_length, minmax_normalize


@dataclass
class EvaluationWeights:
    alpha_diff_norm: float = 0.2
    beta_stomp_time: float = 0.0
    gamma_path_length: float = 0.8

    def normalized(self) -> "EvaluationWeights":
        total = self.alpha_diff_norm + self.beta_stomp_time + self.gamma_path_length
        if total <= 0.0:
            return EvaluationWeights(0.2, 0.0, 0.8)

        return EvaluationWeights(
            alpha_diff_norm=self.alpha_diff_norm / total,
            beta_stomp_time=self.beta_stomp_time / total,
            gamma_path_length=self.gamma_path_length / total,
        )


class PathSeedEvaluator:
    def __init__(self, weights: EvaluationWeights | None = None) -> None:
        self.weights = (weights or EvaluationWeights()).normalized()

    def evaluate(self, candidates: list[PathSeedCandidate]) -> list[PathSeedEvaluation]:
        raw_items = []

        for candidate in candidates:
            if candidate.decoded_path is None:
                continue
            if candidate.modified_path is None:
                continue

            d_i = diff_norm(candidate.decoded_path, candidate.modified_path)
            t_i = (
                candidate.stomp_time_sec
                if candidate.stomp_time_sec is not None
                else candidate.record.plan_time_sec
                if candidate.record.plan_time_sec is not None
                else 0.0
            )
            l_i = path_length(candidate.modified_path)

            raw_items.append((candidate, d_i, float(t_i), l_i))

        if not raw_items:
            return []

        d_values = [x[1] for x in raw_items]
        t_values = [x[2] for x in raw_items]
        l_values = [x[3] for x in raw_items]

        d_norm = minmax_normalize(d_values)
        t_norm = minmax_normalize(t_values)
        l_norm = minmax_normalize(l_values)

        results: list[PathSeedEvaluation] = []

        for idx, (candidate, d_i, t_i, l_i) in enumerate(raw_items):
            score = (
                self.weights.alpha_diff_norm * d_norm[idx]
                + self.weights.beta_stomp_time * t_norm[idx]
                + self.weights.gamma_path_length * l_norm[idx]
            )

            record = candidate.record

            results.append(
                PathSeedEvaluation(
                    seed_id=record.seed_id,
                    environment_id=record.environment_id,
                    skill_name=record.skill_name,
                    relative_path=record.relative_path,
                    absolute_path=str(candidate.absolute_path),
                    diff_norm=d_i,
                    stomp_time_sec=t_i,
                    path_length=l_i,
                    normalized_diff_norm=d_norm[idx],
                    normalized_stomp_time=t_norm[idx],
                    normalized_path_length=l_norm[idx],
                    score=score,
                    success_count=record.success_count,
                )
            )

        results.sort(key=lambda x: x.score)
        return results

#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path


JointPath = list[list[float]]


@dataclass
class PathSeedRecord:
    seed_id: str
    environment_id: str
    relative_path: str
    skill_name: str

    start_joints: list[float]
    goal_joints: list[float]

    plan_time_sec: float | None = None
    success_count: int = 0


@dataclass
class PathSeedCandidate:
    record: PathSeedRecord
    absolute_path: Path

    decoded_path: JointPath | None = None
    modified_path: JointPath | None = None
    stomp_time_sec: float | None = None


@dataclass
class PathSeedEvaluation:
    seed_id: str
    environment_id: str
    skill_name: str
    relative_path: str
    absolute_path: str

    diff_norm: float
    stomp_time_sec: float
    path_length: float

    normalized_diff_norm: float
    normalized_stomp_time: float
    normalized_path_length: float

    score: float
    success_count: int

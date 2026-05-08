#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import annotations

import math

from ..type.models import JointPath


def joint_distance(a: list[float], b: list[float]) -> float:
    if len(a) != len(b):
        raise ValueError(f"joint dimension mismatch: {len(a)} != {len(b)}")
    return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))


def validate_joint_path(path: JointPath) -> None:
    if not path:
        raise ValueError("path is empty")

    dim = len(path[0])
    if dim == 0:
        raise ValueError("joint dimension is zero")

    for i, q in enumerate(path):
        if len(q) != dim:
            raise ValueError(f"path dimension mismatch at index {i}")


def resample_path(path: JointPath, target_len: int) -> JointPath:
    validate_joint_path(path)

    if target_len <= 0:
        raise ValueError("target_len must be positive")

    if len(path) == target_len:
        return [list(q) for q in path]

    if len(path) == 1:
        return [list(path[0]) for _ in range(target_len)]

    result: JointPath = []
    src_last = len(path) - 1

    for i in range(target_len):
        u = 0.0 if target_len == 1 else i / float(target_len - 1)
        x = u * src_last
        lo = int(math.floor(x))
        hi = min(lo + 1, src_last)
        ratio = x - lo

        q = [
            (1.0 - ratio) * path[lo][j] + ratio * path[hi][j]
            for j in range(len(path[0]))
        ]
        result.append(q)

    return result


def diff_norm(decoded_path: JointPath, modified_path: JointPath) -> float:
    validate_joint_path(decoded_path)
    validate_joint_path(modified_path)

    target_len = max(len(decoded_path), len(modified_path))
    d_path = resample_path(decoded_path, target_len)
    m_path = resample_path(modified_path, target_len)

    return sum(joint_distance(d, m) for d, m in zip(d_path, m_path))


def path_length(path: JointPath) -> float:
    validate_joint_path(path)

    if len(path) <= 1:
        return 0.0

    return sum(joint_distance(path[i], path[i + 1]) for i in range(len(path) - 1))


def minmax_normalize(values: list[float]) -> list[float]:
    if not values:
        return []

    v_min = min(values)
    v_max = max(values)

    if math.isclose(v_min, v_max):
        return [0.0 for _ in values]

    return [(v - v_min) / (v_max - v_min) for v in values]

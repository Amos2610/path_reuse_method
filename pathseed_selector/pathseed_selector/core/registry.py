#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

from ..type.models import PathSeedRecord


class PathSeedRegistry:
    def __init__(self, registry_path: str | Path, library_root: str | Path) -> None:
        self.registry_path = Path(registry_path)
        self.library_root = Path(library_root)
        self.records: list[PathSeedRecord] = []
        self.load()

    def load(self) -> None:
        self.records = []

        if not self.registry_path.exists():
            return

        with open(self.registry_path, "r", encoding="utf-8") as f:
            data = json.load(f)

        for item in data.get("records", []):
            metadata = item.get("metadata")
            if metadata is not None and not isinstance(metadata, dict):
                metadata = {"value": metadata}

            self.records.append(
                PathSeedRecord(
                    seed_id=str(item["seed_id"]),
                    environment_id=str(item["environment_id"]),
                    relative_path=str(item["relative_path"]),
                    skill_name=str(item.get("skill_name", "")),
                    start_joints=[float(x) for x in item["start_joints"]],
                    goal_joints=[float(x) for x in item["goal_joints"]],
                    plan_time_sec=(
                        None
                        if item.get("plan_time_sec") is None
                        else float(item.get("plan_time_sec"))
                    ),
                    success_count=int(item.get("success_count", 0)),
                    metadata=metadata,
                )
            )

    def save(self) -> None:
        data = {
            "version": "0.4",
            "records": [
                {
                    "seed_id": r.seed_id,
                    "environment_id": r.environment_id,
                    "relative_path": r.relative_path,
                    "skill_name": r.skill_name,
                    "start_joints": r.start_joints,
                    "goal_joints": r.goal_joints,
                    "plan_time_sec": r.plan_time_sec,
                    "success_count": r.success_count,
                    "metadata": r.metadata or {},
                }
                for r in self.records
            ],
        }

        self.registry_path.parent.mkdir(parents=True, exist_ok=True)
        with open(self.registry_path, "w", encoding="utf-8") as f:
            json.dump(data, f, ensure_ascii=False, indent=2)

    def resolve_path(self, record: PathSeedRecord) -> Path:
        path = Path(record.relative_path)
        if path.is_absolute():
            return path
        return self.library_root / path

    def find_records(
        self,
        environment_id: str | None = None,
        skill_name: str | None = None,
        metadata_filters: dict[str, Any] | None = None,
    ) -> list[PathSeedRecord]:
        results = []

        for record in self.records:
            if environment_id and record.environment_id != environment_id:
                continue

            if skill_name and skill_name not in record.skill_name:
                continue

            if metadata_filters:
                meta = record.metadata or {}
                matched = True
                for k, v in metadata_filters.items():
                    if meta.get(k) != v:
                        matched = False
                        break
                if not matched:
                    continue

            results.append(record)

        return results

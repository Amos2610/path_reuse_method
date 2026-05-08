#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import annotations

from pathlib import Path

from ..type.models import PathSeedCandidate


class PathSeedValidator:
    def validate_file_exists(self, path: Path) -> bool:
        return path.exists() and path.is_file()

    def validate_candidate_ready(self, candidate: PathSeedCandidate) -> bool:
        if not self.validate_file_exists(candidate.absolute_path):
            return False
        if candidate.decoded_path is None:
            return False
        if candidate.modified_path is None:
            return False
        return True

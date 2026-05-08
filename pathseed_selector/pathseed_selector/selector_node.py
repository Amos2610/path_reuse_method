#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import annotations

import json
import rclpy
from rclpy.node import Node

from .core.selector import PathSeedSelector
from .core.planner import PathSeedPlanner
from .core.evaluator import EvaluationWeights


class PathSeedSelectorNode(Node):
    def __init__(self) -> None:
        super().__init__("pathseed_selector_node")

        self.declare_parameter(
            "registry_path",
            "/root/ros2_ws/src/path_reuse_method/pathseed_selector/config/path_registry.json",
        )
        self.declare_parameter(
            "library_root",
            "/root/ros2_ws/src/path_reuse_method/pathseeds/Library",
        )
        self.declare_parameter("environment_id", "desk_scene_v1")

        registry_path = self.get_parameter("registry_path").value
        library_root = self.get_parameter("library_root").value
        environment_id = self.get_parameter("environment_id").value

        selector = PathSeedSelector(
            registry_path=registry_path,
            library_root=library_root,
            weights=EvaluationWeights(
                alpha_diff_norm=0.2,
                beta_stomp_time=0.0,
                gamma_path_length=0.8,
            ),
        )

        evaluated_paths = {}

        for candidate in selector.build_candidates(environment_id=environment_id):
            r = candidate.record

            decoded_path = selector.make_demo_path(
                r.start_joints,
                r.goal_joints,
                offset=0.10 if "pick" in r.seed_id else 0.20,
            )

            modified_path = selector.make_demo_path(
                r.start_joints,
                r.goal_joints,
                offset=0.02 if "pick" in r.seed_id else 0.03,
            )

            evaluated_paths[r.seed_id] = {
                "decoded_path": decoded_path,
                "modified_path": modified_path,
                "stomp_time_sec": r.plan_time_sec or 0.0,
            }

        candidates = selector.build_candidates(
            environment_id=environment_id,
        )

        planner = PathSeedPlanner()

        planned_candidates = planner.run(
            candidates=candidates,
            start_joints=[0.916, 0.724, -1.70014, 0.001, 0.977, -0.67],
            goal_joints=[
                2.268928025,
                0.8203047475,
                -1.8675022975,
                0.0,
                1.04719755,
                0.593411945,
            ],
        )

        result = selector.evaluate_and_select(
            candidates=planned_candidates,
            top_k=10,
        )

        self.get_logger().info("pathseed evaluation result:")
        self.get_logger().info(json.dumps(result, ensure_ascii=False, indent=2))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PathSeedSelectorNode()
    rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

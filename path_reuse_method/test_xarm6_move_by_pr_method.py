#!/usr/bin/env python3
import sys
import argparse
import rclpy
from rclpy.parameter import Parameter
from rclpy.node import Node

# 既存の PR クライアント
from path_seed_client import PathSeedClient

# 既存の xArm ユーティリティ
from xarm_utils_py import XArmUtils, Node as XArmNode


class TestXArm6MoveByPRMethod(Node):
    def __init__(self, seed_name, start, goal, execute):
        self.node = Node("test_xarm6_move_by_pr_method")
        super().__init__(self.node.get_name())  # Node の初期化

        # ROS 初期化
        self.seed_name = seed_name
        self.start = start
        self.goal = goal
        self.execute_flag = execute

        # PathSeedClient と xArm ノードの作成
        self.pr_client = PathSeedClient()
        self.xnode = XArmNode("test_xarm6_pr_method")
        self.xarm = XArmUtils(self.xnode, "xarm6")

    def run(self):
        # === Phase 1: Decode & Set PathSeed ===
        decoded_path = self.pr_client.send_decode_path_seed(self.seed_name, self.start, self.goal)
        # デコードされたパスシードをセット
        self.pr_client.send_set_path_seed(decoded_path)

        # === Phase 2: STOMP planning & execution ===
        self.xarm.set_planning_pipeline("stomp")
        self.xarm.set_move_group_parameter("stomp.use_custom_trajectory", True) # STOMPでカスタム軌道を使用する
        self.xarm.set_joint_value_target(self.goal)

        if self.xarm.plan():
            print(f"Plan success for joint values: {self.goal}")
            if self.execute_flag:
                if self.xarm.execute():
                    print("Execution success")
                else:
                    print("Execution failed")
        else:
            print("Plan failed for joint values:", self.goal)

        # 初期位置に戻す
        self.xarm.set_move_group_parameter("stomp.use_custom_trajectory", False) # STOMPでカスタム軌道を使用しない

        print("Move back to initial position...")
        self.xarm.move_to_initial()


def main():
    # ros2 run path_reuse_method test_xarm6_move_by_pr_method.py   --seed-name src/path_reuse_method/pathseeds/Library/ex1_pick_and_place/updated/pathseed_pick.txt   --start 0,0,0,0,0,0   --goal 0.916,0.724,-1.700,0.001,0.977,-0.67  --execute

    parser = argparse.ArgumentParser(description="PR法によるxArm6動作テスト")
    parser.add_argument("--seed-name", type=str, default="src/path_reuse_method/pathseeds/Library/ex1_pick_and_place/updated/pathseed_pick.txt", help="PathSeedファイルのパス")
    parser.add_argument("--start", type=str, default="0,0,0,0,0,0")
    parser.add_argument("--goal", type=str, default="0.916,0.724,-1.700,0.001,0.977,-0.67", help="目標関節角度のCSV形式")
    parser.add_argument("--execute", action="store_true", help="計画成功時に実行する")

    args = parser.parse_args()

    start = parse_joint_list(args.start)
    goal = parse_joint_list(args.goal)

    if len(start) != 6 or len(goal) != 6:
        print(f"[ERROR] start({len(start)}) and goal({len(goal)}) must both be length 6 for xArm6.")
        return 1

    rclpy.init()
    tester = TestXArm6MoveByPRMethod(args.seed_name, start, goal, args.execute)
    tester.run()
    rclpy.shutdown()

def parse_joint_list(csv: str):
    vals = [float(x.strip()) for x in csv.split(",") if x.strip() != ""]
    return vals


if __name__ == "__main__":
    main()

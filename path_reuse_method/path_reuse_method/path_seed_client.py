#!/usr/bin/env python3

import sys
import os
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory

from path_reuse_method_interfaces.srv import SetPathSeedTrajectory, GetPathSeedTrajectory, DecodePathSeed, EncodePathSeed
from path_reuse_method_interfaces.msg import PathSeed


class PathSeedClient(Node):
    def __init__(self):
        super().__init__('path_seed_client')
        self.set_cli = self.create_client(SetPathSeedTrajectory, 'set_path_seed_trajectory')
        self.get_cli = self.create_client(GetPathSeedTrajectory, 'get_path_seed_trajectory')
        self.decode_cli = self.create_client(DecodePathSeed, 'decode_path_seed')
        self.encode_cli = self.create_client(EncodePathSeed, 'encode_path_seed')

        # すべてのサービスが生きているか待つ（短縮形。実際は必要なサービスだけで良い）
        for cli in [self.set_cli, self.get_cli, self.decode_cli, self.encode_cli]:
            while not cli.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'Service {cli.srv_name} not available, waiting...')

    def send_set_path_seed(self, path_seed: PathSeed):
        req = SetPathSeedTrajectory.Request()
        req.path_seed = path_seed

        future = self.set_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        result = future.result()
        if result:
            self.get_logger().info(f"[Set] Response: success={result.success}, message='{result.message}'")
        else:
            self.get_logger().error('[Set] Service call failed')

    def send_get_path_seed(self):
        req = GetPathSeedTrajectory.Request()
        future = self.get_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        result = future.result()
        if result:
            self.get_logger().info(f"[Get] rows={result.path_seed.rows}, cols={result.path_seed.cols}, data={result.path_seed.data[:10]} ...")
        else:
            self.get_logger().error('[Get] Service call failed')

    def send_decode_path_seed(self, path_seed_name, start_joints, goal_joints):
        req = DecodePathSeed.Request()
        req.path_seed_name = path_seed_name
        req.start_joints = start_joints
        req.goal_joints = goal_joints

        future = self.decode_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        result = future.result()
        if result:
            self.get_logger().info(
                f"[Decode] rows={result.path_seed.rows}, cols={result.path_seed.cols}, data={result.path_seed.data[:10]} ...")
        else:
            self.get_logger().error('[Decode] Service call failed')

        return result.path_seed

    def send_encode_path_seed(self, trajectory=None, trajectory_file_path=None, relative_saved_path=None):
        req = EncodePathSeed.Request()
        if trajectory is None and trajectory_file_path is None:
            self.get_logger().error("No input provided for encoding.")
            return

        req.trajectory = trajectory
        req.trajectory_file_path = trajectory_file_path
        req.relative_saved_path = relative_saved_path

        future = self.encode_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        result = future.result()
        if result:
            self.get_logger().info(f"[Encode] Response: success={result.success}, path_seed_path='{result.path_seed_path}'")
        else:
            self.get_logger().error('[Encode] Service call failed')

        return result.success, result.path_seed_path

    def select_best_path_seed(
        self,
        environment_id: str,
        skill_name: str,
        start_joints: list,
        goal_joints: list,
        registry_path: str = "/root/ros2_ws/src/path_reuse_method/pathseed_selector/config/path_registry.json",
        library_root: str = "/root/ros2_ws/src/path_reuse_method/pathseeds/Library",
    ):
        """
        pathseed_selector を使って、start/goal に最も近い PathSeed を1件返す。
        戻り値: 絶対パス文字列 or None
        """
        try:
            from pathseed_selector.core.selector import PathSeedSelector
            selector = PathSeedSelector(
                registry_path=registry_path,
                library_root=library_root,
            )
            result = selector.select_nearest_by_start_goal(
                environment_id=environment_id,
                target_start_joints=[float(x) for x in start_joints],
                target_goal_joints=[float(x) for x in goal_joints],
                skill_name=skill_name,
                top_k=5,
            )
            selected = result.get("selected_seed")
            if not selected:
                self.get_logger().warn("[Select] No pathseed candidate matched.")
                return None
            abs_path = selected.get("absolute_path")
            if abs_path and os.path.isfile(abs_path):
                self.get_logger().info(
                    f"[Select] selected seed_id={selected.get('seed_id')} distance={selected.get('distance'):.6f} path={abs_path}"
                )
                return abs_path
            self.get_logger().warn(f"[Select] selected candidate has invalid path: {abs_path}")
            return None
        except Exception as e:
            self.get_logger().error(f"[Select] selection failed: {e}")
            return None


def main(args=None):
    rclpy.init(args=args)
    client = PathSeedClient()

    # サンプルデータ
    cols = 6
    data = '0.0253615, 0.00990554, -0.111783, 0.0776433, -0.0137638, -0.0539208, ' \
           '0.0396319, 0.017227, -0.223884, 0.155411, 0.0388611, -0.0250857, ' \
           '0.0484995, 0.0252098, -0.340112, 0.235977, 0.0883874, 0.00642416'
    data = [float(x) for x in data.split(',')]
    rows = len(data) // cols

    # コマンドライン引数で呼び出すサービスを切り替え
    if len(sys.argv) < 2:
        print("使い方: python path_seed_client.py [set|get|decode]")
        rclpy.shutdown()
        return

    cmd = sys.argv[1]
    if cmd == "set":
        path_seed = PathSeed()
        path_seed.rows = rows
        path_seed.cols = cols
        path_seed.data = data
        client.send_set_path_seed(path_seed)
    elif cmd == "get":
        client.send_get_path_seed()
    elif cmd == "decode":
        path_seed_name = "ex1_pick_and_place/updated/pathseed_pick.txt"
        start_joints = [0.1] * 6
        goal_joints = [0.2] * 6
        client.send_decode_path_seed(path_seed_name, start_joints, goal_joints)
    else:
        print("Unknown command:", cmd)

    rclpy.shutdown()

if __name__ == '__main__':
    main()

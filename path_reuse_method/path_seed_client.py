#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node

from path_reuse_method.srv import SetPathSeedTrajectory, GetPathSeedTrajectory, DecodePathSeed
from path_reuse_method.msg import PathSeed


class PathSeedClient(Node):
    def __init__(self):
        super().__init__('path_seed_client')
        self.set_cli = self.create_client(SetPathSeedTrajectory, 'set_path_seed_trajectory')
        self.get_cli = self.create_client(GetPathSeedTrajectory, 'get_path_seed_trajectory')
        self.decode_cli = self.create_client(DecodePathSeed, 'decode_path_seed')

        # すべてのサービスが生きているか待つ（短縮形。実際は必要なサービスだけで良い）
        for cli in [self.set_cli, self.get_cli, self.decode_cli]:
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
        path_seed_name = "dummy_seed"
        start_joints = [0.1] * 6
        goal_joints = [0.2] * 6
        client.send_decode_path_seed(path_seed_name, start_joints, goal_joints)
    else:
        print("Unknown command:", cmd)

    rclpy.shutdown()

if __name__ == '__main__':
    main()

#!/usr/bin/env python3

import os
import pwd
import grp
import yaml
import numpy as np
import rclpy
from rclpy.node import Node
import tempfile
from ament_index_python.packages import get_package_share_directory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from path_reuse_method.srv import EncodePathSeed
from tools import encode
import shutil


class EncodeServer(Node):
    def __init__(self):
        super().__init__('encode_path_seed_sub_server')
        self.encoder = encode.Encoder()

        share_dir = get_package_share_directory('path_reuse_method')
        self.package_root = os.path.abspath(
            os.path.join(share_dir, '..', '..', '..', '..', 'src', 'path_reuse_method')
        )

        self.srv = self.create_service(
            EncodePathSeed,
            'encode_path_seed_sub_server',
            self.encode_cb
        )

    def encode_cb(self, request, response):
        """
        Service callback for encoding path seed.
        Args:
            request:
                trajectory (trajectory_msgs/JointTrajectory): The trajectory data to be encoded.
                trajectory_file_path (string): The file path to the trajectory data.
                relative_saved_path (string): Relative path (from pathseeds/) to save the path seed.
            response:
                success (bool): Whether the encoding was successful.
                path_seed_path (string): Absolute path to the saved path seed file.
        """
        waypoint_data = None

        tmp_dir = tempfile.mkdtemp()

        try:
            # --- 1. angle.txt を用意 ---
            if request.trajectory.points:
                # trajectory から生成
                self.get_logger().info("Encoding from trajectory data...")
                angle_path = os.path.join(tmp_dir, "angle.txt")
                with open(angle_path, "w") as tmp:
                    for p in request.trajectory.points:
                        tmp.write(str(list(p.positions)) + "\n")

            elif request.trajectory_file_path:
                # ファイルから読み込み
                self.get_logger().info(f"Encoding from trajectory file: {request.trajectory_file_path}")

                if not os.path.exists(request.trajectory_file_path):
                    response.success = False
                    response.path_seed_path = "File not found"
                    return response

                try:
                    with open(request.trajectory_file_path, 'r') as f:
                        traj_yaml = yaml.safe_load(f)
                except Exception as e:
                    self.get_logger().error(f"Failed to read trajectory file: {e}")
                    response.success = False
                    response.path_seed_path = "Invalid file format"
                    return response

                points = traj_yaml.get('joint_trajectory', {}).get('points', [])
                if not points:
                    self.get_logger().error("No points found in trajectory file")
                    response.success = False
                    response.path_seed_path = "No points found"
                    return response

                angle_path = os.path.join(tmp_dir, "angle.txt")
                with open(angle_path, "w") as tmp:
                    for pt in points:
                        tmp.write(str(pt.get('positions', [])) + "\n")
            else:
                self.get_logger().error("No trajectory or trajectory_file_path provided.")
                response.success = False
                response.path_seed_path = "No input"
                return response

            # --- 2. エンコード実行 ---
            pathseed_data = self.encoder.generate_pathseed(tmp_dir)
            if pathseed_data is None:
                response.success = False
                response.path_seed_path = "Encode failed"
                return response

            # --- 3. 保存パス組み立て ---
            base_dir = os.path.join(self.package_root, 'pathseeds', 'Library')
            save_path = os.path.join(base_dir, request.relative_saved_path)
            save_dir = os.path.dirname(save_path)
            # フォルダ作成
            os.makedirs(save_dir, exist_ok=True)
            
            #TODO: Docker環境では所有権の設定が必要だが，難しかったので一旦ステイ
            # uid = os.getuid()
            # gid = os.getgid()
            # self.get_logger().info(f"Setting ownership to UID: {uid}, GID: {gid} for {save_dir}")
            # os.chown(save_dir, uid, gid)

            for key in ['r1_array', 'r2_array', 'tau1_array', 'tau2_array', 'delta_array']:
                if isinstance(pathseed_data[key], np.ndarray):
                    pathseed_data[key] = pathseed_data[key].tolist()

            with open(save_path, "w") as file:
                file.write(str(pathseed_data))

            self.get_logger().info(f"✅ Success: Path seed saved to {save_path}")
            response.success = True
            response.path_seed_path = save_path

        except Exception as e:
            self.get_logger().error(f"Encoding failed: {e}")
            response.success = False
            response.path_seed_path = f"Error: {e}"

        finally:
            shutil.rmtree(tmp_dir, ignore_errors=True)

        return response


def main(args=None):
    rclpy.init(args=args)
    node = EncodeServer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

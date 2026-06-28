#!/usr/bin/env python3

import ast
import os
import traceback
import rclpy
from rclpy.node import Node
from path_reuse_method_interfaces.srv import DecodePathSeed
from path_reuse_method.tools import decode


class DecodeServer(Node):
    def __init__(self):
        super().__init__('decode_path_seed_sub_server')
        # Decodeクラスのインスタンスを生成
        self.decoder = decode.Decoder()

        self.srv = self.create_service(DecodePathSeed, 'decode_path_seed_sub_server', self.decode_cb)

    def _set_empty_path_seed(self, response):
        response.path_seed.rows = 0
        response.path_seed.cols = 0
        response.path_seed.data = []
        return response

    def decode_cb(self, request, response):
        path_seed_name = request.path_seed_name
        self.get_logger().info(f"Received request to decode path seed: {path_seed_name}")
        self.get_logger().info(
            "Decode request detail: "
            f"exists={os.path.exists(path_seed_name)}, "
            f"start_joints_len={len(request.start_joints)}, "
            f"goal_joints_len={len(request.goal_joints)}"
        )
        self.get_logger().debug(f"Start joint values: {request.start_joints}")
        self.get_logger().debug(f"Goal joint values: {request.goal_joints}")
        #TODO: start_joint_valuesとgoal_joint_valuesの整合性を確認する処理を追加

        # Missing or broken pathseed files must not kill this worker. If this
        # process exits, path_seed_server keeps running but later requests become
        # "decode_path_seed_worker not available" until the launch is restarted.
        try:
            generate_path = self.decoder.generate_path(
                path_seed_name,
                request.start_joints,
                request.goal_joints,
            )
        except Exception as exc:
            self.get_logger().error(
                "Decode failed; returning empty PathSeed and keeping worker alive. "
                f"path_seed_name={path_seed_name}, error={type(exc).__name__}: {exc}"
            )
            self.get_logger().debug(traceback.format_exc())
            return self._set_empty_path_seed(response)

        matrix = []
        try:
            if isinstance(generate_path, str):
                # 改行ごとに1行、各行は "[f1, f2, ...]" 形式
                lines = [ln.strip() for ln in generate_path.strip().splitlines() if ln.strip()]
                for ln in lines:
                    row = ast.literal_eval(ln)  # -> list[float]
                    matrix.append([float(x) for x in row])
            else:
                # すでに 2次元配列として返るケース
                for row in generate_path:
                    # row が numpy/array.array の可能性もあるので float キャスト
                    matrix.append([float(x) for x in row])
        except Exception as exc:
            self.get_logger().error(
                "Decoded path format is invalid; returning empty PathSeed. "
                f"path_seed_name={path_seed_name}, error={type(exc).__name__}: {exc}"
            )
            self.get_logger().debug(traceback.format_exc())
            return self._set_empty_path_seed(response)

        if not matrix:
            self.get_logger().error(
                "Decoded path is empty; returning empty PathSeed. "
                f"path_seed_name={path_seed_name}"
            )
            return self._set_empty_path_seed(response)

        # matrixの最初にstart_joint_values，最後に goal_joint_values を追加
        matrix.insert(0, request.start_joints)
        matrix.append(request.goal_joints)

        # generate_pathをPathSeedメッセージに変換
        rows = len(matrix)
        cols = len(matrix[0]) if rows > 0 else 0
        flat_data = [x for row in matrix for x in row]  # フラット化

        # レスポンスにデータを設定
        response.path_seed.rows = rows
        response.path_seed.cols = cols
        response.path_seed.data = flat_data
        self.get_logger().info(
            "Decode succeeded: "
            f"path_seed_name={path_seed_name}, rows={rows}, cols={cols}, data_size={len(flat_data)}"
        )

        return response

def main(args=None):
    rclpy.init(args=args)
    node = DecodeServer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

#!/usr/bin/env python3

import ast
import rclpy
from rclpy.node import Node
from path_reuse_method.srv import DecodePathSeed
import decode


class DecodeServer(Node):
    def __init__(self):
        super().__init__('decode_path_seed_sub_server')
        # Decodeクラスのインスタンスを生成
        self.decoder = decode.Decoder()

        self.srv = self.create_service(DecodePathSeed, 'decode_path_seed_sub_server', self.decode_cb)

    def decode_cb(self, request, response):
        self.get_logger().info(f"Received request to decode path seed: {request.path_seed_name}")
        self.get_logger().info(f"Start joint values: {request.start_joints}")
        self.get_logger().info(f"Goal joint values: {request.goal_joints}")
        #TODO: start_joint_valuesとgoal_joint_valuesの整合性を確認する処理を追加

        # デコード処理を呼び出す
        generate_path = self.decoder.generate_path(request.path_seed_name, request.start_joints, request.goal_joints)

        matrix = []
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

        return response

def main(args=None):
    rclpy.init(args=args)
    node = DecodeServer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

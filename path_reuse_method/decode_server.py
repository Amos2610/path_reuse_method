"""
【実装中】このプログラムはまだ実行できません．
"""

import rclpy
from rclpy.node import Node
from path_reuse_method.srv import DecodePathSeed
import decode


class DecodeServer(Node):
    def __init__(self):
        super().__init__('decode_path_seed_server')
        # Decodeクラスのインスタンスを生成
        self.decoder = decode.Decoder()

        self.srv = self.create_service(DecodePathSeed, 'decode_path_seed', self.decode_cb)

    def decode_cb(self, request, response):
        self.get_logger().info(f"Received request to decode path seed: {request.path_seed_name}")
        self.get_logger().info(f"Start joint values: {request.start_joints}")
        self.get_logger().info(f"Goal joint values: {request.goal_joints}")
        #TODO: start_joint_valuesとgoal_joint_valuesの整合性を確認する処理を追加

        # デコード処理を呼び出す
        generate_path = self.decoder.generate_path(request.path_seed_name, request.start_joints, request.goal_joints)

        self.get_logger().info(f"Generated path: {generate_path}")

        # generate_pathをPathSeedメッセージに変換
        rows = len(generate_path)  # 行数を設定
        cols = len(generate_path[0]) if generate_path else 0  # 列数を設定
        data = list(generate_path)  # データをリストに変換

        # レスポンスにデータを設定
        response.path_seed.rows = rows
        response.path_seed.cols = cols
        response.path_seed.data = data

        return response

def main(args=None):
    rclpy.init(args=args)
    node = DecodeServer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

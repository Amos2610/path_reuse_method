#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import datetime
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from geometry_msgs.msg import Pose
from trajectory_msgs.msg import JointTrajectory
from moveit_msgs.action import ExecuteTrajectory
from xarm_utils_py import XArmUtils
from xarm_utils_py import Node as XArmNode
from path_reuse_method_interfaces.srv import EncodePathSeed


class GeneratePathSeedFromGUI(Node):
    def __init__(self):
        super().__init__('generate_pathseed_from_gui_node')

        xarm_node = XArmNode("xarm6_utils_node")

        # XArm6初期化
        self.xarm = XArmUtils(xarm_node, "xarm6")

        # ActionServerの初期化
        self._action_server = ActionServer(
            self,
            ExecuteTrajectory,
            '/execute_trajectory',
            self.execute_callback
        )

        # サービスクライアントの作成
        self.cli = self.create_client(EncodePathSeed, 'encode_path_seed_sub_server')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('サービス encode_path_seed_sub_server の起動待ち...')
        self.get_logger().info('サービス encode_path_seed_sub_server に接続しました')

    def __del__(self):
        self.get_logger().info("ノードをシャットダウンします...")
        rclpy.shutdown()
        self.get_logger().info("シャットダウン完了")

    def _call_encode_service(self, trajectory: JointTrajectory):
        """EncodePathSeed サービス呼び出し"""
        relative_path = input("相対保存パスを入力してください (例: ex1_pick_and_place/pre_defined/pick/pathseed_pick.txt): ")
        req = EncodePathSeed.Request()
        req.trajectory = trajectory
        req.trajectory_file_path = ""  # trajectory_file_path を使わない場合は空文字
        req.relative_saved_path = relative_path
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            res = future.result()
            if res.success:
                self.get_logger().info(f"PathSeed生成完了: {res.path_seed_path}")
            else:
                self.get_logger().warn(f"PathSeed生成失敗")
            return res
        else:
            self.get_logger().error("サービス呼び出しに失敗しました")
            return None
    
    async def execute_callback(self, goal_handle):
        self.get_logger().info("ExecuteTrajectory goal 受信")
        traj = goal_handle.request.trajectory.joint_trajectory
        self._call_encode_service(traj)
        goal_handle.succeed()
        return ExecuteTrajectory.Result()


def main():
    rclpy.init()
    node = GeneratePathSeedFromGUI()
    # waiting log
    node.get_logger().info("Waiting for ExecuteTrajectory goals...")
    node.get_logger().info("RVizのGUIを利用して，目標位置までのパスを実行してください．")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

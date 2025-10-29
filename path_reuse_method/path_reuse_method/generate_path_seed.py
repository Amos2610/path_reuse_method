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

START_JOINT_VALUES = [0.916, 0.724, -1.700, 0.001, 0.977, -0.67]
GOAL_JOINT_VALUES  = [2.227, 0.731, -1.714, 0.002, 0.983, 0.551]


class GeneratePathSeed(Node):
    def __init__(self):
        super().__init__('generate_pathseed_node')

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

    def to_given_goal(self):
        """手動で start/goal を与えて生成"""
        self.get_logger().info(f"Start joints: {START_JOINT_VALUES}")
        self.get_logger().info(f"Goal joints:  {GOAL_JOINT_VALUES}")

        # start位置に移動
        self.get_logger().info("Moving to start position...")
        self.xarm.set_joint_value_target(START_JOINT_VALUES)
        success = self.xarm.plan()
        if success is True:
            self.xarm.execute()
            self.get_logger().info("Start position reached")

        result = None
        while result != "y":
            success, plan, _, _ = self.xarm.plan()
            if not success:
                self.get_logger().warn("Planning failed, retrying...")
                continue

            result = input(">>> Do you want to save this plan in pathseed? (y/n):")
            if result == "y":
                self._call_encode_service(plan)
                break

def main():
    rclpy.init()
    node = GeneratePathSeed()
    if input(">>> Give a new start and goal? (y/n): ").lower() == 'y':
        try:
            # waiting log
            node.get_logger().info("Waiting for ExecuteTrajectory goals...")
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
    else:
        node.to_given_goal()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

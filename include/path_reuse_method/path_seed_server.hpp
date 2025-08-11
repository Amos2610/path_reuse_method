#pragma once

#include <memory>
#include <string>
#include <vector>
#include "rclcpp/rclcpp.hpp"

#include "path_reuse_method/msg/path_seed.hpp"
#include "path_reuse_method/srv/set_path_seed_trajectory.hpp"
#include "path_reuse_method/srv/get_path_seed_trajectory.hpp"
#include "path_reuse_method/srv/decode_path_seed.hpp"

// PathSeedServer: 全サービスを1ノードで提供する
class PathSeedServer : public rclcpp::Node
{
public:
  PathSeedServer();

private:
  // SetPathSeedTrajectory: PathSeed登録サービス
  void handle_set_path_seed(
    const std::shared_ptr<path_reuse_method::srv::SetPathSeedTrajectory::Request> request,
    std::shared_ptr<path_reuse_method::srv::SetPathSeedTrajectory::Response> response);

  // GetPathSeedTrajectory: PathSeed取得サービス
  void handle_get_path_seed(
    const std::shared_ptr<path_reuse_method::srv::GetPathSeedTrajectory::Request> request,
    std::shared_ptr<path_reuse_method::srv::GetPathSeedTrajectory::Response> response);

  // DecodePathSeed: 条件からPathSeed生成・検索サービス
  void handle_decode_path_seed(
    const std::shared_ptr<path_reuse_method::srv::DecodePathSeed::Request> request,
    std::shared_ptr<path_reuse_method::srv::DecodePathSeed::Response> response);

  // サービスハンドル
  rclcpp::Service<path_reuse_method::srv::SetPathSeedTrajectory>::SharedPtr set_srv_;
  rclcpp::Service<path_reuse_method::srv::GetPathSeedTrajectory>::SharedPtr get_srv_;
  rclcpp::Service<path_reuse_method::srv::DecodePathSeed>::SharedPtr decode_srv_;
  // path_seed_server.py にサブサーバを構築
  rclcpp::Client<path_reuse_method::srv::DecodePathSeed>::SharedPtr decode_sub_srv_;

  // コールバックグループ（python側のサブサービスを待つため）
  rclcpp::CallbackGroup::SharedPtr client_cbg_;


  // 内部状態（PathSeedの保存用など、必要なら追加で定義）
  path_reuse_method::msg::PathSeed latest_path_seed_;
  // 必要ならstd::map等で複数管理も可
};


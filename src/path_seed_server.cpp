#include "path_reuse_method/path_seed_server.hpp"


PathSeedServer::PathSeedServer() : Node("path_seed_server")
{
  set_srv_ = this->create_service<path_reuse_method::srv::SetPathSeedTrajectory>(
    "set_path_seed_trajectory",
    std::bind(&PathSeedServer::handle_set_path_seed, this, std::placeholders::_1, std::placeholders::_2));
  get_srv_ = this->create_service<path_reuse_method::srv::GetPathSeedTrajectory>(
    "get_path_seed_trajectory",
    std::bind(&PathSeedServer::handle_get_path_seed, this, std::placeholders::_1, std::placeholders::_2));
  decode_srv_ = this->create_service<path_reuse_method::srv::DecodePathSeed>(
    "decode_path_seed",
    std::bind(&PathSeedServer::handle_decode_path_seed, this, std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(this->get_logger(), "PathSeedServer with all services ready!");
}

// --------- SetPathSeedTrajectoryサービスのコールバック ---------
void PathSeedServer::handle_set_path_seed(
  const std::shared_ptr<path_reuse_method::srv::SetPathSeedTrajectory::Request> request,
  std::shared_ptr<path_reuse_method::srv::SetPathSeedTrajectory::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "SetPathSeed: rows=%d, cols=%d, data_size=%zu",
              request->path_seed.rows, request->path_seed.cols, request->path_seed.data.size());
  // 最新PathSeed情報を保存
  latest_path_seed_ = request->path_seed;
  response->success = true;
  response->message = "PathSeed registered";
}

// --------- GetPathSeedTrajectoryサービスのコールバック ---------
void PathSeedServer::handle_get_path_seed(
  const std::shared_ptr<path_reuse_method::srv::GetPathSeedTrajectory::Request> /*request*/,
  std::shared_ptr<path_reuse_method::srv::GetPathSeedTrajectory::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "GetPathSeed: returning latest PathSeed rows=%d cols=%d size=%zu",
              latest_path_seed_.rows, latest_path_seed_.cols, latest_path_seed_.data.size());
  response->path_seed = latest_path_seed_;
}

// --------- DecodePathSeedサービスのコールバック ---------
void PathSeedServer::handle_decode_path_seed(
  const std::shared_ptr<path_reuse_method::srv::DecodePathSeed::Request> request,
  std::shared_ptr<path_reuse_method::srv::DecodePathSeed::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "DecodePathSeed: name=%s, start_joints_size=%zu, goal_joints_size=%zu",
              request->path_seed_name.c_str(), request->start_joints.size(), request->goal_joints.size());
  // ここでは一旦最新PathSeedを返す
  response->path_seed = latest_path_seed_;
}

// --------- main関数 ---------
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PathSeedServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
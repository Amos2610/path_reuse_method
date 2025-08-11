#include "path_reuse_method/path_seed_server.hpp"

PathSeedServer::PathSeedServer() : Node("path_seed_server")
{
    // クライアント用の Reentrant グループ
    client_cbg_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    // サービスの登録
    set_srv_ = this->create_service<path_reuse_method::srv::SetPathSeedTrajectory>(
        "set_path_seed_trajectory",
        std::bind(&PathSeedServer::handle_set_path_seed, this, std::placeholders::_1, std::placeholders::_2));
    get_srv_ = this->create_service<path_reuse_method::srv::GetPathSeedTrajectory>(
        "get_path_seed_trajectory",
        std::bind(&PathSeedServer::handle_get_path_seed, this, std::placeholders::_1, std::placeholders::_2));
    decode_srv_ = this->create_service<path_reuse_method::srv::DecodePathSeed>(
        "decode_path_seed",
        std::bind(&PathSeedServer::handle_decode_path_seed, this, std::placeholders::_1, std::placeholders::_2));
    // Python側のサブサービスのクライアントを作成
    decode_sub_srv_ = this->create_client<path_reuse_method::srv::DecodePathSeed>(
        "decode_path_seed_sub_server",
        rmw_qos_profile_services_default,
        client_cbg_);

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
    // Pythonのサブサービスを待つ
    if (!decode_sub_srv_->wait_for_service(std::chrono::seconds(10))) {
        RCLCPP_ERROR(this->get_logger(), "decode_path_seed_worker not available");
        response->path_seed = path_reuse_method::msg::PathSeed(); // 空のPathSeedを返す
        return;
    }
    
    // リクエストをそのまま転送
    auto req = std::make_shared<path_reuse_method::srv::DecodePathSeed::Request>();
    req->path_seed_name = request->path_seed_name;
    req->start_joints = request->start_joints;
    req->goal_joints = request->goal_joints;

    auto future = decode_sub_srv_->async_send_request(req);
    
    // マルチスレッド実行器なら別スレッドで返信コールバックが回るので wait_for でOK
    auto status = future.wait_for(std::chrono::seconds(60));
    if (status != std::future_status::ready) {
        RCLCPP_ERROR(this->get_logger(), "Timeout waiting decode_path_seed_worker response");
        response->path_seed = path_reuse_method::msg::PathSeed();
        return;
    }

    // レスポンスを受け取る
    auto worker_resp = future.get();
    if (!worker_resp)
    {
        RCLCPP_ERROR(this->get_logger(),
                     "decode_path_seed_worker returned null response");
        response->path_seed = path_reuse_method::msg::PathSeed();
        return;
    }

    // 受け取ったPathSeedを返し、最新にも保存
    response->path_seed = worker_resp->path_seed;
    latest_path_seed_ = worker_resp->path_seed;

    // 一貫性チェック（任意）
    const size_t expected =
        static_cast<size_t>(latest_path_seed_.rows) *
        static_cast<size_t>(latest_path_seed_.cols);
    if (latest_path_seed_.data.size() != expected)
    {
        RCLCPP_WARN(this->get_logger(),
                    "Decoded PathSeed size mismatch: data=%zu, rows=%d, cols=%d",
                    latest_path_seed_.data.size(), latest_path_seed_.rows, latest_path_seed_.cols);
    }

    RCLCPP_INFO(this->get_logger(),
                "Decode done: rows=%d cols=%d size=%zu",
                latest_path_seed_.rows, latest_path_seed_.cols,
                latest_path_seed_.data.size());
}

// --------- main関数 ---------
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PathSeedServer>();

    // マルチスレッドで回す（最低2スレッド推奨）
    rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 2);
    exec.add_node(node);
    exec.spin();

    rclcpp::shutdown();
    return 0;
}
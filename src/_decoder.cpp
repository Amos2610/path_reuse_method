#include "path_reuse_method/decoder.hpp"
#include <fstream>
#include <sstream>
#include <iomanip>
#include <cmath>
#include <stdexcept>
#include <iostream>
#include <algorithm>
#include <cnpy.h>

Decoder::Decoder() {}

PathSeedData Decoder::loadData(const std::string& pathseed_file)
{
    PathSeedData data;
    auto npz = cnpy::npz_load(pathseed_file);

    data.r1_array = Eigen::Map<Eigen::MatrixXd>(npz["r1_array"].data<double>(), npz["r1_array"].shape[0], npz["r1_array"].shape[1]);
    data.r2_array = Eigen::Map<Eigen::MatrixXd>(npz["r2_array"].data<double>(), npz["r2_array"].shape[0], npz["r2_array"].shape[1]);
    data.tau1_array = Eigen::Map<Eigen::MatrixXd>(npz["tau1_array"].data<double>(), npz["tau1_array"].shape[0], npz["tau1_array"].shape[1]);
    data.tau2_array = Eigen::Map<Eigen::MatrixXd>(npz["tau2_array"].data<double>(), npz["tau2_array"].shape[0], npz["tau2_array"].shape[1]);
    data.delta_array = Eigen::Map<Eigen::MatrixXd>(npz["delta_array"].data<double>(), npz["delta_array"].shape[0], npz["delta_array"].shape[1]);
    data.w_num = static_cast<int>(npz["r1_array"].shape[0]);

    return data;
}

Eigen::VectorXd Decoder::normalizeVector(const Eigen::VectorXd& v)
{
    return v.normalized();
}

Eigen::Vector3d Decoder::crossThreeDim(const Eigen::Vector3d& v1, const Eigen::Vector3d& v2)
{
    return v1.cross(v2);
}

std::string Decoder::convertData(const Eigen::MatrixXd& matrix)
{
    std::ostringstream oss;
    oss << std::setprecision(8) << std::fixed;
    for (int i = 0; i < matrix.rows(); ++i) {
        for (int j = 0; j < matrix.cols(); ++j) {
            oss << matrix(i, j);
            if (!(i == matrix.rows()-1 && j == matrix.cols()-1)) {
                oss << ", ";
            }
        }
    }
    return oss.str();
}

std::string Decoder::formatedData(const std::string& data)
{
    // CSV風データのまま返すだけ（Python同様に必要に応じて実装）
    return data;
}

void Decoder::writeData(const std::string& write_data, const std::string& write_file)
{
    std::ofstream out(write_file);
    out << write_data;
    out.close();
}

std::string Decoder::decode(
    const PathSeedData& pathseed_data,
    const Eigen::VectorXd& start_joint_values,
    const Eigen::VectorXd& goal_joint_values)
{
    // === 1. 開始・終了ジョイントから最も近いwインデックスを探索 ===
    int best_index = -1;
    double best_score = 1e9;
    for (int w = 0; w < pathseed_data.w_num; ++w) {
        // Python: np.linalg.norm(start - r1_array[w]) + np.linalg.norm(goal - r2_array[w])
        double score = (start_joint_values - pathseed_data.r1_array.row(w).transpose()).norm()
                     + (goal_joint_values - pathseed_data.r2_array.row(w).transpose()).norm();
        if (score < best_score) {
            best_score = score;
            best_index = w;
        }
    }
    if (best_index < 0) throw std::runtime_error("No matching path seed found");

    // === 2. パス配列を抽出（N,6行列として） ===
    Eigen::MatrixXd path(pathseed_data.tau1_array.cols(), 6);
    for (int i = 0; i < pathseed_data.tau1_array.cols(); ++i) {
        // Python: tau = tau1_array[best_index,i], tau2_array[best_index,i]
        for (int j = 0; j < 6; ++j) {
            double v = (i < pathseed_data.tau1_array.cols()/2)
                ? pathseed_data.tau1_array(best_index, i)
                : pathseed_data.tau2_array(best_index, i - pathseed_data.tau1_array.cols()/2);
            path(i, j) = v;
        }
    }
    // 必要に応じて「合成」「変換」処理を追加

    // === 3. 文字列出力に変換 ===
    return convertData(path);
}

std::string Decoder::generatePath(const std::string& pathseed_file,
                                  const Eigen::VectorXd& start_joint_values,
                                  const Eigen::VectorXd& goal_joint_values)
{
    auto data = loadData(pathseed_file);
    return decode(data, start_joint_values, goal_joint_values);
}

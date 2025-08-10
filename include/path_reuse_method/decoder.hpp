#pragma once

#include <Eigen/Dense>
#include <string>
#include <vector>

struct PathSeedData {
    int w_num;
    Eigen::MatrixXd r1_array;
    Eigen::MatrixXd r2_array;
    Eigen::MatrixXd tau1_array;
    Eigen::MatrixXd tau2_array;
    Eigen::MatrixXd delta_array;
};

class Decoder
{
public:
    Decoder();

    PathSeedData loadData(const std::string& pathseed_file);
    Eigen::VectorXd normalizeVector(const Eigen::VectorXd& v);
    Eigen::Vector3d crossThreeDim(const Eigen::Vector3d& v1, const Eigen::Vector3d& v2);
    std::string convertData(const Eigen::MatrixXd& matrix);
    std::string formatedData(const std::string& data);
    std::string decode(const PathSeedData& pathseed_data,
                       const Eigen::VectorXd& start_joint_values,
                       const Eigen::VectorXd& goal_joint_values);
    void writeData(const std::string& write_data, const std::string& write_file);
    std::string generatePath(const std::string& pathseed_file,
                             const Eigen::VectorXd& start_joint_values,
                             const Eigen::VectorXd& goal_joint_values);

};

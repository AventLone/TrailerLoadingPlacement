#pragma once
#include <Eigen/Core>

// struct HistoryData
// {
//     Eigen::Vector3f trailer_pose;
//     Eigen::Vector3f trailer_dimensions;
//     Eigen::Vector3f ramp_position;
//     Eigen::Vector2f ramp_size;
//     float ramp_tilt{};
//     std::vector<Eigen::Vector2f> placement_zones;
//
//     void reset()
//     {
//         trailer_pose.setZero();
//         trailer_dimensions.setZero();
//         ramp_position.setZero();
//         ramp_size.setZero();
//         ramp_tilt = 0.0f;
//     }
// };

// struct HistoryData
// {
//     std::vector<Eigen::Vector3f> trailer_pose;
//     Eigen::Vector3f trailer_dimensions;
//     Eigen::Vector3f ramp_position;
//     Eigen::Vector2f ramp_size;
//     float ramp_tilt{};
//
//     void reset()
//     {
//         trailer_pose.setZero();
//         trailer_dimensions.setZero();
//         ramp_position.setZero();
//         ramp_size.setZero();
//         ramp_tilt = 0.0f;
//     }
//
//     void update(const Eigen::Vector3f& trailer_pose_observe, const Eigen::Vector3f& trailer_dimensions,)
// };
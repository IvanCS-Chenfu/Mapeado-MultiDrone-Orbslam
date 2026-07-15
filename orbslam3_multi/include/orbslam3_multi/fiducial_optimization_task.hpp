#pragma once

#include "orbslam3_multi/raw_map_types.hpp"

#include <Eigen/Core>

#include <cstdint>
#include <string>

namespace orbslam3_multi
{

// F1H: tarea ligera creada cuando una segunda observacion fiducial mide un
// residual alto. No ejecuta optimizacion; solo conserva evidencia para que
// PoseGraphBuilder/OptimizationManager la consuman en subfases posteriores.
struct FiducialOptimizationTask
{
    uint64_t task_id = 0;
    std::string task_type = "FIDUCIAL_REVISIT_ERROR";
    RawKeyFrameId keyframe_id;
    RawSubmapId submap_id;
    uint32_t drone_id = 0;
    uint64_t map_epoch = 0;
    int fiducial_id = 0;
    Eigen::Matrix4d estimated_world_T_kf = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d target_world_T_kf = Eigen::Matrix4d::Identity();
    double error_t_m = 0.0;
    double error_rot_rad = 0.0;
    double error_yaw_rad = 0.0;
    double threshold_t_m = 0.0;
    double threshold_rot_rad = 0.0;
    double threshold_yaw_rad = 0.0;
    uint64_t task_generation = 1;
    uint64_t created_arrival_id = 0;
    uint64_t latest_arrival_id = 0;
    std::string status = "PENDING";
    uint64_t update_count = 0;
    std::string created_source;
    std::string latest_source;
    uint64_t arrival_id = 0;
    std::string source;
};

}  // namespace orbslam3_multi

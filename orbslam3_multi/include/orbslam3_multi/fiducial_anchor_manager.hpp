#pragma once

#include "orbslam3_multi/fiducial_optimization_task.hpp"
#include "orbslam3_multi/global_pose_store.hpp"
#include "orbslam3_multi/raw_map_database.hpp"
#include "orbslam3_multi/raw_map_types.hpp"

#include <Eigen/Core>

#include <cstdint>
#include <map>
#include <set>
#include <string>
#include <vector>

namespace orbslam3_multi
{

struct FiducialObservation
{
    uint64_t arrival_id = 0;
    uint32_t drone_id = 0;
    uint64_t map_epoch = 0;
    uint64_t local_keyframe_id = 0;
    uint64_t global_keyframe_id = 0;
    int fiducial_id = 0;
    Eigen::Matrix4d world_T_body_fiducial = Eigen::Matrix4d::Identity();
    double keyframe_stamp_sec = 0.0;
    double gt_stamp_sec = 0.0;
    double association_dt_sec = 0.0;
    double distance_to_fiducial_m = 0.0;
    std::string source;
};

struct FiducialAnchorResult
{
    bool observation_accepted = false;
    bool first_anchor_created = false;
    bool submap_already_anchored = false;
    bool keyframe_marked_hard_fiducial = false;
    bool revisit = false;
    bool pose_error_valid = false;
    bool revisit_ok = false;
    bool task_created = false;
    bool duplicate_task = false;
    RawSubmapId submap_id;
    RawKeyFrameId keyframe_id;
    Eigen::Matrix4d world_T_local = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d world_T_camera_fiducial = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d estimated_world_T_kf = Eigen::Matrix4d::Identity();
    double error_t_m = 0.0;
    double error_rot_rad = 0.0;
    double error_yaw_rad = 0.0;
    double threshold_t_m = 0.0;
    double threshold_rot_rad = 0.0;
    double threshold_yaw_rad = 0.0;
    uint64_t task_id = 0;
    std::string reason;
};

struct FiducialAnchorStats
{
    uint64_t observations = 0;
    uint64_t accepted = 0;
    uint64_t rejected = 0;
    uint64_t anchors_created = 0;
    uint64_t replay_observations = 0;
    uint64_t hard_fiducial_keyframes = 0;
    uint64_t revisit_observations = 0;
    uint64_t revisit_ok = 0;
    uint64_t revisit_high_error = 0;
    uint64_t tasks_created = 0;
    uint64_t task_duplicates = 0;
    uint64_t revisit_no_pose = 0;
};

struct FiducialRevisitConfig
{
    double error_threshold_m = 0.35;
    double yaw_threshold_rad = 0.25;
    double rot_threshold_rad = 0.35;
    std::string source = "default";
};

// F1E: backend puro de anclaje fiducial. Recibe observaciones ya construidas
// por el servidor, consulta raw/local pose y escribe el anchor en GlobalPoseStore.
// No conoce ROS, Gazebo ni topics; eso mantiene el GT simulado fuera del backend.
class FiducialAnchorManager
{
public:
    void ConfigureRevisitThresholds(const FiducialRevisitConfig& config);

    FiducialAnchorResult RegisterFiducialObservation(
        const FiducialObservation& observation,
        const RawMapDatabase& raw_db,
        GlobalPoseStore& pose_store);

    FiducialAnchorStats GetStats() const;
    std::vector<FiducialOptimizationTask> GetPendingFiducialOptimizationTasks() const;
    uint64_t PendingFiducialOptimizationTaskCount() const;

private:
    struct TaskGroupKey
    {
        RawSubmapId submap_id;
        int fiducial_id = 0;

        bool operator<(const TaskGroupKey& other) const
        {
            if (submap_id.drone_id != other.submap_id.drone_id)
            {
                return submap_id.drone_id < other.submap_id.drone_id;
            }
            if (submap_id.map_epoch != other.submap_id.map_epoch)
            {
                return submap_id.map_epoch < other.submap_id.map_epoch;
            }
            return fiducial_id < other.fiducial_id;
        }
    };

    std::set<RawKeyFrameId> hard_fiducial_keyframes_;
    std::vector<FiducialOptimizationTask> pending_tasks_;
    std::map<TaskGroupKey, uint64_t> task_ids_by_key_;
    FiducialAnchorStats stats_;
    FiducialRevisitConfig revisit_config_;
    uint64_t next_task_id_ = 1;
};

}  // namespace orbslam3_multi

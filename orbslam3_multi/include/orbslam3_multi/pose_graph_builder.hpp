#pragma once

#include "orbslam3_multi/fiducial_optimization_task.hpp"
#include "orbslam3_multi/global_pose_store.hpp"
#include "orbslam3_multi/pose_graph_problem.hpp"
#include "orbslam3_multi/raw_map_database.hpp"

#include <cstdint>
#include <string>

namespace orbslam3_multi
{

struct PoseGraphBuilderConfig
{
    uint64_t max_vertices = 24;
    uint64_t min_vertices = 2;
    uint64_t max_path_length = 80;
    bool anchor_stop_enabled = true;
    bool fiducial_connectivity_enabled = true;
    bool branch_selection_enabled = true;
    double subdivision_candidate_radius_m = 2.0;
    double fiducial_neighborhood_radius_m = 4.0;
    // Legacy launch knob kept for compatibility; 1L uses the metric radius.
    uint64_t fiducial_neighborhood_radius_kfs = 3;
    uint64_t max_temporal_edge_kf_gap = 15;
    double max_temporal_edge_length_m = 4.0;
    double corner_yaw_threshold_rad = 0.5235987756;
    bool include_temporal_edges = true;
    double temporal_edge_weight = 25.0;
    double temporal_edge_weight_sparse = 10.0;
    double fiducial_hard_weight = 1000000.0;
    double fiducial_target_translation_weight = 5000.0;
    double fiducial_target_rotation_weight = 2500.0;
    double current_pose_soft_weight = 5.0;
};

struct PoseGraphBuildResult
{
    bool success = false;
    std::string reason;
    PoseGraphProblem problem;
};

// F1I: construye un PoseGraphProblem temporal desde una tarea fiducial.
// Entradas: tarea F1H, RawMapDatabase y GlobalPoseStore de solo lectura.
// Salida: grafo con vertices, aristas, priors y PropagationPlan; no optimiza ni
// aplica poses. Existe para aislar el problema que consumira el solver futuro.
class PoseGraphBuilder
{
public:
    PoseGraphBuilder() = default;
    explicit PoseGraphBuilder(const PoseGraphBuilderConfig& config);

    void Configure(const PoseGraphBuilderConfig& config);
    const PoseGraphBuilderConfig& GetConfig() const;

    PoseGraphBuildResult BuildForFiducialTask(
        const FiducialOptimizationTask& task,
        const RawMapDatabase& raw_db,
        const GlobalPoseStore& pose_store) const;

private:
    PoseGraphBuilderConfig config_;
};

}  // namespace orbslam3_multi

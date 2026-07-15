#pragma once

#include "orbslam3_multi/raw_map_types.hpp"

#include <Eigen/Core>

#include <cstdint>
#include <string>
#include <vector>

namespace orbslam3_multi
{

// F1J: propuesta temporal de pose producida por el dry-run. No representa
// estado persistente; en 1K sera la entrada para un apply seguro si `useful`
// queda aceptado.
struct OptimizationPoseProposal
{
    RawKeyFrameId keyframe_id;
    Eigen::Matrix4d before_world_T_kf = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d proposed_world_T_kf = Eigen::Matrix4d::Identity();
    bool variable_vertex = false;
    bool fixed_vertex = false;
    bool propagated = false;
    double delta_t_m = 0.0;
    double delta_yaw_rad = 0.0;
};

// F1J: resultado completo de un dry-run. Se devuelve en memoria al servidor
// para log/decision; no se serializa ni se usa como API por CSV/SVG.
struct OptimizationDryRunResult
{
    uint64_t task_id = 0;
    std::string task_type;
    bool success = false;
    bool useful = false;
    bool partial_candidate = false;
    bool precheck_ok = false;
    bool no_state_mutation = true;
    std::string reason;
    std::string decision_reason;
    std::string partial_reason;

    uint64_t solver_iterations = 0;
    double initial_cost = 0.0;
    double final_cost = 0.0;
    double before_error_t = 0.0;
    double after_error_t = 0.0;
    double before_error_yaw = 0.0;
    double after_error_yaw = 0.0;
    double improvement_ratio = 0.0;

    uint64_t copied_poses = 0;
    uint64_t moved_kfs = 0;
    uint64_t fixed_kfs = 0;
    bool hard_fixed_moved = false;
    bool anchor_preservation_required = false;
    bool anchor_preservation_graph_satisfied = false;
    bool anchor_preservation_ok = true;
    uint64_t previous_fiducial_fixed_count = 0;
    uint64_t previous_fiducial_neighborhood_fixed_count = 0;
    double max_previous_fiducial_delta_t = 0.0;
    double max_previous_fiducial_delta_yaw = 0.0;
    double max_previous_fiducial_neighborhood_delta_t = 0.0;
    double max_previous_fiducial_neighborhood_delta_yaw = 0.0;
    bool has_nan = false;
    double max_delta_t = 0.0;
    double mean_delta_t = 0.0;
    double max_delta_yaw = 0.0;

    uint64_t propagated_kfs = 0;
    double max_propagated_delta_t = 0.0;
    double mean_propagated_delta_t = 0.0;

    std::vector<OptimizationPoseProposal> proposed_vertex_poses;
    std::vector<OptimizationPoseProposal> proposed_propagated_poses;
    std::string debug_export_path;
};

// F1K: detalle de cada KF escrito durante un apply real. Permite que el
// servidor loggee que se movio, de que forma y que correccion heredable quedo
// asociada sin exponer estructuras internas de GlobalPoseStore.
struct OptimizationApplyKeyFrameRecord
{
    RawKeyFrameId keyframe_id;
    bool propagated = false;
    bool fixed = false;
    bool applied = false;
    double delta_t_m = 0.0;
    double delta_yaw_rad = 0.0;
    double correction_t_m = 0.0;
    double correction_yaw_rad = 0.0;
    std::string source;
    std::string reason;
};

// F1K: resultado del primer apply persistente del pipeline nuevo. Escribe solo
// en GlobalPoseStore; RawMapDatabase debe permanecer intacta y queda reflejado
// explicitamente para validacion por logs.
struct OptimizationApplyResult
{
    uint64_t task_id = 0;
    std::string task_type;
    bool precheck_ok = false;
    bool applied = false;
    std::string reason;

    uint64_t optimized_kfs = 0;
    uint64_t propagated_kfs = 0;
    uint64_t skipped_propagated_kfs = 0;
    uint64_t pending_rebased_kfs = 0;
    uint64_t fixed_kfs = 0;
    bool hard_fixed_moved = false;
    double max_delta_t = 0.0;
    double mean_delta_t = 0.0;
    double max_delta_yaw = 0.0;
    bool last_correction_set = false;
    RawSubmapId last_correction_submap;
    RawKeyFrameId last_correction_from_keyframe;
    bool raw_db_modified = false;
    bool global_pose_store_modified = false;

    std::vector<OptimizationApplyKeyFrameRecord> optimized_records;
    std::vector<OptimizationApplyKeyFrameRecord> propagated_records;
};

enum class PostApplyDecision : uint8_t
{
    RejectRollback = 0,
    Accept = 1,
    PartialKeepForNextPass = 2,
};

struct PostApplyErrorMetrics
{
    double before_error_t = 0.0;
    double predicted_after_error_t = 0.0;
    double real_after_error_t = 0.0;
    double before_error_yaw = 0.0;
    double predicted_after_error_yaw = 0.0;
    double real_after_error_yaw = 0.0;
    double improvement_ratio = 0.0;
};

struct PostApplyInternalErrorMetrics
{
    double internal_mean_before = 0.0;
    double internal_mean_after = 0.0;
    double internal_max_before = 0.0;
    double internal_max_after = 0.0;
    double max_strong_internal_after = 0.0;
    double max_deformable_internal_after = 0.0;
    uint64_t num_edges_worse = 0;
    uint64_t num_edges_broken = 0;
    uint64_t strong_edges_broken = 0;
    uint64_t deformable_edges_broken = 0;
    uint64_t strong_edges_checked = 0;
    uint64_t deformable_edges_checked = 0;
};

struct PostApplyFixedCheck
{
    bool ok = true;
    bool hard_fixed_moved = false;
    uint64_t fixed_moved_count = 0;
    double max_fixed_delta_t = 0.0;
    double max_fixed_delta_yaw = 0.0;
};

struct PostApplyPropagationCheck
{
    bool ok = true;
    uint64_t propagated_count = 0;
    uint64_t skipped_propagated_count = 0;
    uint64_t rebased_count = 0;
    double propagated_max_delta_t = 0.0;
    double propagated_mean_delta_t = 0.0;
    double propagation_discontinuity_max_t = 0.0;
    double propagation_discontinuity_max_yaw = 0.0;
};

struct PostApplyAnchorPreservationCheck
{
    bool ok = true;
    bool required = false;
    bool graph_satisfied = true;
    uint64_t previous_fiducial_fixed_count = 0;
    uint64_t previous_fiducial_neighborhood_fixed_count = 0;
    uint64_t independent_branches = 0;
    uint64_t checked_branch_anchors = 0;
    uint64_t subdivision_candidates = 0;
    double max_previous_fiducial_delta_t = 0.0;
    double max_previous_fiducial_delta_yaw = 0.0;
    double max_previous_fiducial_neighborhood_delta_t = 0.0;
    double max_previous_fiducial_neighborhood_delta_yaw = 0.0;
    std::string reason = "not_required";
};

struct RetrySuggestion
{
    bool retry_allowed = false;
    std::string strategy = "REJECT_CANDIDATE";
    std::string reason;
};

// F1L: validacion real posterior al apply. Compara la prediccion 1J contra el
// estado persistente ya escrito en GlobalPoseStore y decide commit, parcial o
// rollback sin tocar RawMapDatabase.
struct PostApplyValidationResult
{
    uint64_t task_id = 0;
    std::string task_type;
    bool success = false;
    PostApplyDecision decision = PostApplyDecision::RejectRollback;
    std::string reason;
    bool forced_reject = false;
    bool needs_second_pass = false;
    bool internal_edges_ok = false;
    bool fixed_ok = false;
    bool propagation_ok = false;
    bool anchor_preservation_ok = true;
    bool primary_error_improved = false;
    bool final_error_ok = false;

    PostApplyErrorMetrics error;
    PostApplyInternalErrorMetrics internal_error;
    PostApplyFixedCheck fixed_check;
    PostApplyPropagationCheck propagation_check;
    PostApplyAnchorPreservationCheck anchor_preservation_check;
    RetrySuggestion retry;
};

const char* ToString(PostApplyDecision decision);

}  // namespace orbslam3_multi

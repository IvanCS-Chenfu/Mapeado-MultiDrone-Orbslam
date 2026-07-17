#include "orbslam3_multi/pose_graph_builder.hpp"

#include <Eigen/LU>

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <optional>
#include <set>
#include <vector>

namespace orbslam3_multi
{
namespace
{

RawSubmapId SubmapOf(const RawKeyFrameId& keyframe_id)
{
    return RawSubmapId{keyframe_id.drone_id, keyframe_id.map_epoch};
}

bool SameSubmap(const RawKeyFrameId& keyframe_id, const RawSubmapId& submap_id)
{
    return keyframe_id.drone_id == submap_id.drone_id &&
           keyframe_id.map_epoch == submap_id.map_epoch;
}

uint64_t AbsDiff(uint64_t a, uint64_t b)
{
    return a > b ? a - b : b - a;
}

constexpr double kPi = 3.14159265358979323846;

double NormalizeAngle(double angle)
{
    while (angle > kPi)
    {
        angle -= 2.0 * kPi;
    }
    while (angle < -kPi)
    {
        angle += 2.0 * kPi;
    }
    return angle;
}

double YawFromTransform(const Eigen::Matrix4d& transform)
{
    return std::atan2(transform(1, 0), transform(0, 0));
}

double PlanarDistance(const Eigen::Matrix4d& a, const Eigen::Matrix4d& b)
{
    return (a.block<2, 1>(0, 3) - b.block<2, 1>(0, 3)).norm();
}

double SpatialDistance(const Eigen::Matrix4d& a, const Eigen::Matrix4d& b)
{
    return (a.block<3, 1>(0, 3) - b.block<3, 1>(0, 3)).norm();
}

double SupportRigidityMultiplier(double density_kfs_per_m)
{
    if (!std::isfinite(density_kfs_per_m) || density_kfs_per_m <= 0.0)
    {
        return 0.65;
    }

    // 1I: mas KFs por metro sugiere mas textura/seguimiento local. Saturamos
    // para que esta señal no domine fiduciales ni convierta un tramo curvado en
    // bloque indeformable.
    constexpr double kReferenceDensity = 4.0;
    const double raw = std::sqrt(density_kfs_per_m / kReferenceDensity);
    return std::max(0.65, std::min(2.25, raw));
}

Eigen::Matrix4d RelativeTransform(const Eigen::Matrix4d& world_T_a,
                                  const Eigen::Matrix4d& world_T_b)
{
    return world_T_a.inverse() * world_T_b;
}

PoseGraphProblemSummary Summarize(const PoseGraphProblem& problem)
{
    PoseGraphProblemSummary summary;
    summary.vertices = problem.vertices.size();
    summary.edges = problem.edges.size();
    summary.priors = problem.priors.size();
    summary.fixed_vertices = problem.fixed_keyframes.size();
    summary.variable_vertices = problem.variable_keyframes.size();
    summary.affected_non_variable_keyframes =
        problem.affected_non_variable_keyframes.size();
    summary.propagation_entries = problem.propagation_plan.size();
    for (const auto& vertex : problem.vertices)
    {
        if (vertex.is_hard_fiducial)
        {
            ++summary.hard_fiducial_vertices;
        }
    }
    return summary;
}

std::optional<int32_t> FiducialIdForKeyFrame(
    const RawMapDatabase& raw_db,
    const RawKeyFrameId& keyframe_id)
{
    const auto observations = raw_db.GetFiducialObservationJournalCopy();
    std::optional<int32_t> fiducial_id;
    for (const auto& observation : observations)
    {
        if (observation.drone_id == keyframe_id.drone_id &&
            observation.map_epoch == keyframe_id.map_epoch &&
            observation.local_keyframe_id == keyframe_id.local_kf_id)
        {
            fiducial_id = observation.fiducial_id;
        }
    }
    return fiducial_id;
}

uint64_t KeyFrameGap(const RawKeyFrameId& a, const RawKeyFrameId& b)
{
    return AbsDiff(a.local_kf_id, b.local_kf_id);
}

double CumulativeDistanceBetween(
    const std::map<RawKeyFrameId, double>& cumulative_distance,
    const RawKeyFrameId& a,
    const RawKeyFrameId& b)
{
    const auto a_it = cumulative_distance.find(a);
    const auto b_it = cumulative_distance.find(b);
    if (a_it == cumulative_distance.end() || b_it == cumulative_distance.end())
    {
        return 0.0;
    }
    return std::abs(b_it->second - a_it->second);
}

}  // namespace

const char* ToString(PoseGraphEdgeType type)
{
    switch (type)
    {
        case PoseGraphEdgeType::TemporalNeighbor:
            return "TEMPORAL_NEIGHBOR";
        case PoseGraphEdgeType::SoftConsistency:
            return "SOFT_CONSISTENCY";
    }
    return "UNKNOWN";
}

const char* ToString(PoseGraphPriorType type)
{
    switch (type)
    {
        case PoseGraphPriorType::FiducialHard:
            return "FIDUCIAL_HARD";
        case PoseGraphPriorType::FiducialTarget:
            return "FIDUCIAL_TARGET";
        case PoseGraphPriorType::CurrentPoseSoft:
            return "CURRENT_POSE_SOFT";
    }
    return "UNKNOWN";
}

const char* ToString(PoseGraphPropagationMode mode)
{
    switch (mode)
    {
        case PoseGraphPropagationMode::NearestControlVertex:
            return "NEAREST_CONTROL_VERTEX";
        case PoseGraphPropagationMode::PathSegment:
            return "PATH_SEGMENT";
        case PoseGraphPropagationMode::InheritLastCorrection:
            return "INHERIT_LAST_CORRECTION";
    }
    return "UNKNOWN";
}

const char* ToString(FiducialConnectivityEdgeStatus status)
{
    switch (status)
    {
        case FiducialConnectivityEdgeStatus::DirectObserved:
            return "DIRECT_OBSERVED";
        case FiducialConnectivityEdgeStatus::DirectUncertain:
            return "DIRECT_UNCERTAIN";
        case FiducialConnectivityEdgeStatus::SubdivisionCandidate:
            return "SUBDIVISION_CANDIDATE";
        case FiducialConnectivityEdgeStatus::SubdividedConfirmed:
            return "SUBDIVIDED_CONFIRMED";
        case FiducialConnectivityEdgeStatus::BypassConfirmed:
            return "BYPASS_CONFIRMED";
    }
    return "UNKNOWN";
}

PoseGraphBuilder::PoseGraphBuilder(const PoseGraphBuilderConfig& config)
    : config_(config)
{
}

void PoseGraphBuilder::Configure(const PoseGraphBuilderConfig& config)
{
    config_ = config;
}

const PoseGraphBuilderConfig& PoseGraphBuilder::GetConfig() const
{
    return config_;
}

PoseGraphBuildResult PoseGraphBuilder::BuildForFiducialTask(
    const FiducialOptimizationTask& task,
    const RawMapDatabase& raw_db,
    const GlobalPoseStore& pose_store,
    const CovisibilityDatabase* covisibility_db) const
{
    // F1I: la ventana se calcula solo en el submapa de la tarea. No se expande
    // a loops ni a otros drones porque esas evidencias aun no existen en 1I.
    PoseGraphBuildResult result;
    result.problem.task_id = task.task_id;
    result.problem.task_type = task.task_type;
    result.problem.source = task.source;
    result.problem.submap_id = task.submap_id;
    result.problem.target_keyframe_id = task.keyframe_id;

    if (!SameSubmap(task.keyframe_id, task.submap_id))
    {
        result.reason = "task_keyframe_not_in_task_submap";
        return result;
    }
    if (!raw_db.HasSubmap(task.submap_id))
    {
        result.reason = "raw_submap_missing";
        return result;
    }
    if (!pose_store.GetWorldPose(task.keyframe_id))
    {
        result.reason = "target_world_pose_missing";
        return result;
    }

    std::vector<RawKeyFrameId> all_keyframes =
        raw_db.GetKeyFrameIdsForSubmap(task.submap_id);
    std::sort(all_keyframes.begin(), all_keyframes.end());
    if (all_keyframes.empty())
    {
        result.reason = "submap_without_keyframes";
        return result;
    }

    const auto target_it =
        std::find(all_keyframes.begin(), all_keyframes.end(), task.keyframe_id);
    if (target_it == all_keyframes.end())
    {
        result.reason = "target_keyframe_missing";
        return result;
    }

    std::vector<RawKeyFrameId> world_keyframes;
    world_keyframes.reserve(all_keyframes.size());
    for (const auto& keyframe_id : all_keyframes)
    {
        if (pose_store.GetWorldPose(keyframe_id))
        {
            world_keyframes.push_back(keyframe_id);
        }
    }
    if (world_keyframes.empty())
    {
        result.reason = "window_without_world_poses";
        return result;
    }

    const uint64_t target_local_id = task.keyframe_id.local_kf_id;
    std::vector<RawKeyFrameId> hard_fiducial_keyframes;
    for (const auto& keyframe_id : world_keyframes)
    {
        if (keyframe_id == task.keyframe_id)
        {
            continue;
        }
        const auto fiducial_id = FiducialIdForKeyFrame(raw_db, keyframe_id);
        if (pose_store.IsHardFiducialKeyFrame(keyframe_id) &&
            (!fiducial_id || fiducial_id.value() != task.fiducial_id))
        {
            hard_fiducial_keyframes.push_back(keyframe_id);
        }
    }

    // 1I: la seleccion topologica minima es temporal y conservadora.
    // Elegimos como fronteras independientes el hard fiducial mas cercano antes
    // y despues del target. Esto evita trasladar libremente una ventana sin
    // fijos, y descarta fiduciales mas lejanos dominados por uno mas cercano en
    // el mismo camino temporal.
    std::vector<RawKeyFrameId> branch_anchor_keyframes;
    std::optional<RawKeyFrameId> nearest_left_hard;
    std::optional<RawKeyFrameId> nearest_right_hard;
    for (const auto& keyframe_id : hard_fiducial_keyframes)
    {
        if (keyframe_id.local_kf_id < target_local_id)
        {
            if (!nearest_left_hard ||
                keyframe_id.local_kf_id > nearest_left_hard->local_kf_id)
            {
                nearest_left_hard = keyframe_id;
            }
        }
        else if (keyframe_id.local_kf_id > target_local_id)
        {
            if (!nearest_right_hard ||
                keyframe_id.local_kf_id < nearest_right_hard->local_kf_id)
            {
                nearest_right_hard = keyframe_id;
            }
        }
    }
    if (config_.fiducial_connectivity_enabled && config_.branch_selection_enabled)
    {
        if (nearest_left_hard)
        {
            branch_anchor_keyframes.push_back(nearest_left_hard.value());
        }
        if (nearest_right_hard)
        {
            branch_anchor_keyframes.push_back(nearest_right_hard.value());
        }
    }

    result.problem.anchor_preservation.required =
        !hard_fiducial_keyframes.empty();
    result.problem.anchor_preservation.independent_branches =
        branch_anchor_keyframes.size();
    result.problem.anchor_preservation.branch_anchor_count =
        branch_anchor_keyframes.size();
    result.problem.anchor_preservation.reason =
        branch_anchor_keyframes.empty()
            ? "no_previous_fiducial_branch_anchor"
            : "branch_anchors_selected";

    const auto target_fiducial_id = task.fiducial_id;
    for (const auto& anchor : branch_anchor_keyframes)
    {
        FiducialConnectivityEdge edge;
        edge.from_keyframe_id = anchor;
        edge.to_keyframe_id = task.keyframe_id;
        edge.from_fiducial_id = FiducialIdForKeyFrame(raw_db, anchor).value_or(-1);
        edge.to_fiducial_id = target_fiducial_id;
        edge.selected_as_branch_anchor = true;
        edge.independent_branch = true;
        edge.kf_gap = KeyFrameGap(anchor, task.keyframe_id);
        edge.status = edge.kf_gap > config_.max_path_length
            ? FiducialConnectivityEdgeStatus::DirectUncertain
            : FiducialConnectivityEdgeStatus::DirectObserved;
        edge.reason = edge.kf_gap > config_.max_path_length
            ? "selected_anchor_beyond_nominal_path_length"
            : "selected_nearest_branch_anchor";
        result.problem.fiducial_connectivity_edges.push_back(edge);
    }
    for (const auto& hard : hard_fiducial_keyframes)
    {
        if (std::find(branch_anchor_keyframes.begin(),
                      branch_anchor_keyframes.end(),
                      hard) != branch_anchor_keyframes.end())
        {
            continue;
        }
        FiducialConnectivityEdge edge;
        edge.from_keyframe_id = hard;
        edge.to_keyframe_id = task.keyframe_id;
        edge.from_fiducial_id = FiducialIdForKeyFrame(raw_db, hard).value_or(-1);
        edge.to_fiducial_id = target_fiducial_id;
        edge.kf_gap = KeyFrameGap(hard, task.keyframe_id);
        edge.status = FiducialConnectivityEdgeStatus::SubdividedConfirmed;
        edge.reason = "dominated_by_nearer_fiducial_in_same_temporal_branch";
        result.problem.fiducial_connectivity_edges.push_back(edge);
        ++result.problem.anchor_preservation.subdivided_confirmed;
    }

    uint64_t left_id = target_local_id;
    uint64_t right_id = target_local_id;
    bool found_left_hard = static_cast<bool>(nearest_left_hard);
    bool found_right_hard = static_cast<bool>(nearest_right_hard);
    if (nearest_left_hard)
    {
        left_id = nearest_left_hard->local_kf_id;
    }
    if (nearest_right_hard)
    {
        right_id = nearest_right_hard->local_kf_id;
    }

    if (!found_left_hard)
    {
        left_id = target_local_id > config_.max_path_length
            ? target_local_id - config_.max_path_length
            : 0;
    }
    if (!found_right_hard)
    {
        right_id = target_local_id + config_.max_path_length;
    }
    if (config_.anchor_stop_enabled && !branch_anchor_keyframes.empty())
    {
        for (const auto& anchor : branch_anchor_keyframes)
        {
            left_id = std::min<uint64_t>(left_id, anchor.local_kf_id);
            right_id = std::max<uint64_t>(right_id, anchor.local_kf_id);
        }
        found_left_hard = found_left_hard || static_cast<bool>(nearest_left_hard);
        found_right_hard = found_right_hard || static_cast<bool>(nearest_right_hard);
    }
    if (!config_.anchor_stop_enabled)
    {
        left_id = target_local_id > config_.max_path_length
            ? target_local_id - config_.max_path_length
            : 0;
        right_id = target_local_id + config_.max_path_length;
    }

    std::vector<RawKeyFrameId> window_keyframes;
    for (const auto& keyframe_id : world_keyframes)
    {
        if (keyframe_id.local_kf_id >= left_id &&
            keyframe_id.local_kf_id <= right_id)
        {
            window_keyframes.push_back(keyframe_id);
        }
    }
    if (window_keyframes.empty())
    {
        result.reason = "empty_window";
        return result;
    }

    std::map<RawKeyFrameId, Eigen::Matrix4d> window_poses;
    for (const auto& keyframe_id : window_keyframes)
    {
        const auto world_pose = pose_store.GetWorldPose(keyframe_id);
        if (world_pose)
        {
            window_poses[keyframe_id] = world_pose.value();
        }
    }
    std::map<RawKeyFrameId, double> cumulative_distance;
    double cumulative = 0.0;
    for (size_t i = 0; i < window_keyframes.size(); ++i)
    {
        if (i > 0)
        {
            const auto prev_it = window_poses.find(window_keyframes[i - 1]);
            const auto curr_it = window_poses.find(window_keyframes[i]);
            if (prev_it != window_poses.end() && curr_it != window_poses.end())
            {
                cumulative += PlanarDistance(prev_it->second, curr_it->second);
            }
        }
        cumulative_distance[window_keyframes[i]] = cumulative;
    }

    std::set<RawKeyFrameId> selected;
    selected.insert(task.keyframe_id);
    selected.insert(window_keyframes.front());
    selected.insert(window_keyframes.back());
    for (const auto& keyframe_id : branch_anchor_keyframes)
    {
        selected.insert(keyframe_id);
    }
    std::set<RawKeyFrameId> anchor_neighborhood_keyframes;
    for (const auto& anchor : branch_anchor_keyframes)
    {
        const auto anchor_pose_it = window_poses.find(anchor);
        if (anchor_pose_it == window_poses.end())
        {
            continue;
        }
        for (const auto& keyframe_id : window_keyframes)
        {
            const auto keyframe_pose_it = window_poses.find(keyframe_id);
            if (keyframe_pose_it != window_poses.end() &&
                SpatialDistance(keyframe_pose_it->second,
                                anchor_pose_it->second) <=
                    config_.fiducial_neighborhood_radius_m)
            {
                // 1I: la vecindad inmediata del fiducial previo se define por
                // distancia espacial, no por un numero fijo de KFs. Todo
                // control dentro del radio se entrega al solver como lock
                // relativo al fiducial, no como muestra equidistante normal.
                selected.insert(keyframe_id);
                anchor_neighborhood_keyframes.insert(keyframe_id);
            }
        }
    }
    std::set<RawKeyFrameId> target_neighborhood_keyframes;
    const auto target_pose_it = window_poses.find(task.keyframe_id);
    for (const auto& keyframe_id : window_keyframes)
    {
        const auto keyframe_pose_it = window_poses.find(keyframe_id);
        if (target_pose_it != window_poses.end() &&
            keyframe_pose_it != window_poses.end() &&
            SpatialDistance(keyframe_pose_it->second,
                            target_pose_it->second) <=
                config_.fiducial_neighborhood_radius_m)
        {
            // 1I: el fiducial objetivo no usa GT para decidir vecinos; se usa
            // la distancia en mapa actual respecto al KF fiducial observado.
            selected.insert(keyframe_id);
            target_neighborhood_keyframes.insert(keyframe_id);
        }
    }
    for (const auto& keyframe_id : window_keyframes)
    {
        if (pose_store.IsHardFiducialKeyFrame(keyframe_id))
        {
            selected.insert(keyframe_id);
        }
    }

    const uint64_t max_vertices = std::max<uint64_t>(config_.min_vertices, config_.max_vertices);
    const uint64_t available = window_keyframes.size();
    const double total_distance = cumulative_distance.empty()
        ? 0.0
        : cumulative_distance.rbegin()->second;
    if (total_distance > 1e-9 && max_vertices > 1U)
    {
        const uint64_t desired_samples =
            std::min<uint64_t>(available, std::max<uint64_t>(2U, max_vertices));
        // 1I: muestreo por longitud acumulada, no por indice local. Asi los
        // vertices de control quedan repartidos en espacio cuando ORB-SLAM3
        // emite KFs con frecuencia irregular.
        for (uint64_t sample = 0;
             sample < desired_samples && selected.size() < max_vertices;
             ++sample)
        {
            const double target_distance =
                total_distance *
                static_cast<double>(sample) /
                static_cast<double>(std::max<uint64_t>(1U, desired_samples - 1U));
            auto best = window_keyframes.front();
            double best_error = std::numeric_limits<double>::max();
            for (const auto& keyframe_id : window_keyframes)
            {
                const double dist = cumulative_distance[keyframe_id];
                const double error = std::abs(dist - target_distance);
                if (error < best_error)
                {
                    best = keyframe_id;
                    best_error = error;
                }
            }
            selected.insert(best);
        }
    }
    else
    {
        const uint64_t stride =
            max_vertices > 1U
                ? std::max<uint64_t>(1U, available / (max_vertices - 1U))
                : available;
        for (size_t i = 0;
             i < window_keyframes.size() && selected.size() < max_vertices;
             i += stride)
        {
            selected.insert(window_keyframes[i]);
        }
    }

    // 1I: las esquinas son puntos donde ORB-SLAM3 suele acumular deriva en
    // paredes pobres en textura. Se fuerzan como vertices antes de crear
    // edges para que la correccion pueda deshacer una esquina falsa sin que
    // una unica arista represente medio tramo deformado.
    for (size_t i = 1; i + 1 < window_keyframes.size() && selected.size() < max_vertices; ++i)
    {
        const auto prev_it = window_poses.find(window_keyframes[i - 1]);
        const auto curr_it = window_poses.find(window_keyframes[i]);
        const auto next_it = window_poses.find(window_keyframes[i + 1]);
        if (prev_it == window_poses.end() ||
            curr_it == window_poses.end() ||
            next_it == window_poses.end())
        {
            continue;
        }
        const double yaw_prev =
            NormalizeAngle(YawFromTransform(curr_it->second) -
                           YawFromTransform(prev_it->second));
        const double yaw_next =
            NormalizeAngle(YawFromTransform(next_it->second) -
                           YawFromTransform(curr_it->second));
        const double yaw_change = std::abs(NormalizeAngle(yaw_next - yaw_prev));
        if (yaw_change >= config_.corner_yaw_threshold_rad)
        {
            selected.insert(window_keyframes[i]);
        }
    }

    // 1I: si los fiduciales reales ya llenan el limite, conservamos los KFs
    // mas cercanos al objetivo porque son los que explican el residual.
    if (selected.size() < std::min<uint64_t>(max_vertices, window_keyframes.size()))
    {
        std::vector<RawKeyFrameId> by_distance = window_keyframes;
        std::sort(
            by_distance.begin(),
            by_distance.end(),
            [target_local_id](const RawKeyFrameId& a, const RawKeyFrameId& b)
            {
                return AbsDiff(a.local_kf_id, target_local_id) <
                       AbsDiff(b.local_kf_id, target_local_id);
            });
        for (const auto& keyframe_id : by_distance)
        {
            if (selected.size() >= max_vertices)
            {
                break;
            }
            selected.insert(keyframe_id);
        }
    }

    // 1I: antes de materializar vertices, subdividimos tramos demasiado
    // largos. Si esto se hiciera solo al crear edges, los controles nuevos no
    // existirian como variables y la propagacion seguiria interpolando sobre
    // aristas que representan demasiados KFs.
    std::set<RawKeyFrameId> long_edge_split_keyframes;
    const uint64_t adaptive_vertex_budget = window_keyframes.size();
    bool inserted_split = true;
    while (inserted_split && selected.size() < adaptive_vertex_budget)
    {
        inserted_split = false;
        std::vector<RawKeyFrameId> split_ordered(selected.begin(), selected.end());
        std::sort(split_ordered.begin(), split_ordered.end());
        for (size_t i = 1;
             i < split_ordered.size() && selected.size() < adaptive_vertex_budget;
             ++i)
        {
            const auto& from = split_ordered[i - 1];
            const auto& to = split_ordered[i];
            const uint64_t gap = KeyFrameGap(from, to);
            const double segment_length =
                CumulativeDistanceBetween(cumulative_distance, from, to);
            if (gap <= config_.max_temporal_edge_kf_gap &&
                (config_.max_temporal_edge_length_m <= 0.0 ||
                 segment_length <= config_.max_temporal_edge_length_m))
            {
                continue;
            }
            const double midpoint_distance =
                cumulative_distance[from] + 0.5 * segment_length;
            RawKeyFrameId best = from;
            double best_error = std::numeric_limits<double>::max();
            for (const auto& keyframe_id : window_keyframes)
            {
                if (keyframe_id.local_kf_id <= from.local_kf_id ||
                    keyframe_id.local_kf_id >= to.local_kf_id)
                {
                    continue;
                }
                const double error =
                    std::abs(cumulative_distance[keyframe_id] - midpoint_distance);
                if (error < best_error)
                {
                    best = keyframe_id;
                    best_error = error;
                }
            }
            if (!(best == from) && !(best == to))
            {
                selected.insert(best);
                long_edge_split_keyframes.insert(best);
                inserted_split = true;
                break;
            }
        }
    }

    std::vector<RawKeyFrameId> selected_ordered(selected.begin(), selected.end());
    std::sort(selected_ordered.begin(), selected_ordered.end());

    result.problem.coverage.window_keyframes = window_keyframes.size();
    result.problem.coverage.control_vertices = selected_ordered.size();
    result.problem.coverage.max_vertices_exhausted =
        selected_ordered.size() >= max_vertices &&
        selected_ordered.size() < window_keyframes.size();
    if (selected.find(task.keyframe_id) == selected.end())
    {
        ++result.problem.coverage.mandatory_vertices_missing;
    }
    for (const auto& anchor : branch_anchor_keyframes)
    {
        if (selected.find(anchor) == selected.end())
        {
            ++result.problem.coverage.mandatory_vertices_missing;
        }
    }
    for (size_t i = 1; i < selected_ordered.size(); ++i)
    {
        const auto& from = selected_ordered[i - 1];
        const auto& to = selected_ordered[i];
        const uint64_t gap = KeyFrameGap(from, to);
        const double segment_length =
            CumulativeDistanceBetween(cumulative_distance, from, to);
        result.problem.coverage.max_control_span_kfs =
            std::max<uint64_t>(result.problem.coverage.max_control_span_kfs, gap);
        result.problem.coverage.max_control_span_m =
            std::max<double>(result.problem.coverage.max_control_span_m, segment_length);
        const bool gap_bad = gap > config_.max_temporal_edge_kf_gap;
        const bool length_bad =
            config_.max_temporal_edge_length_m > 0.0 &&
            segment_length > config_.max_temporal_edge_length_m;
        if (gap_bad || length_bad)
        {
            ++result.problem.coverage.uncovered_long_segments;
        }
    }
    result.problem.coverage.coverage_complete =
        result.problem.coverage.mandatory_vertices_missing == 0U &&
        result.problem.coverage.uncovered_long_segments == 0U;
    if (!result.problem.coverage.coverage_complete)
    {
        result.problem.coverage.reason =
            result.problem.coverage.mandatory_vertices_missing > 0U
                ? "mandatory_vertices_missing"
                : "bad_window_coverage";
        result.reason = result.problem.coverage.reason;
        result.problem.summary = Summarize(result.problem);
        return result;
    }
    result.problem.coverage.reason = "coverage_complete";

    std::map<RawKeyFrameId, Eigen::Matrix4d> selected_poses;
    for (const auto& keyframe_id : selected_ordered)
    {
        const auto world_pose = pose_store.GetWorldPose(keyframe_id);
        if (!world_pose)
        {
            continue;
        }

        const bool hard = pose_store.IsHardFiducialKeyFrame(keyframe_id);
        const bool previous_anchor =
            std::find(branch_anchor_keyframes.begin(),
                      branch_anchor_keyframes.end(),
                      keyframe_id) != branch_anchor_keyframes.end();
        const bool anchor_neighborhood =
            anchor_neighborhood_keyframes.find(keyframe_id) !=
            anchor_neighborhood_keyframes.end();
        PoseGraphVertex vertex;
        vertex.keyframe_id = keyframe_id;
        vertex.submap_id = SubmapOf(keyframe_id);
        vertex.initial_world_T_kf = world_pose.value();
        vertex.is_hard_fiducial = hard;
        vertex.is_anchor_neighborhood = anchor_neighborhood;
        // 1I: los vecinos de ambos fiduciales se tratan simetricamente como
        // bloques relativos al fiducial, no como poses absolutas fijas.
        // El fiducial/ancla sigue fijo; su vecindad entra como variable con
        // un lock fuerte en OptimizationManager.
        vertex.is_fixed = hard || previous_anchor;
        vertex.is_variable = !vertex.is_fixed;
        vertex.weight = hard ? config_.fiducial_hard_weight : config_.current_pose_soft_weight;
        vertex.support_count = 1;
        if (keyframe_id == task.keyframe_id)
        {
            vertex.selection_reason = "target_fiducial_error";
        }
        else if (previous_anchor)
        {
            vertex.selection_reason = "previous_fiducial_anchor";
        }
        else if (hard)
        {
            vertex.selection_reason = "hard_fiducial_boundary";
        }
        else if (anchor_neighborhood)
        {
            vertex.selection_reason = "previous_fiducial_neighborhood";
        }
        else if (target_neighborhood_keyframes.find(keyframe_id) !=
                 target_neighborhood_keyframes.end())
        {
            vertex.selection_reason = "target_fiducial_neighborhood";
        }
        else if (long_edge_split_keyframes.find(keyframe_id) !=
                 long_edge_split_keyframes.end())
        {
            vertex.selection_reason = "long_edge_split_vertex";
        }
        else if (keyframe_id == window_keyframes.front() || keyframe_id == window_keyframes.back())
        {
            vertex.selection_reason = "window_boundary";
        }
        else if (selected.find(keyframe_id) != selected.end())
        {
            bool corner = false;
            const auto key_it = std::find(
                window_keyframes.begin(), window_keyframes.end(), keyframe_id);
            if (key_it != window_keyframes.begin() &&
                key_it != window_keyframes.end() &&
                std::next(key_it) != window_keyframes.end())
            {
                const auto prev_it = window_poses.find(*std::prev(key_it));
                const auto curr_it = window_poses.find(keyframe_id);
                const auto next_it = window_poses.find(*std::next(key_it));
                if (prev_it != window_poses.end() &&
                    curr_it != window_poses.end() &&
                    next_it != window_poses.end())
                {
                    const double yaw_prev =
                        NormalizeAngle(YawFromTransform(curr_it->second) -
                                       YawFromTransform(prev_it->second));
                    const double yaw_next =
                        NormalizeAngle(YawFromTransform(next_it->second) -
                                       YawFromTransform(curr_it->second));
                    corner = std::abs(NormalizeAngle(yaw_next - yaw_prev)) >=
                             config_.corner_yaw_threshold_rad;
                }
            }
            vertex.selection_reason =
                corner ? "corner_yaw_vertex" : "distance_sample";
        }
        else
        {
            vertex.selection_reason = "temporal_sample";
        }

        if (vertex.is_fixed)
        {
            result.problem.fixed_keyframes.push_back(keyframe_id);
            if (vertex.selection_reason == "previous_fiducial_anchor")
            {
                ++result.problem.anchor_preservation.previous_fiducial_fixed_count;
            }
            if (vertex.is_anchor_neighborhood)
            {
                ++result.problem.anchor_preservation
                      .previous_fiducial_neighborhood_fixed_count;
            }
        }
        if (vertex.is_variable)
        {
            result.problem.variable_keyframes.push_back(keyframe_id);
        }
        selected_poses[keyframe_id] = world_pose.value();
        result.problem.vertices.push_back(vertex);
    }

    if (result.problem.vertices.size() < config_.min_vertices)
    {
        result.reason = "not_enough_vertices";
        return result;
    }

    if (config_.include_temporal_edges)
    {
        uint64_t edge_id = 1;
        // 1I: las aristas conectan solo vertices de control consecutivos en la
        // ventana sparse. Su soporte resume cuantos KFs crudos quedan entre
        // ambos, dato que 1J usa para ponderar rigidez sin añadir mas vertices.
        for (size_t i = 1; i < selected_ordered.size(); ++i)
        {
            const auto from_it = selected_poses.find(selected_ordered[i - 1]);
            const auto to_it = selected_poses.find(selected_ordered[i]);
            if (from_it == selected_poses.end() || to_it == selected_poses.end())
            {
                continue;
            }

            PoseGraphEdge edge;
            edge.edge_id = edge_id++;
            edge.from_keyframe_id = selected_ordered[i - 1];
            edge.to_keyframe_id = selected_ordered[i];
            edge.edge_type = PoseGraphEdgeType::TemporalNeighbor;
            edge.relative_T_from_to = RelativeTransform(from_it->second, to_it->second);
            const uint64_t gap =
                edge.to_keyframe_id.local_kf_id - edge.from_keyframe_id.local_kf_id;
            const double segment_length =
                CumulativeDistanceBetween(
                    cumulative_distance,
                    edge.from_keyframe_id,
                    edge.to_keyframe_id);
            edge.intermediate_keyframe_count = gap > 0U ? gap - 1U : 0U;
            edge.support_keyframe_count = gap + 1U;
            edge.support_length_m = segment_length;
            edge.support_density_kfs_per_m =
                segment_length > 1e-9
                    ? static_cast<double>(edge.support_keyframe_count) / segment_length
                    : static_cast<double>(edge.support_keyframe_count);
            edge.support_rigidity_multiplier =
                SupportRigidityMultiplier(edge.support_density_kfs_per_m);
            const double base_weight = gap <= 1U
                ? config_.temporal_edge_weight
                : config_.temporal_edge_weight_sparse;
            edge.weight = base_weight * edge.support_rigidity_multiplier;
            edge.source = "F1I_TEMPORAL_WINDOW";
            result.problem.edges.push_back(edge);
        }

        // F1M: las relaciones confirmadas se convierten en restricciones extra
        // solo cuando ambos extremos ya pertenecen a la ventana seleccionada.
        // No crean vertices y no sustituyen las aristas temporales de 1I.
        if (covisibility_db)
        {
            const auto covisibility_edges = covisibility_db->GetEdgesForWindow(
                selected_ordered,
                config_.covisibility_min_weight);
            for (const auto& covisibility_edge : covisibility_edges)
            {
                PoseGraphEdge edge;
                edge.edge_id = edge_id++;
                edge.from_keyframe_id = covisibility_edge.kf_a;
                edge.to_keyframe_id = covisibility_edge.kf_b;
                edge.edge_type = PoseGraphEdgeType::SoftConsistency;
                edge.relative_T_from_to = covisibility_edge.relative_pose_current;
                edge.weight = covisibility_edge.information_weight *
                    config_.covisibility_edge_weight_scale;
                edge.support_keyframe_count =
                    covisibility_edge.shared_mappoints_or_inliers;
                edge.source = std::string("F1M_") +
                    ToString(covisibility_edge.source);
                result.problem.edges.push_back(edge);
            }
        }
    }

    for (const auto& vertex : result.problem.vertices)
    {
        PoseGraphPrior prior;
        prior.keyframe_id = vertex.keyframe_id;
        prior.target_world_T_kf = vertex.initial_world_T_kf;
        prior.source = "F1I_CURRENT_POSE";
        prior.weight_translation = config_.current_pose_soft_weight;
        prior.weight_rotation = config_.current_pose_soft_weight;

        if (vertex.is_hard_fiducial)
        {
            prior.prior_type = PoseGraphPriorType::FiducialHard;
            prior.hard = true;
            prior.weight_translation = config_.fiducial_hard_weight;
            prior.weight_rotation = config_.fiducial_hard_weight;
            prior.source = "F1I_HARD_FIDUCIAL";
        }
        else if (vertex.keyframe_id == task.keyframe_id)
        {
            prior.prior_type = PoseGraphPriorType::FiducialTarget;
            prior.target_world_T_kf = task.target_world_T_kf;
            prior.weight_translation = config_.fiducial_target_translation_weight;
            prior.weight_rotation = config_.fiducial_target_rotation_weight;
            prior.source = "F1I_FIDUCIAL_TARGET";
        }
        result.problem.priors.push_back(prior);
    }

    // 1I: los KFs de la ventana que no son vertices se guardan en un plan de
    // propagacion. 1J calcula una propuesta con este plan y 1K la escribe si
    // el apply pasa sus prechecks.
    for (const auto& keyframe_id : window_keyframes)
    {
        if (selected.find(keyframe_id) != selected.end())
        {
            continue;
        }

        result.problem.affected_non_variable_keyframes.push_back(keyframe_id);

        PoseGraphPropagationPlanEntry entry;
        entry.affected_keyframe_id = keyframe_id;
        entry.submap_id = task.submap_id;
        entry.path_id = task.task_id;
        entry.reason = "window_non_vertex_keyframe";

        RawKeyFrameId before = selected_ordered.front();
        RawKeyFrameId after = selected_ordered.back();
        bool has_before = false;
        bool has_after = false;
        for (const auto& control : selected_ordered)
        {
            if (control.local_kf_id <= keyframe_id.local_kf_id)
            {
                before = control;
                has_before = true;
            }
            if (control.local_kf_id >= keyframe_id.local_kf_id)
            {
                after = control;
                has_after = true;
                break;
            }
        }

        if (has_before && has_after && !(before == after))
        {
            entry.control_vertex_a = before;
            entry.control_vertex_b = after;
            entry.mode = PoseGraphPropagationMode::PathSegment;
            entry.distance_from_a_m =
                CumulativeDistanceBetween(cumulative_distance, before, keyframe_id);
            entry.segment_length_m =
                CumulativeDistanceBetween(cumulative_distance, before, after);
            entry.segment_alpha = entry.segment_length_m > 1e-9
                ? entry.distance_from_a_m / entry.segment_length_m
                : 0.0;
            entry.control_span_kf_gap = KeyFrameGap(before, after);
        }
        else
        {
            entry.control_vertex_a = has_before ? before : after;
            entry.mode = PoseGraphPropagationMode::NearestControlVertex;
            entry.segment_alpha = 0.0;
            entry.distance_from_a_m = 0.0;
            entry.segment_length_m = 0.0;
            entry.control_span_kf_gap = 0;
        }
        result.problem.propagation_plan.push_back(entry);
    }

    result.problem.summary = Summarize(result.problem);
    result.problem.anchor_preservation.satisfied =
        !result.problem.anchor_preservation.required ||
        result.problem.anchor_preservation.previous_fiducial_fixed_count > 0U;
    result.success = true;
    result.reason = "pose_graph_problem_created";
    return result;
}

}  // namespace orbslam3_multi

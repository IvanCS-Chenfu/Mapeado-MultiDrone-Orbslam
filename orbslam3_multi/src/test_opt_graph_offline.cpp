#include "orbslam3_multi/optimization_manager.hpp"
#include "orbslam3_multi/pose_graph_problem_io.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <vector>

namespace
{

double YawFromTransform(const Eigen::Matrix4d& transform)
{
    return std::atan2(transform(1, 0), transform(0, 0));
}

double OpticalYawFromTransform(const Eigen::Matrix4d& transform)
{
    // KeyFrames are camera poses (Twc). In OpenCV/ORB-SLAM camera convention,
    // the viewing direction is the local +Z axis, not the local +X axis used by
    // yaw-from-transform helpers.
    const Eigen::Vector3d optical_axis = transform.block<3, 3>(0, 0).col(2);
    if (optical_axis.head<2>().norm() <= 1e-9)
    {
        return YawFromTransform(transform);
    }
    return std::atan2(optical_axis.y(), optical_axis.x());
}

double NormalizeAngle(double angle)
{
    constexpr double kPi = 3.14159265358979323846;
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

double TranslationError(const Eigen::Matrix4d& a, const Eigen::Matrix4d& b)
{
    return (a.block<3, 1>(0, 3) - b.block<3, 1>(0, 3)).norm();
}

double RotationError3D(const Eigen::Matrix4d& a, const Eigen::Matrix4d& b)
{
    const Eigen::Matrix3d delta =
        a.block<3, 3>(0, 0).transpose() * b.block<3, 3>(0, 0);
    Eigen::AngleAxisd angle_axis(delta);
    return std::abs(NormalizeAngle(angle_axis.angle()));
}

Eigen::Vector3d GlobalTranslationDelta(const Eigen::Matrix4d& from,
                                       const Eigen::Matrix4d& to)
{
    return to.block<3, 1>(0, 3) - from.block<3, 1>(0, 3);
}

double PlanarBearing(const Eigen::Matrix4d& from,
                     const Eigen::Matrix4d& to)
{
    const Eigen::Vector2d delta =
        to.block<2, 1>(0, 3) - from.block<2, 1>(0, 3);
    if (delta.norm() <= 1e-9)
    {
        return 0.0;
    }
    return std::atan2(delta.y(), delta.x());
}

double SignedPlanarTurn(const Eigen::Matrix4d& a,
                        const Eigen::Matrix4d& b,
                        const Eigen::Matrix4d& c)
{
    return NormalizeAngle(PlanarBearing(b, c) - PlanarBearing(a, b));
}

Eigen::Matrix4d ApplyScaledEndpointDelta(
    const Eigen::Matrix4d& pose,
    const Eigen::Matrix4d& endpoint_before,
    const Eigen::Matrix4d& endpoint_after,
    double alpha)
{
    const double bounded_alpha = std::max(0.0, std::min(1.0, alpha));
    Eigen::Matrix4d result = pose;
    const Eigen::Vector3d endpoint_delta_t =
        endpoint_after.block<3, 1>(0, 3) -
        endpoint_before.block<3, 1>(0, 3);
    result.block<3, 1>(0, 3) =
        pose.block<3, 1>(0, 3) + bounded_alpha * endpoint_delta_t;

    Eigen::Quaterniond delta_q(
        endpoint_after.block<3, 3>(0, 0) *
        endpoint_before.block<3, 3>(0, 0).transpose());
    delta_q.normalize();
    const Eigen::Quaterniond scaled_delta =
        Eigen::Quaterniond::Identity().slerp(bounded_alpha, delta_q).normalized();
    result.block<3, 3>(0, 0) =
        scaled_delta.toRotationMatrix() * pose.block<3, 3>(0, 0);
    return result;
}

Eigen::Matrix4d ApplyInterpolatedControlDelta(
    const Eigen::Matrix4d& pose,
    const Eigen::Matrix4d& control_a_before,
    const Eigen::Matrix4d& control_a_after,
    const Eigen::Matrix4d& control_b_before,
    const Eigen::Matrix4d& control_b_after,
    double alpha)
{
    const double bounded_alpha = std::max(0.0, std::min(1.0, alpha));
    const Eigen::Vector3d delta_a_t =
        control_a_after.block<3, 1>(0, 3) -
        control_a_before.block<3, 1>(0, 3);
    const Eigen::Vector3d delta_b_t =
        control_b_after.block<3, 1>(0, 3) -
        control_b_before.block<3, 1>(0, 3);

    Eigen::Quaterniond delta_a_q(
        control_a_after.block<3, 3>(0, 0) *
        control_a_before.block<3, 3>(0, 0).transpose());
    Eigen::Quaterniond delta_b_q(
        control_b_after.block<3, 3>(0, 0) *
        control_b_before.block<3, 3>(0, 0).transpose());
    delta_a_q.normalize();
    delta_b_q.normalize();

    Eigen::Matrix4d result = pose;
    result.block<3, 1>(0, 3) =
        pose.block<3, 1>(0, 3) +
        (1.0 - bounded_alpha) * delta_a_t + bounded_alpha * delta_b_t;
    const Eigen::Quaterniond interpolated_delta =
        delta_a_q.slerp(bounded_alpha, delta_b_q).normalized();
    result.block<3, 3>(0, 0) =
        interpolated_delta.toRotationMatrix() * pose.block<3, 3>(0, 0);
    return result;
}

void PrintUsage()
{
    std::cout
        << "Uso: test_opt_graph_offline --graph <path> [--html <path>]\n"
        << "Ejecuta OptimizationManager::RunDryRunGraphOnly sobre un dump F1L.\n";
}

struct OfflinePoint
{
    orbslam3_multi::RawKeyFrameId id;
    Eigen::Matrix4d before = Eigen::Matrix4d::Identity();
    std::optional<Eigen::Matrix4d> after;
    std::optional<Eigen::Matrix4d> gt;
    std::string gt_quality = "MISSING";
    double gt_dt_sec = -1.0;
    bool is_vertex = false;
    bool is_fixed = false;
    bool is_target = false;
};

std::set<orbslam3_multi::RawKeyFrameId> FiducialNeighborhoodLockedKeyFrames(
    const orbslam3_multi::PoseGraphProblem& graph)
{
    constexpr uint64_t kLockedTailEdgeCount = 3;
    std::set<orbslam3_multi::RawKeyFrameId> locked;

    std::optional<orbslam3_multi::RawKeyFrameId> previous_anchor;
    for (const auto& vertex : graph.vertices)
    {
        if (vertex.selection_reason == "previous_fiducial_anchor")
        {
            previous_anchor = vertex.keyframe_id;
            break;
        }
    }
    if (previous_anchor)
    {
        std::optional<size_t> anchor_edge_index;
        for (size_t i = 0; i < graph.edges.size(); ++i)
        {
            const auto& edge = graph.edges[i];
            if (edge.from_keyframe_id == previous_anchor.value() ||
                edge.to_keyframe_id == previous_anchor.value())
            {
                anchor_edge_index = i;
                break;
            }
        }

        orbslam3_multi::RawKeyFrameId current = previous_anchor.value();
        if (anchor_edge_index)
        {
            size_t edge_index = anchor_edge_index.value();
            for (uint64_t step = 0; step < kLockedTailEdgeCount; ++step)
            {
                const auto& edge = graph.edges[edge_index];
                if (edge.from_keyframe_id == current)
                {
                    locked.insert(edge.to_keyframe_id);
                    current = edge.to_keyframe_id;
                }
                else if (edge.to_keyframe_id == current)
                {
                    locked.insert(edge.from_keyframe_id);
                    current = edge.from_keyframe_id;
                }
                else
                {
                    break;
                }

                ++edge_index;
                if (edge_index >= graph.edges.size())
                {
                    break;
                }
            }
        }
    }

    std::optional<size_t> target_edge_index;
    for (size_t i = 0; i < graph.edges.size(); ++i)
    {
        const auto& edge = graph.edges[i];
        if (edge.from_keyframe_id == graph.target_keyframe_id ||
            edge.to_keyframe_id == graph.target_keyframe_id)
        {
            target_edge_index = i;
            break;
        }
    }
    if (target_edge_index)
    {
        orbslam3_multi::RawKeyFrameId current = graph.target_keyframe_id;
        size_t edge_index = target_edge_index.value();
        for (uint64_t step = 0; step < kLockedTailEdgeCount; ++step)
        {
            const auto& edge = graph.edges[edge_index];
            if (edge.to_keyframe_id == current)
            {
                locked.insert(edge.from_keyframe_id);
                current = edge.from_keyframe_id;
            }
            else if (edge.from_keyframe_id == current)
            {
                locked.insert(edge.to_keyframe_id);
                current = edge.to_keyframe_id;
            }
            else
            {
                break;
            }

            if (edge_index == 0U)
            {
                break;
            }
            --edge_index;
        }
    }

    return locked;
}

std::map<orbslam3_multi::RawKeyFrameId, OfflinePoint> BuildOfflinePoints(
    const orbslam3_multi::PoseGraphProblem& graph,
    const orbslam3_multi::OptimizationDryRunResult& result)
{
    std::map<orbslam3_multi::RawKeyFrameId, OfflinePoint> points;
    const auto locked_neighborhood_keyframes =
        FiducialNeighborhoodLockedKeyFrames(graph);
    auto ensure = [&](const orbslam3_multi::RawKeyFrameId& id) -> OfflinePoint&
    {
        auto& point = points[id];
        point.id = id;
        return point;
    };

    for (const auto& debug : graph.debug_keyframes)
    {
        auto& point = ensure(debug.keyframe_id);
        point.before = debug.map_world_T_kf;
        point.gt_quality = debug.association_quality;
        point.gt_dt_sec = debug.association_dt_sec;
        if (debug.has_gt)
        {
            point.gt = debug.gt_world_T_kf;
        }
    }
    for (const auto& vertex : graph.vertices)
    {
        auto& point = ensure(vertex.keyframe_id);
        point.before = vertex.initial_world_T_kf;
        point.is_vertex = true;
        point.is_fixed =
            vertex.is_fixed || vertex.is_hard_fiducial ||
            locked_neighborhood_keyframes.find(vertex.keyframe_id) !=
                locked_neighborhood_keyframes.end();
        point.is_target = vertex.keyframe_id == graph.target_keyframe_id;
    }
    for (const auto& proposal : result.proposed_vertex_poses)
    {
        auto& point = ensure(proposal.keyframe_id);
        point.before = proposal.before_world_T_kf;
        point.after = proposal.proposed_world_T_kf;
        point.is_vertex = true;
        point.is_fixed = point.is_fixed || proposal.fixed_vertex;
        point.is_target = point.is_target ||
            proposal.keyframe_id == graph.target_keyframe_id;
    }

    std::map<orbslam3_multi::RawKeyFrameId, Eigen::Matrix4d> before_poses;
    std::map<orbslam3_multi::RawKeyFrameId, Eigen::Matrix4d> after_poses;
    for (const auto& [id, point] : points)
    {
        before_poses[id] = point.before;
        if (point.after.has_value())
        {
            after_poses[id] = point.after.value();
        }
    }

    for (const auto& entry : graph.propagation_plan)
    {
        const auto before_it = before_poses.find(entry.affected_keyframe_id);
        const auto control_a_before_it = before_poses.find(entry.control_vertex_a);
        const auto control_a_after_it = after_poses.find(entry.control_vertex_a);
        if (before_it == before_poses.end() ||
            control_a_before_it == before_poses.end() ||
            control_a_after_it == after_poses.end())
        {
            continue;
        }

        Eigen::Matrix4d proposed =
            ApplyScaledEndpointDelta(
                before_it->second,
                control_a_before_it->second,
                control_a_after_it->second,
                1.0);
        if (entry.mode == orbslam3_multi::PoseGraphPropagationMode::PathSegment &&
            entry.control_vertex_b)
        {
            const auto control_b_before_it =
                before_poses.find(entry.control_vertex_b.value());
            const auto control_b_after_it =
                after_poses.find(entry.control_vertex_b.value());
            if (control_b_before_it != before_poses.end() &&
                control_b_after_it != after_poses.end())
            {
                proposed = ApplyInterpolatedControlDelta(
                    before_it->second,
                    control_a_before_it->second,
                    control_a_after_it->second,
                    control_b_before_it->second,
                    control_b_after_it->second,
                    entry.segment_alpha);
            }
        }
        ensure(entry.affected_keyframe_id).after = proposed;
    }

    return points;
}

double EdgeCenterAlphaInWindow(const orbslam3_multi::PoseGraphProblem& graph,
                               const orbslam3_multi::PoseGraphEdge& edge)
{
    if (graph.vertices.empty())
    {
        return 0.5;
    }

    uint64_t min_kf = std::numeric_limits<uint64_t>::max();
    uint64_t max_kf = 0;
    for (const auto& vertex : graph.vertices)
    {
        min_kf = std::min(min_kf, vertex.keyframe_id.local_kf_id);
        max_kf = std::max(max_kf, vertex.keyframe_id.local_kf_id);
    }
    if (min_kf == std::numeric_limits<uint64_t>::max() || max_kf <= min_kf)
    {
        return 0.5;
    }

    const double center =
        0.5 * (static_cast<double>(edge.from_keyframe_id.local_kf_id) +
               static_cast<double>(edge.to_keyframe_id.local_kf_id));
    const double alpha =
        (center - static_cast<double>(min_kf)) /
        static_cast<double>(max_kf - min_kf);
    return std::max(0.0, std::min(1.0, alpha));
}

double SparseEdgeTranslationRigidityForLog(
    const orbslam3_multi::PoseGraphProblem& graph,
    const orbslam3_multi::PoseGraphEdge& edge)
{
    const auto from_it =
        std::find_if(
            graph.vertices.begin(),
            graph.vertices.end(),
            [&](const orbslam3_multi::PoseGraphVertex& vertex)
            {
                return vertex.keyframe_id == edge.from_keyframe_id;
            });
    const auto to_it =
        std::find_if(
            graph.vertices.begin(),
            graph.vertices.end(),
            [&](const orbslam3_multi::PoseGraphVertex& vertex)
            {
                return vertex.keyframe_id == edge.to_keyframe_id;
            });
    if (from_it == graph.vertices.end() || to_it == graph.vertices.end())
    {
        return 1.0;
    }

    const double alpha = EdgeCenterAlphaInWindow(graph, edge);
    const double center_flexibility =
        std::max(0.0, std::min(1.0, 4.0 * alpha * (1.0 - alpha)));
    const double low_support =
        std::max(
            0.0,
            std::min(
                1.0,
                (1.10 - edge.support_rigidity_multiplier) / (1.10 - 0.65)));
    const Eigen::Vector3d global_delta =
        GlobalTranslationDelta(
            from_it->initial_world_T_kf,
            to_it->initial_world_T_kf);
    const double global_xy = global_delta.head<2>().norm();
    const double local_xy =
        edge.relative_T_from_to.block<2, 1>(0, 3).norm();
    const double local_global_xy_ratio =
        global_xy > 1e-6 ? local_xy / global_xy : 1.0;
    const double projection_conflict =
        std::max(
            0.0,
            std::min(
                1.0,
                (0.55 - local_global_xy_ratio) / (0.55 - 0.20)));
    return std::max(
        0.08,
        1.0 - 0.55 * center_flexibility * low_support *
                  projection_conflict);
}

void PrintEdgeSupport(const orbslam3_multi::PoseGraphProblem& graph)
{
    if (graph.edges.empty())
    {
        std::cout << "[F1L-OFFLINE-EDGE-SUPPORT] edges=0\n";
        return;
    }

    double sum_density = 0.0;
    double sum_multiplier = 0.0;
    double min_multiplier = std::numeric_limits<double>::infinity();
    double max_multiplier = 0.0;
    for (const auto& edge : graph.edges)
    {
        sum_density += edge.support_density_kfs_per_m;
        sum_multiplier += edge.support_rigidity_multiplier;
        min_multiplier = std::min(min_multiplier, edge.support_rigidity_multiplier);
        max_multiplier = std::max(max_multiplier, edge.support_rigidity_multiplier);
        const double sparse_translation_rigidity =
            SparseEdgeTranslationRigidityForLog(graph, edge);
        std::cout << "[F1L-OFFLINE-EDGE-SUPPORT-EDGE] edge_id=" << edge.edge_id
                  << " from_kf=" << edge.from_keyframe_id.local_kf_id
                  << " to_kf=" << edge.to_keyframe_id.local_kf_id
                  << " intermediate_kfs=" << edge.intermediate_keyframe_count
                  << " support_kfs=" << edge.support_keyframe_count
                  << " length_m=" << edge.support_length_m
                  << " density_kfs_per_m=" << edge.support_density_kfs_per_m
                  << " multiplier=" << edge.support_rigidity_multiplier
                  << " sparse_translation_rigidity="
                  << sparse_translation_rigidity
                  << " weight=" << edge.weight
                  << "\n";
    }

    const double n = static_cast<double>(graph.edges.size());
    std::cout << "[F1L-OFFLINE-EDGE-SUPPORT] edges=" << graph.edges.size()
              << " mean_density_kfs_per_m=" << (sum_density / n)
              << " mean_multiplier=" << (sum_multiplier / n)
              << " min_multiplier=" << min_multiplier
              << " max_multiplier=" << max_multiplier
              << "\n";
}

void PrintEdgeDeformation(
    const orbslam3_multi::PoseGraphProblem& graph,
    const std::map<orbslam3_multi::RawKeyFrameId, OfflinePoint>& points)
{
    for (const auto& edge : graph.edges)
    {
        const auto from_it = points.find(edge.from_keyframe_id);
        const auto to_it = points.find(edge.to_keyframe_id);
        if (from_it == points.end() || to_it == points.end() ||
            !from_it->second.after.has_value() ||
            !to_it->second.after.has_value())
        {
            continue;
        }

        const Eigen::Matrix4d before_relative =
            from_it->second.before.inverse() * to_it->second.before;
        const Eigen::Matrix4d after_relative =
            from_it->second.after.value().inverse() * to_it->second.after.value();
        const Eigen::Vector3d before_map_delta =
            GlobalTranslationDelta(from_it->second.before, to_it->second.before);
        const Eigen::Vector3d after_map_delta =
            GlobalTranslationDelta(
                from_it->second.after.value(), to_it->second.after.value());
        const double before_len =
            before_relative.block<3, 1>(0, 3).norm();
        const double after_len =
            after_relative.block<3, 1>(0, 3).norm();
        const double before_xy_len =
            before_relative.block<2, 1>(0, 3).norm();
        const double after_xy_len =
            after_relative.block<2, 1>(0, 3).norm();
        const double before_z = before_relative(2, 3);
        const double after_z = after_relative(2, 3);
        double gt_len = -1.0;
        double gt_xy_len = -1.0;
        double gt_delta_z = 0.0;
        if (from_it->second.gt.has_value() && to_it->second.gt.has_value())
        {
            const Eigen::Vector3d gt_delta =
                GlobalTranslationDelta(
                    from_it->second.gt.value(), to_it->second.gt.value());
            gt_len = gt_delta.norm();
            gt_xy_len = gt_delta.head<2>().norm();
            gt_delta_z = gt_delta.z();
        }
        const double before_yaw = YawFromTransform(before_relative);
        const double after_yaw = YawFromTransform(after_relative);
        const Eigen::Vector3d correction_from =
            from_it->second.after.value().block<3, 1>(0, 3) -
            from_it->second.before.block<3, 1>(0, 3);
        const Eigen::Vector3d correction_to =
            to_it->second.after.value().block<3, 1>(0, 3) -
            to_it->second.before.block<3, 1>(0, 3);
        const double correction_yaw_from =
            NormalizeAngle(
                YawFromTransform(from_it->second.after.value()) -
                YawFromTransform(from_it->second.before));
        const double correction_yaw_to =
            NormalizeAngle(
                YawFromTransform(to_it->second.after.value()) -
                YawFromTransform(to_it->second.before));
        std::cout << "[F1L-OFFLINE-EDGE-DEFORMATION] edge_id="
                  << edge.edge_id
                  << " from_kf=" << edge.from_keyframe_id.local_kf_id
                  << " to_kf=" << edge.to_keyframe_id.local_kf_id
                  << " support_kfs=" << edge.support_keyframe_count
                  << " multiplier=" << edge.support_rigidity_multiplier
                  << " before_len=" << before_len
                  << " after_len=" << after_len
                  << " delta_len=" << (after_len - before_len)
                  << " before_xy_len=" << before_xy_len
                  << " after_xy_len=" << after_xy_len
                  << " delta_xy_len=" << (after_xy_len - before_xy_len)
                  << " before_rel_z=" << before_z
                  << " after_rel_z=" << after_z
                  << " delta_rel_z=" << (after_z - before_z)
                  << " before_map_len=" << before_map_delta.norm()
                  << " after_map_len=" << after_map_delta.norm()
                  << " delta_map_len="
                  << (after_map_delta.norm() - before_map_delta.norm())
                  << " before_map_xy_len="
                  << before_map_delta.head<2>().norm()
                  << " after_map_xy_len="
                  << after_map_delta.head<2>().norm()
                  << " delta_map_xy_len="
                  << (after_map_delta.head<2>().norm() -
                      before_map_delta.head<2>().norm())
                  << " before_map_delta_z=" << before_map_delta.z()
                  << " after_map_delta_z=" << after_map_delta.z()
                  << " delta_map_delta_z="
                  << (after_map_delta.z() - before_map_delta.z())
                  << " gt_map_len=" << gt_len
                  << " gt_map_xy_len=" << gt_xy_len
                  << " gt_map_delta_z=" << gt_delta_z
                  << " before_rel_yaw=" << before_yaw
                  << " after_rel_yaw=" << after_yaw
                  << " delta_rel_yaw=" << NormalizeAngle(after_yaw - before_yaw)
                  << " correction_gradient_t="
                  << (correction_to - correction_from).norm()
                  << " correction_gradient_yaw="
                  << NormalizeAngle(correction_yaw_to - correction_yaw_from)
                  << "\n";
    }
}

void PrintCornerDeformation(
    const orbslam3_multi::PoseGraphProblem& graph,
    const std::map<orbslam3_multi::RawKeyFrameId, OfflinePoint>& points)
{
    for (size_t i = 1; i < graph.edges.size(); ++i)
    {
        const auto& previous_edge = graph.edges[i - 1];
        const auto& next_edge = graph.edges[i];
        if (!(previous_edge.to_keyframe_id == next_edge.from_keyframe_id))
        {
            continue;
        }

        const auto a_it = points.find(previous_edge.from_keyframe_id);
        const auto b_it = points.find(previous_edge.to_keyframe_id);
        const auto c_it = points.find(next_edge.to_keyframe_id);
        if (a_it == points.end() || b_it == points.end() ||
            c_it == points.end() || !a_it->second.after.has_value() ||
            !b_it->second.after.has_value() || !c_it->second.after.has_value())
        {
            continue;
        }

        const double before_in =
            PlanarBearing(a_it->second.before, b_it->second.before);
        const double before_out =
            PlanarBearing(b_it->second.before, c_it->second.before);
        const double after_in =
            PlanarBearing(a_it->second.after.value(), b_it->second.after.value());
        const double after_out =
            PlanarBearing(b_it->second.after.value(), c_it->second.after.value());
        const double before_turn =
            SignedPlanarTurn(
                a_it->second.before,
                b_it->second.before,
                c_it->second.before);
        const double after_turn =
            SignedPlanarTurn(
                a_it->second.after.value(),
                b_it->second.after.value(),
                c_it->second.after.value());
        const bool sign_flipped =
            before_turn * after_turn < 0.0 &&
            std::abs(before_turn) > 0.10 &&
            std::abs(after_turn) > 0.10;
        std::cout << "[F1L-OFFLINE-CORNER-DEFORMATION] center_kf="
                  << previous_edge.to_keyframe_id.local_kf_id
                  << " prev_kf=" << previous_edge.from_keyframe_id.local_kf_id
                  << " next_kf=" << next_edge.to_keyframe_id.local_kf_id
                  << " prev_support_kfs=" << previous_edge.support_keyframe_count
                  << " next_support_kfs=" << next_edge.support_keyframe_count
                  << " before_in_bearing=" << before_in
                  << " before_out_bearing=" << before_out
                  << " before_turn=" << before_turn
                  << " after_in_bearing=" << after_in
                  << " after_out_bearing=" << after_out
                  << " after_turn=" << after_turn
                  << " delta_turn=" << NormalizeAngle(after_turn - before_turn)
                  << " sign_flipped=" << (sign_flipped ? "true" : "false")
                  << "\n";
    }
}

void PrintGtStats(const std::map<orbslam3_multi::RawKeyFrameId, OfflinePoint>& points)
{
    uint64_t valid_before = 0;
    uint64_t valid_after = 0;
    uint64_t worsened = 0;
    double sum_before = 0.0;
    double sum_after = 0.0;
    double max_before = 0.0;
    double max_after = 0.0;
    for (const auto& [id, point] : points)
    {
        (void)id;
        if (!point.gt.has_value())
        {
            continue;
        }
        const double before_error = TranslationError(point.before, point.gt.value());
        ++valid_before;
        sum_before += before_error;
        max_before = std::max(max_before, before_error);
        if (point.after.has_value())
        {
            const double after_error =
                TranslationError(point.after.value(), point.gt.value());
            ++valid_after;
            sum_after += after_error;
            max_after = std::max(max_after, after_error);
            if (after_error > before_error + 0.05)
            {
                ++worsened;
            }
        }
    }

    std::cout << "[F1L-OFFLINE-GT-STATS] valid_before=" << valid_before
              << " valid_after=" << valid_after
              << " mean_before=" << (valid_before > 0 ? sum_before / valid_before : 0.0)
              << " max_before=" << max_before
              << " mean_after=" << (valid_after > 0 ? sum_after / valid_after : 0.0)
              << " max_after=" << max_after
              << " worsened_kfs=" << worsened
              << "\n";
}

bool ExportHtml(const orbslam3_multi::PoseGraphProblem& graph,
                const orbslam3_multi::OptimizationDryRunResult& result,
                const std::map<orbslam3_multi::RawKeyFrameId, OfflinePoint>& points,
                const std::string& html_path,
                std::string& reason)
{
    if (points.empty())
    {
        reason = "empty_points";
        return false;
    }

    double min_x = std::numeric_limits<double>::infinity();
    double max_x = -std::numeric_limits<double>::infinity();
    double min_y = std::numeric_limits<double>::infinity();
    double max_y = -std::numeric_limits<double>::infinity();
    auto observe = [&](const Eigen::Matrix4d& pose)
    {
        min_x = std::min(min_x, pose(0, 3));
        max_x = std::max(max_x, pose(0, 3));
        min_y = std::min(min_y, pose(1, 3));
        max_y = std::max(max_y, pose(1, 3));
    };
    uint64_t vertex_count = 0;
    uint64_t gt_count = 0;
    uint64_t after_count = 0;
    for (const auto& [id, point] : points)
    {
        (void)id;
        observe(point.before);
        if (point.gt.has_value())
        {
            observe(point.gt.value());
            ++gt_count;
        }
        if (point.after.has_value())
        {
            observe(point.after.value());
            ++after_count;
        }
        if (point.is_vertex)
        {
            ++vertex_count;
        }
    }
    if (!std::isfinite(min_x) || !std::isfinite(min_y))
    {
        reason = "non_finite_bounds";
        return false;
    }

    min_x -= 1.0;
    max_x += 1.0;
    min_y -= 1.0;
    max_y += 1.0;
    const double width = std::max(1.0, max_x - min_x);
    const double height = std::max(1.0, max_y - min_y);
    const double canvas = 1000.0;
    auto sx = [&](double x) { return 50.0 + (x - min_x) / width * (canvas - 100.0); };
    auto sy = [&](double y) { return canvas - 80.0 - (y - min_y) / height * (canvas - 140.0); };

    std::ofstream out(html_path);
    if (!out)
    {
        reason = "open_failed";
        return false;
    }

    auto write_circle = [&](const OfflinePoint& point,
                            const Eigen::Matrix4d& pose,
                            const std::string& fill,
                            double radius,
                            double opacity)
    {
        out << "<circle cx=\"" << sx(pose(0, 3))
            << "\" cy=\"" << sy(pose(1, 3))
            << "\" r=\"" << radius
            << "\" fill=\"" << fill
            << "\" opacity=\"" << opacity
            << "\"><title>kf=" << point.id.local_kf_id
            << " x=" << pose(0, 3)
            << " y=" << pose(1, 3)
            << " x_axis_yaw=" << YawFromTransform(pose)
            << " optical_yaw=" << OpticalYawFromTransform(pose)
            << "</title></circle>\n";
    };
    auto write_heading = [&](const OfflinePoint& point,
                             const Eigen::Matrix4d& pose,
                             const std::string& color,
                             double length_m,
                             double opacity)
    {
        const double yaw = OpticalYawFromTransform(pose);
        const double x2 = pose(0, 3) + length_m * std::cos(yaw);
        const double y2 = pose(1, 3) + length_m * std::sin(yaw);
        out << "<line x1=\"" << sx(pose(0, 3))
            << "\" y1=\"" << sy(pose(1, 3))
            << "\" x2=\"" << sx(x2)
            << "\" y2=\"" << sy(y2)
            << "\" stroke=\"" << color
            << "\" stroke-width=\"" << (point.is_target ? 3.0 : 2.0)
            << "\" opacity=\"" << opacity
            << "\" marker-end=\"url(#arrow)\"><title>heading kf="
            << point.id.local_kf_id
            << " optical_yaw=" << yaw
            << " x_axis_yaw=" << YawFromTransform(pose)
            << "</title></line>\n";
    };
    auto write_svg_open = [&]()
    {
        out << "<svg width=\"1000\" height=\"1000\" viewBox=\"0 0 1000 1000\">\n";
        out << "<rect width=\"1000\" height=\"1000\" fill=\"white\"/>\n";
        out << "<defs><marker id=\"arrow\" viewBox=\"0 0 10 10\" refX=\"8\" refY=\"5\""
               " markerWidth=\"5\" markerHeight=\"5\" orient=\"auto-start-reverse\">"
               "<path d=\"M 0 0 L 10 5 L 0 10 z\" fill=\"context-stroke\"/>"
               "</marker></defs>\n";
        out << "<text x=\"30\" y=\"970\" font-size=\"14\">XY projection. GT is diagnostic only and never feeds the solver.</text>\n";
    };
    auto write_edges = [&](const std::string& color, bool optimized)
    {
        for (const auto& edge : graph.edges)
        {
            const auto from_it = points.find(edge.from_keyframe_id);
            const auto to_it = points.find(edge.to_keyframe_id);
            if (from_it == points.end() || to_it == points.end())
            {
                continue;
            }
            const auto& from_pose =
                optimized && from_it->second.after.has_value()
                    ? from_it->second.after.value()
                    : from_it->second.before;
            const auto& to_pose =
                optimized && to_it->second.after.has_value()
                    ? to_it->second.after.value()
                    : to_it->second.before;
            out << "<line x1=\"" << sx(from_pose(0, 3))
                << "\" y1=\"" << sy(from_pose(1, 3))
                << "\" x2=\"" << sx(to_pose(0, 3))
                << "\" y2=\"" << sy(to_pose(1, 3))
                << "\" stroke=\"" << color
                << "\" stroke-width=\"1.5\" opacity=\"0.55\"><title>edge "
                << edge.edge_id << " support=" << edge.support_keyframe_count
                << " density=" << edge.support_density_kfs_per_m
                << " mult=" << edge.support_rigidity_multiplier
                << "</title></line>\n";
        }
    };

    out << std::setprecision(8);
    out << "<!doctype html><html><head><meta charset=\"utf-8\"/>\n";
    out << "<title>F1L offline graph task " << graph.task_id << "</title>\n";
    out << "<style>body{font-family:sans-serif;margin:0;background:#f7f7f4;color:#1f2933;}"
           "header{padding:12px 18px;background:#111827;color:white;}"
           "button{margin:8px 4px;padding:8px 12px;border:1px solid #999;background:white;cursor:pointer;}"
           "button.active{background:#1d4ed8;color:white;border-color:#1d4ed8;}"
           ".wrap{padding:12px 18px}.frame{display:none}.frame.active{display:block}"
           "svg{background:white;border:1px solid #d1d5db;max-width:100%;height:auto}"
           ".legend span{display:inline-block;margin-right:16px;font-size:13px}</style>\n";
    out << "</head><body><header><strong>F1L offline task " << graph.task_id
        << "</strong> &nbsp; before=" << result.before_error_t
        << " after=" << result.after_error_t
        << " useful=" << (result.useful ? "true" : "false")
        << "</header><div class=\"wrap\">\n";
    out << "<div><button id=\"b0\" class=\"active\" onclick=\"showFrame(0)\">Inicial</button>"
           "<button id=\"b1\" onclick=\"showFrame(1)\">Grafo</button>"
           "<button id=\"b2\" onclick=\"showFrame(2)\">Optimizado</button>"
           "<button onclick=\"togglePlay()\">Play/Pausa</button></div>\n";
    out << "<p class=\"legend\"><span style=\"color:#dc2626\">rojo: mapa antes</span>"
           "<span style=\"color:#111\">negro: GT</span>"
           "<span style=\"color:#7c3aed\">morado: vertices antes</span>"
           "<span style=\"color:#f97316\">naranja: fijo/fiducial</span>"
           "<span style=\"color:#2563eb\">azul: KFs propagados offline</span>"
           "<span style=\"color:#16a34a\">verde: vertices optimizados</span></p>\n";

    out << "<div id=\"f0\" class=\"frame active\">";
    write_svg_open();
    for (const auto& [id, point] : points)
    {
        (void)id;
        write_circle(point, point.before, "#dc2626", 3.2, 0.80);
        if (point.gt.has_value())
        {
            write_circle(point, point.gt.value(), "#111111", 2.7, 0.70);
        }
    }
    out << "</svg></div>\n";

    out << "<div id=\"f1\" class=\"frame\">";
    write_svg_open();
    write_edges("#7c3aed", false);
    for (const auto& [id, point] : points)
    {
        (void)id;
        if (point.gt.has_value())
        {
            write_circle(point, point.gt.value(), "#111111", 2.2, 0.25);
        }
        write_circle(point, point.before, "#dc2626", 2.4, 0.30);
        if (point.is_vertex)
        {
            write_heading(point, point.before,
                          point.is_fixed ? "#f97316" : "#7c3aed",
                          point.is_target ? 0.85 : 0.55,
                          0.85);
            write_circle(point, point.before,
                         point.is_fixed ? "#f97316" : "#7c3aed",
                         point.is_target ? 10.0 : 7.0,
                         0.95);
        }
    }
    out << "</svg></div>\n";

    out << "<div id=\"f2\" class=\"frame\">";
    write_svg_open();
    write_edges("#16a34a", true);
    for (const auto& [id, point] : points)
    {
        (void)id;
        if (point.gt.has_value())
        {
            write_circle(point, point.gt.value(), "#111111", 2.5, 0.35);
        }
        if (point.after.has_value())
        {
            if (point.is_vertex)
            {
                write_heading(point, point.after.value(),
                              "#16a34a",
                              point.is_target ? 0.85 : 0.55,
                              0.85);
            }
            write_circle(point, point.after.value(),
                         point.is_vertex ? "#16a34a" : "#2563eb",
                         point.is_vertex ? (point.is_target ? 10.0 : 7.0) : 3.5,
                         0.90);
        }
    }
    out << "</svg></div>\n";

    out << "<p>window_kfs=" << points.size()
        << " vertices=" << vertex_count
        << " gt_valid=" << gt_count
        << " optimized_or_propagated=" << after_count
        << "</p>\n";
    out << "<script>let frame=0,playing=true;function showFrame(i){frame=i;"
           "for(let n=0;n<3;n++){document.getElementById('f'+n).classList.toggle('active',n===i);"
           "document.getElementById('b'+n).classList.toggle('active',n===i);}}"
           "function togglePlay(){playing=!playing;}setInterval(()=>{if(playing)showFrame((frame+1)%3);},1400);</script>\n";
    out << "</div></body></html>\n";
    reason = "ok";
    return true;
}

}  // namespace

int main(int argc, char** argv)
{
    std::string graph_path;
    std::string html_path;
    for (int i = 1; i < argc; ++i)
    {
        const std::string arg(argv[i]);
        if (arg == "--graph" && i + 1 < argc)
        {
            graph_path = argv[++i];
        }
        else if (arg == "--html" && i + 1 < argc)
        {
            html_path = argv[++i];
        }
        else if (arg == "--help" || arg == "-h")
        {
            PrintUsage();
            return 0;
        }
    }

    if (graph_path.empty())
    {
        PrintUsage();
        std::cerr << "[F1L-OFFLINE-ERROR] reason=missing_graph\n";
        return 2;
    }

    orbslam3_multi::PoseGraphProblem graph;
    const auto load = orbslam3_multi::LoadPoseGraphProblem(graph_path, graph);
    if (!load.success)
    {
        std::cerr << "[F1L-OFFLINE-LOAD] success=false path=" << graph_path
                  << " reason=" << load.reason << "\n";
        return 3;
    }

    orbslam3_multi::OptimizationManager optimizer;
    const auto result = optimizer.RunDryRunGraphOnly(graph);
    const auto points = BuildOfflinePoints(graph, result);

    std::cout << "[F1L-OFFLINE-LOAD] success=true path=" << graph_path
              << " task_id=" << graph.task_id
              << " vertices=" << graph.vertices.size()
              << " edges=" << graph.edges.size()
              << " priors=" << graph.priors.size()
              << " propagation=" << graph.propagation_plan.size()
              << " debug_kfs=" << graph.debug_keyframes.size()
              << "\n";
    PrintEdgeSupport(graph);
    std::cout << "[F1L-OFFLINE-DRYRUN] success="
              << (result.success ? "true" : "false")
              << " precheck_ok=" << (result.precheck_ok ? "true" : "false")
              << " useful=" << (result.useful ? "true" : "false")
              << " partial_candidate=" << (result.partial_candidate ? "true" : "false")
              << " reason=" << result.reason
              << " decision_reason=" << result.decision_reason
              << " iterations=" << result.solver_iterations
              << "\n";
    std::cout << "[F1L-OFFLINE-METRICS] before_t=" << result.before_error_t
              << " after_t=" << result.after_error_t
              << " before_yaw=" << result.before_error_yaw
              << " after_yaw=" << result.after_error_yaw
              << " improvement_ratio=" << result.improvement_ratio
              << " initial_cost=" << result.initial_cost
              << " final_cost=" << result.final_cost
              << " moved_kfs=" << result.moved_kfs
              << " max_delta_t=" << result.max_delta_t
              << " mean_delta_t=" << result.mean_delta_t
              << " max_delta_yaw=" << result.max_delta_yaw
              << " proposed_vertices=" << result.proposed_vertex_poses.size()
              << " proposed_propagated=" << result.proposed_propagated_poses.size()
              << "\n";
    const auto target_prior_it =
        std::find_if(
            graph.priors.begin(),
            graph.priors.end(),
            [](const orbslam3_multi::PoseGraphPrior& prior)
            {
                return prior.prior_type ==
                       orbslam3_multi::PoseGraphPriorType::FiducialTarget;
            });
    if (target_prior_it != graph.priors.end())
    {
        const auto point_it = points.find(target_prior_it->keyframe_id);
        if (point_it != points.end() && point_it->second.after.has_value())
        {
            std::cout << "[F1L-OFFLINE-TARGET-POSE-3D] before_rotation="
                      << RotationError3D(
                             target_prior_it->target_world_T_kf,
                             point_it->second.before)
                      << " after_rotation="
                      << RotationError3D(
                             target_prior_it->target_world_T_kf,
                             point_it->second.after.value())
                      << " before_translation="
                      << TranslationError(
                             point_it->second.before,
                             target_prior_it->target_world_T_kf)
                      << " after_translation="
                      << TranslationError(
                             point_it->second.after.value(),
                             target_prior_it->target_world_T_kf)
                      << "\n";
        }
    }
    PrintEdgeDeformation(graph, points);
    PrintCornerDeformation(graph, points);
    PrintGtStats(points);

    if (!html_path.empty())
    {
        std::string reason;
        const bool html_ok = ExportHtml(graph, result, points, html_path, reason);
        std::cout << "[F1L-OFFLINE-HTML] success="
                  << (html_ok ? "true" : "false")
                  << " path=" << html_path
                  << " reason=" << reason
                  << " points=" << points.size()
                  << "\n";
        if (!html_ok)
        {
            return 5;
        }
    }

    return result.success ? 0 : 4;
}

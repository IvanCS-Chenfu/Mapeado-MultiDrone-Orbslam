#include "orbslam3_multi/optimization_manager.hpp"

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <set>
#include <string>
#include <vector>

namespace orbslam3_multi
{
namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kEdgeSpatialLengthResidualScale = 4.0;
constexpr double kEdgePlanarLengthResidualScale = 35.0;
constexpr double kEdgePlanarDirectionResidualScale = 0.45;
constexpr double kEdgeVerticalResidualScale = 12.0;
constexpr double kEdgeGlobalVerticalResidualScale = 8.0;
constexpr double kEdgeRotationResidualScale = 0.08;
constexpr double kPreviousAnchorEndpointEdgeRigidity = 25.0;
constexpr double kTargetLockedEdgeRigidity = 100.0;
constexpr double kTargetLockedEdgeMinAlpha = 0.965;
constexpr double kTargetNeighborhoodEdgeRigidity = 20.0;
constexpr double kTargetNeighborhoodRigidityStartAlpha = 0.86;
constexpr double kTargetTailPoseLockTranslationScale = 180.0;
constexpr double kTargetTailPoseLockRotationScale = 35.0;
constexpr double kSparseSupportRigidityThreshold = 0.75;
constexpr double kReliableSupportRigidityThreshold = 1.15;
constexpr double kSparseSupportRigidity = 0.80;
constexpr double kReliableSupportRigidity = 1.50;
constexpr double kReliableCornerExtraRigidity = 1.0;
constexpr double kAngularRigidityCenterMultiplier = 1.0;
constexpr double kAngularRigidityEndpointExtraMultiplier = 0.0;
constexpr double kCorrectionGradientTranslationScale = 2.0;
constexpr double kCorrectionGradientRotationScale = 1.6;
constexpr double kCorrectionGradientAngularEndpointExtraMultiplier = 18.0;
constexpr double kCorrectionGradientAngularCenterMultiplier = 0.20;
constexpr double kCorrectionGradientSupportExponent = 0.5;
constexpr double kCornerHingeResidualScale = 3.5;
constexpr double kPropagatedKeyFrameShapeTranslationScale = 0.0;
constexpr double kPropagatedKeyFrameShapeRotationScale = 0.0;
constexpr double kCornerHingeStraightMultiplier = 0.25;
constexpr double kCornerHingeMinEndpointRigidity = 0.15;
constexpr double kCornerHingeEndpointRigidity = 6.0;
constexpr double kTurnTranslationRelaxStartRad = 0.45;
constexpr double kTurnTranslationRelaxFullRad = 1.20;
constexpr double kTurnTranslationMinMultiplier = 0.35;
constexpr double kSparseEdgeTranslationMinMultiplier = 0.08;
constexpr double kSparseEdgeTranslationMaxRelax = 0.55;
constexpr double kCurrentPoseSoftTranslationScale = 0.05;
constexpr double kCurrentPoseSoftRotationScale = 0.04;

using Vector6d = Eigen::Matrix<double, 6, 1>;

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

double TranslationError(const Eigen::Matrix4d& a, const Eigen::Matrix4d& b)
{
    return (a.block<3, 1>(0, 3) - b.block<3, 1>(0, 3)).norm();
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

double RotationError(const Eigen::Matrix4d& a, const Eigen::Matrix4d& b)
{
    const Eigen::Matrix3d delta =
        a.block<3, 3>(0, 0).transpose() * b.block<3, 3>(0, 0);
    Eigen::AngleAxisd angle_axis(delta);
    return std::abs(NormalizeAngle(angle_axis.angle()));
}

Eigen::Vector3d RotationVector(const Eigen::Matrix3d& rotation)
{
    Eigen::AngleAxisd angle_axis(rotation);
    double angle = NormalizeAngle(angle_axis.angle());
    Eigen::Vector3d axis = angle_axis.axis();
    if (!axis.allFinite())
    {
        return Eigen::Vector3d::Zero();
    }
    return axis * angle;
}

Eigen::Matrix3d RotationMatrixFromVector(const Eigen::Vector3d& rotation_vector)
{
    const double angle = rotation_vector.norm();
    if (angle <= 1e-12)
    {
        return Eigen::Matrix3d::Identity();
    }
    return Eigen::AngleAxisd(angle, rotation_vector / angle).toRotationMatrix();
}

Eigen::Matrix4d InterpolatePose(const Eigen::Matrix4d& from,
                                const Eigen::Matrix4d& to,
                                double alpha)
{
    const double bounded_alpha = std::max(0.0, std::min(1.0, alpha));
    Eigen::Matrix4d result = Eigen::Matrix4d::Identity();
    result.block<3, 1>(0, 3) =
        (1.0 - bounded_alpha) * from.block<3, 1>(0, 3) +
        bounded_alpha * to.block<3, 1>(0, 3);
    Eigen::Quaterniond from_q(from.block<3, 3>(0, 0));
    Eigen::Quaterniond to_q(to.block<3, 3>(0, 0));
    from_q.normalize();
    to_q.normalize();
    result.block<3, 3>(0, 0) =
        from_q.slerp(bounded_alpha, to_q).normalized().toRotationMatrix();
    return result;
}

Eigen::Matrix4d ApplyScaledEndpointDelta(
    const Eigen::Matrix4d& pose,
    const Eigen::Matrix4d& endpoint_before,
    const Eigen::Matrix4d& endpoint_after,
    double alpha)
{
    // 1K/propuesta de propagacion: se interpola el desplazamiento del endpoint,
    // no una matriz SE(3) premultiplicada. Premultiplicar una rotacion parcial
    // gira cada KF sobre el origen world y separa la nube cuando el yaw residual
    // es grande.
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
    // 1K/propuesta de propagacion: cada KF no-control hereda una correccion
    // interpolada entre los dos controles de su segmento. La pose raw/local no
    // se modifica; solo se propone una pose world final para GlobalPoseStore.
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

bool IsFiniteTransform(const Eigen::Matrix4d& transform)
{
    return transform.allFinite();
}

const PoseGraphPrior* FindTargetPrior(const PoseGraphProblem& graph)
{
    for (const auto& prior : graph.priors)
    {
        if (prior.prior_type == PoseGraphPriorType::FiducialTarget)
        {
            return &prior;
        }
    }
    return nullptr;
}

bool HasVertex(const PoseGraphProblem& graph, const RawKeyFrameId& keyframe_id)
{
    return std::any_of(
        graph.vertices.begin(),
        graph.vertices.end(),
        [&keyframe_id](const PoseGraphVertex& vertex)
        {
            return vertex.keyframe_id == keyframe_id;
        });
}

std::vector<RawKeyFrameId> CollectAffectedKeyFrames(
    const OptimizationDryRunResult& dry_run)
{
    std::vector<RawKeyFrameId> affected;
    affected.reserve(
        dry_run.proposed_vertex_poses.size() +
        dry_run.proposed_propagated_poses.size());
    for (const auto& proposal : dry_run.proposed_vertex_poses)
    {
        affected.push_back(proposal.keyframe_id);
    }
    for (const auto& proposal : dry_run.proposed_propagated_poses)
    {
        affected.push_back(proposal.keyframe_id);
    }
    return affected;
}

OptimizationApplyKeyFrameRecord MakeApplyRecord(
    const OptimizationPoseProposal& proposal,
    const GlobalPoseOptimizedResult& pose_result,
    bool propagated,
    const std::string& source)
{
    OptimizationApplyKeyFrameRecord record;
    record.keyframe_id = proposal.keyframe_id;
    record.propagated = propagated;
    record.fixed = proposal.fixed_vertex;
    record.applied = pose_result.success;
    record.delta_t_m = proposal.delta_t_m;
    record.delta_yaw_rad = proposal.delta_yaw_rad;
    record.source = source;
    record.reason = pose_result.reason;
    if (pose_result.success)
    {
        record.correction_t_m =
            pose_result.correction_T_latest.block<3, 1>(0, 3).norm();
        record.correction_yaw_rad =
            std::abs(NormalizeAngle(YawFromTransform(pose_result.correction_T_latest)));
    }
    return record;
}

struct EdgeResidualStats
{
    double mean_t = 0.0;
    double max_t = 0.0;
    std::vector<double> residuals_t;
    std::vector<double> residuals_yaw;
};

EdgeResidualStats ComputeEdgeResidualStats(
    const PoseGraphProblem& graph,
    const std::map<RawKeyFrameId, Eigen::Matrix4d>& poses)
{
    EdgeResidualStats stats;
    if (graph.edges.empty())
    {
        return stats;
    }

    double sum = 0.0;
    for (const auto& edge : graph.edges)
    {
        const auto from_it = poses.find(edge.from_keyframe_id);
        const auto to_it = poses.find(edge.to_keyframe_id);
        if (from_it == poses.end() || to_it == poses.end())
        {
            stats.residuals_t.push_back(0.0);
            continue;
        }
        const Eigen::Matrix4d current_relative =
            from_it->second.inverse() * to_it->second;
        // 1L: diagnostico post-apply. La direccion local puede cambiar si la
        // cadena se dobla por yaw; para detectar separaciones reales entre
        // vertices, el guard interno mide principalmente cambio de longitud.
        const double residual_t =
            std::abs(current_relative.block<3, 1>(0, 3).norm() -
                     edge.relative_T_from_to.block<3, 1>(0, 3).norm());
        const double residual_yaw =
            std::abs(NormalizeAngle(
                YawFromTransform(current_relative) -
                YawFromTransform(edge.relative_T_from_to)));
        stats.residuals_t.push_back(residual_t);
        stats.residuals_yaw.push_back(residual_yaw);
        stats.max_t = std::max(stats.max_t, residual_t);
        sum += residual_t;
    }
    if (!stats.residuals_t.empty())
    {
        stats.mean_t = sum / static_cast<double>(stats.residuals_t.size());
    }
    return stats;
}

const PoseGraphVertex* FindVertex(const PoseGraphProblem& graph,
                                  const RawKeyFrameId& keyframe_id)
{
    for (const auto& vertex : graph.vertices)
    {
        if (vertex.keyframe_id == keyframe_id)
        {
            return &vertex;
        }
    }
    return nullptr;
}

bool IsHardFixedVertex(const PoseGraphVertex& vertex)
{
    return vertex.is_hard_fiducial ||
           vertex.selection_reason == "previous_fiducial_anchor";
}

const PoseGraphDebugKeyFramePose* FindDebugKeyFrame(
    const PoseGraphProblem& graph,
    const RawKeyFrameId& keyframe_id)
{
    for (const auto& debug : graph.debug_keyframes)
    {
        if (debug.keyframe_id == keyframe_id)
        {
            return &debug;
        }
    }
    return nullptr;
}

double CorrectionAlphaForVertex(const PoseGraphProblem& graph,
                                const RawKeyFrameId& keyframe_id);

struct PoseState6D
{
    // 1J: la optimizacion mantiene la pose inicial como referencia fija y
    // resuelve una delta 6D en world. Los tres ultimos componentes son
    // LogSO3(delta_R), no roll/pitch/yaw absolutos.
    Eigen::Matrix4d initial_world_T_kf = Eigen::Matrix4d::Identity();
    Vector6d state = Vector6d::Zero();
};

PoseState6D MakePoseState6D(const Eigen::Matrix4d& pose)
{
    PoseState6D result;
    result.initial_world_T_kf = pose;
    result.state << pose(0, 3), pose(1, 3), pose(2, 3), 0.0, 0.0, 0.0;
    return result;
}

void SetStateFromPose(PoseState6D& state, const Eigen::Matrix4d& pose)
{
    state.state.segment<3>(0) = pose.block<3, 1>(0, 3);
    state.state.segment<3>(3) =
        RotationVector(pose.block<3, 3>(0, 0) *
                       state.initial_world_T_kf.block<3, 3>(0, 0).transpose());
}

Eigen::Matrix4d PoseFromState6D(const PoseState6D& state)
{
    Eigen::Matrix4d pose = state.initial_world_T_kf;
    pose(0, 3) = state.state.x();
    pose(1, 3) = state.state.y();
    pose(2, 3) = state.state.z();

    // 1J: la optimizacion angular usa SO(3) completo. Los tres ultimos
    // estados son el vector de rotacion Log(delta_R), no roll/pitch/yaw.
    const Eigen::Matrix3d delta_rotation =
        RotationMatrixFromVector(state.state.segment<3>(3));
    pose.block<3, 3>(0, 0) =
        delta_rotation * state.initial_world_T_kf.block<3, 3>(0, 0);
    return pose;
}

std::map<RawKeyFrameId, PoseState6D> BuildInitialPoseStates(
    const PoseGraphProblem& graph)
{
    std::map<RawKeyFrameId, PoseState6D> states;
    for (const auto& vertex : graph.vertices)
    {
        states[vertex.keyframe_id] =
            MakePoseState6D(vertex.initial_world_T_kf);
    }
    return states;
}

std::map<RawKeyFrameId, Eigen::Matrix4d> BuildFiducialNeighborhoodPoseLocks(
    const PoseGraphProblem& graph);

std::vector<RawKeyFrameId> CollectVariableVertexIds(
    const PoseGraphProblem& graph)
{
    std::vector<RawKeyFrameId> ids;
    const PoseGraphPrior* target_prior = FindTargetPrior(graph);
    const auto fiducial_neighborhood_pose_locks =
        BuildFiducialNeighborhoodPoseLocks(graph);
    for (const auto& vertex : graph.vertices)
    {
        if (!IsHardFixedVertex(vertex) &&
            fiducial_neighborhood_pose_locks.find(vertex.keyframe_id) ==
                fiducial_neighborhood_pose_locks.end() &&
            (!target_prior || !(vertex.keyframe_id == target_prior->keyframe_id)))
        {
            ids.push_back(vertex.keyframe_id);
        }
    }
    return ids;
}

void PinFiducialTargetState(
    const PoseGraphProblem& graph,
    std::map<RawKeyFrameId, PoseState6D>& states)
{
    const PoseGraphPrior* target_prior = FindTargetPrior(graph);
    if (!target_prior)
    {
        return;
    }

    auto target_it = states.find(target_prior->keyframe_id);
    if (target_it == states.end())
    {
        return;
    }

    target_it->second = MakePoseState6D(target_prior->target_world_T_kf);
}

void PinFiducialNeighborhoodPoseLocks(
    const PoseGraphProblem& graph,
    std::map<RawKeyFrameId, PoseState6D>& states)
{
    const auto fiducial_neighborhood_pose_locks =
        BuildFiducialNeighborhoodPoseLocks(graph);
    for (const auto& [keyframe_id, locked_pose] :
         fiducial_neighborhood_pose_locks)
    {
        auto state_it = states.find(keyframe_id);
        if (state_it == states.end())
        {
            continue;
        }
        SetStateFromPose(state_it->second, locked_pose);
    }
}

Eigen::VectorXd FlattenVariableStates(
    const std::vector<RawKeyFrameId>& variable_ids,
    const std::map<RawKeyFrameId, PoseState6D>& states)
{
    Eigen::VectorXd x =
        Eigen::VectorXd::Zero(static_cast<Eigen::Index>(6 * variable_ids.size()));
    for (size_t i = 0; i < variable_ids.size(); ++i)
    {
        const auto it = states.find(variable_ids[i]);
        if (it == states.end())
        {
            continue;
        }
        x.segment<6>(static_cast<Eigen::Index>(6 * i)) = it->second.state;
    }
    return x;
}

void SeedVariablesWithFiducialElasticGuess(
    const PoseGraphProblem& graph,
    const std::vector<RawKeyFrameId>& variable_ids,
    const std::map<RawKeyFrameId, PoseState6D>& base_states,
    Eigen::VectorXd& x)
{
    const PoseGraphPrior* target_prior = FindTargetPrior(graph);
    if (!target_prior)
    {
        return;
    }

    const auto target_state_it = base_states.find(target_prior->keyframe_id);
    if (target_state_it == base_states.end())
    {
        return;
    }

    const Eigen::Vector3d target_delta_t =
        target_prior->target_world_T_kf.block<3, 1>(0, 3) -
        target_state_it->second.initial_world_T_kf.block<3, 1>(0, 3);
    const Eigen::Vector3d target_rotation_delta =
        RotationVector(target_prior->target_world_T_kf.block<3, 3>(0, 0) *
                       target_state_it->second.initial_world_T_kf.block<3, 3>(0, 0).transpose());

    for (size_t i = 0; i < variable_ids.size(); ++i)
    {
        const double alpha = CorrectionAlphaForVertex(graph, variable_ids[i]);
        const double smooth_alpha = alpha * alpha * (3.0 - 2.0 * alpha);
        const Eigen::Index offset = static_cast<Eigen::Index>(6 * i);
        x.segment<3>(offset) += smooth_alpha * target_delta_t;
        x.segment<3>(offset + 3) += smooth_alpha * target_rotation_delta;
    }
}

void SeedVariablesWithTargetBackPropagation(
    const PoseGraphProblem& graph,
    const std::vector<RawKeyFrameId>& variable_ids,
    const std::map<RawKeyFrameId, PoseState6D>& base_states,
    Eigen::VectorXd& x)
{
    const PoseGraphPrior* target_prior = FindTargetPrior(graph);
    if (!target_prior)
    {
        return;
    }

    std::map<RawKeyFrameId, Eigen::Matrix4d> backpropagated_poses;
    backpropagated_poses[target_prior->keyframe_id] =
        target_prior->target_world_T_kf;

    // 1J: semilla opcional desde el target hacia atras. Propaga poses deseadas
    // siguiendo aristas temporales inversas para que el solver no arranque
    // siempre desde una deformacion nula cuando el fiducial esta muy lejos.
    bool changed = true;
    while (changed)
    {
        changed = false;
        for (auto edge_it = graph.edges.rbegin();
             edge_it != graph.edges.rend();
             ++edge_it)
        {
            if (backpropagated_poses.find(edge_it->from_keyframe_id) !=
                backpropagated_poses.end())
            {
                continue;
            }
            const auto to_pose_it =
                backpropagated_poses.find(edge_it->to_keyframe_id);
            if (to_pose_it == backpropagated_poses.end())
            {
                continue;
            }
            backpropagated_poses[edge_it->from_keyframe_id] =
                to_pose_it->second * edge_it->relative_T_from_to.inverse();
            changed = true;
        }
    }

    for (size_t i = 0; i < variable_ids.size(); ++i)
    {
        const auto pose_it = backpropagated_poses.find(variable_ids[i]);
        const auto state_it = base_states.find(variable_ids[i]);
        if (pose_it == backpropagated_poses.end() ||
            state_it == base_states.end())
        {
            continue;
        }

        const double alpha = CorrectionAlphaForVertex(graph, variable_ids[i]);
        const double smooth_alpha = alpha * alpha * (3.0 - 2.0 * alpha);
        if (smooth_alpha <= 0.0)
        {
            continue;
        }

        PoseState6D desired_state = state_it->second;
        SetStateFromPose(desired_state, pose_it->second);
        const Eigen::Index offset = static_cast<Eigen::Index>(6 * i);
        x.segment<6>(offset) =
            (1.0 - smooth_alpha) * x.segment<6>(offset) +
            smooth_alpha * desired_state.state;
    }
}

std::map<RawKeyFrameId, PoseState6D> StatesWithVariableVector(
    const std::map<RawKeyFrameId, PoseState6D>& base_states,
    const std::vector<RawKeyFrameId>& variable_ids,
    const Eigen::VectorXd& x)
{
    auto states = base_states;
    for (size_t i = 0; i < variable_ids.size(); ++i)
    {
        auto it = states.find(variable_ids[i]);
        if (it == states.end())
        {
            continue;
        }
        it->second.state = x.segment<6>(static_cast<Eigen::Index>(6 * i));
    }
    return states;
}

std::map<RawKeyFrameId, Eigen::Matrix4d> BuildPoseMapFromStates(
    const std::map<RawKeyFrameId, PoseState6D>& states)
{
    std::map<RawKeyFrameId, Eigen::Matrix4d> poses;
    for (const auto& entry : states)
    {
        poses[entry.first] = PoseFromState6D(entry.second);
    }
    return poses;
}

double SmoothCurrentPriorScale(const PoseGraphProblem& graph,
                               const PoseGraphPrior& prior)
{
    if (prior.prior_type != PoseGraphPriorType::CurrentPoseSoft)
    {
        return 1.0;
    }

    // 1J: no bloqueamos binariamente el entorno fiducial. Aumentamos de forma
    // suave el prior de pose actual cerca del fiducial previo y dejamos mas
    // libertad hacia el interior de la ventana. El target fiducial ya tiene su
    // propio prior absoluto fuerte.
    const double alpha = CorrectionAlphaForVertex(graph, prior.keyframe_id);
    const double near_previous_anchor = 1.0 - std::max(0.0, std::min(1.0, alpha));
    return 0.15 + 20.0 * near_previous_anchor * near_previous_anchor;
}

double PriorRotationResidualScale(const PoseGraphPrior& prior)
{
    if (prior.prior_type == PoseGraphPriorType::CurrentPoseSoft)
    {
        return kCurrentPoseSoftRotationScale;
    }
    // 1J: un fiducial observado define una pose absoluta 6D del KeyFrame.
    // No basta con anclar posicion+yaw; roll/pitch deben tirar con el peso
    // completo del prior fiducial. El GT de simulacion sigue siendo solo
    // diagnostico externo.
    return 1.0;
}

double EdgeCenterAlphaInWindow(const PoseGraphProblem& graph,
                               const PoseGraphEdge& edge)
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

bool IsTargetTailLockedEdge(const PoseGraphProblem& graph,
                            const PoseGraphEdge& edge)
{
    const PoseGraphVertex* from_vertex = FindVertex(graph, edge.from_keyframe_id);
    const PoseGraphVertex* to_vertex = FindVertex(graph, edge.to_keyframe_id);
    auto target_locked = [&](const PoseGraphVertex* vertex)
    {
        return vertex &&
               (vertex->keyframe_id == graph.target_keyframe_id ||
                vertex->selection_reason == "target_fiducial_neighborhood");
    };
    auto previous_locked = [](const PoseGraphVertex* vertex)
    {
        return vertex &&
               (vertex->selection_reason == "previous_fiducial_anchor" ||
                vertex->selection_reason == "previous_fiducial_neighborhood");
    };

    return (target_locked(from_vertex) && target_locked(to_vertex)) ||
           (previous_locked(from_vertex) && previous_locked(to_vertex));
}

std::map<RawKeyFrameId, Eigen::Matrix4d> BuildTargetTailPoseLocks(
    const PoseGraphProblem& graph)
{
    // 1J: la vecindad inmediata del fiducial objetivo se pincha por pose
    // relativa al propio target. Esto evita que la cola final se estire para
    // cumplir solo el prior absoluto del ultimo KF.
    std::map<RawKeyFrameId, Eigen::Matrix4d> locks;
    const PoseGraphPrior* target_prior = FindTargetPrior(graph);
    const PoseGraphVertex* target_vertex =
        FindVertex(graph, graph.target_keyframe_id);
    if (!target_prior || !target_vertex)
    {
        return locks;
    }

    const Eigen::Matrix4d target_initial_inverse =
        target_vertex->initial_world_T_kf.inverse();
    for (const auto& vertex : graph.vertices)
    {
        if (vertex.keyframe_id == graph.target_keyframe_id ||
            vertex.selection_reason != "target_fiducial_neighborhood")
        {
            continue;
        }
        const Eigen::Matrix4d target_T_neighbor =
            target_initial_inverse * vertex.initial_world_T_kf;
        locks[vertex.keyframe_id] =
            target_prior->target_world_T_kf * target_T_neighbor;
    }
    return locks;
}

std::map<RawKeyFrameId, Eigen::Matrix4d> BuildPreviousAnchorTailPoseLocks(
    const PoseGraphProblem& graph)
{
    // 1J: la vecindad del fiducial previo se reconstruye desde el anchor ya
    // aceptado. Este lock protege el borde fijo de la ventana sin consultar GT.
    std::map<RawKeyFrameId, Eigen::Matrix4d> locks;
    const PoseGraphVertex* anchor_vertex = nullptr;
    for (const auto& vertex : graph.vertices)
    {
        if (vertex.selection_reason == "previous_fiducial_anchor")
        {
            anchor_vertex = &vertex;
            break;
        }
    }
    if (!anchor_vertex)
    {
        return locks;
    }

    const Eigen::Matrix4d anchor_initial_inverse =
        anchor_vertex->initial_world_T_kf.inverse();
    for (const auto& vertex : graph.vertices)
    {
        if (vertex.keyframe_id == anchor_vertex->keyframe_id ||
            vertex.selection_reason != "previous_fiducial_neighborhood")
        {
            continue;
        }
        const Eigen::Matrix4d anchor_T_neighbor =
            anchor_initial_inverse * vertex.initial_world_T_kf;
        locks[vertex.keyframe_id] =
            anchor_vertex->initial_world_T_kf * anchor_T_neighbor;
    }
    return locks;
}

std::map<RawKeyFrameId, Eigen::Matrix4d> BuildFiducialNeighborhoodPoseLocks(
    const PoseGraphProblem& graph)
{
    std::map<RawKeyFrameId, Eigen::Matrix4d> locks =
        BuildPreviousAnchorTailPoseLocks(graph);
    const auto target_locks = BuildTargetTailPoseLocks(graph);
    for (const auto& [keyframe_id, locked_pose] : target_locks)
    {
        locks[keyframe_id] = locked_pose;
    }
    return locks;
}

double TargetNeighborhoodEdgeRigidity(const PoseGraphProblem& graph,
                                      const PoseGraphEdge& edge)
{
    const PoseGraphVertex* from_vertex = FindVertex(graph, edge.from_keyframe_id);
    const PoseGraphVertex* to_vertex = FindVertex(graph, edge.to_keyframe_id);
    const bool touches_previous_anchor =
        (from_vertex &&
         from_vertex->selection_reason == "previous_fiducial_anchor") ||
        (to_vertex &&
         to_vertex->selection_reason == "previous_fiducial_anchor");
    if (IsTargetTailLockedEdge(graph, edge))
    {
        return kTargetLockedEdgeRigidity;
    }
    if (touches_previous_anchor)
    {
        return kPreviousAnchorEndpointEdgeRigidity;
    }

    const double from_alpha =
        CorrectionAlphaForVertex(graph, edge.from_keyframe_id);
    const double to_alpha =
        CorrectionAlphaForVertex(graph, edge.to_keyframe_id);
    const double target_min_alpha = std::min(from_alpha, to_alpha);
    if (target_min_alpha >= kTargetLockedEdgeMinAlpha)
    {
        return kTargetLockedEdgeRigidity;
    }

    const double target_proximity = std::max(from_alpha, to_alpha);
    if (target_proximity <= kTargetNeighborhoodRigidityStartAlpha)
    {
        return 1.0;
    }

    const double alpha =
        (target_proximity - kTargetNeighborhoodRigidityStartAlpha) /
        (1.0 - kTargetNeighborhoodRigidityStartAlpha);
    const double bounded_alpha =
        std::max(0.0, std::min(1.0, alpha));
    const double smooth_alpha =
        bounded_alpha * bounded_alpha * (3.0 - 2.0 * bounded_alpha);
    return 1.0 + smooth_alpha * (kTargetNeighborhoodEdgeRigidity - 1.0);
}

double SupportReliabilityRigidity(double support_multiplier);
double EdgeSupportRigidity(const PoseGraphEdge& edge);

double ParabolicAngularRigidity(const PoseGraphProblem& graph,
                                const PoseGraphEdge& edge)
{
    const double target_rigidity = TargetNeighborhoodEdgeRigidity(graph, edge);
    if (target_rigidity > 1.0)
    {
        return target_rigidity;
    }

    const double alpha = EdgeCenterAlphaInWindow(graph, edge);
    const double flexibility = 4.0 * alpha * (1.0 - alpha);
    const double bounded_flexibility =
        std::max(0.0, std::min(1.0, flexibility));
    return EdgeSupportRigidity(edge) *
           (kAngularRigidityCenterMultiplier +
            kAngularRigidityEndpointExtraMultiplier *
                (1.0 - bounded_flexibility));
}

double CorrectionGradientSupportRigidity(const PoseGraphEdge& edge)
{
    return std::pow(
        std::max(0.1, edge.support_rigidity_multiplier),
        kCorrectionGradientSupportExponent);
}

double EdgeSupportRigidity(const PoseGraphEdge& edge)
{
    return SupportReliabilityRigidity(edge.support_rigidity_multiplier);
}

double SupportReliabilityRigidity(double support_multiplier)
{
    // 1J: el soporte derivado del muestreo de KFs modula rigidez, pero queda
    // acotado. No sustituye a una futura base de covisibilidad/loops.
    if (support_multiplier <= kSparseSupportRigidityThreshold)
    {
        return kSparseSupportRigidity;
    }
    if (support_multiplier >= kReliableSupportRigidityThreshold)
    {
        const double oversupport =
            std::min(1.0, support_multiplier - kReliableSupportRigidityThreshold);
        return kReliableSupportRigidity * (1.0 + 0.5 * oversupport);
    }

    const double alpha =
        (support_multiplier - kSparseSupportRigidityThreshold) /
        (kReliableSupportRigidityThreshold - kSparseSupportRigidityThreshold);
    const double smooth_alpha = alpha * alpha * (3.0 - 2.0 * alpha);
    return kSparseSupportRigidity +
           smooth_alpha * (1.0 - kSparseSupportRigidity);
}

double ParabolicCorrectionAngularGradientRigidity(const PoseGraphProblem& graph,
                                                  const PoseGraphEdge& edge)
{
    const double alpha = EdgeCenterAlphaInWindow(graph, edge);
    const double flexibility = 4.0 * alpha * (1.0 - alpha);
    const double bounded_flexibility =
        std::max(0.0, std::min(1.0, flexibility));
    const double support = CorrectionGradientSupportRigidity(edge);
    const Eigen::Vector3d raw_rotation =
        RotationVector(edge.relative_T_from_to.block<3, 3>(0, 0));
    const double raw_turn = raw_rotation.norm();
    double turn_rigidity = 1.0;
    if (raw_turn <= 0.25)
    {
        turn_rigidity = 3.0;
    }
    else if (raw_turn < 0.90)
    {
        const double turn_alpha = (raw_turn - 0.25) / (0.90 - 0.25);
        turn_rigidity = 3.0 - 2.0 * turn_alpha;
    }
    else
    {
        turn_rigidity = 0.75;
    }

    const PoseGraphVertex* from_vertex = FindVertex(graph, edge.from_keyframe_id);
    const PoseGraphVertex* to_vertex = FindVertex(graph, edge.to_keyframe_id);
    const bool touches_corner_vertex =
        (from_vertex && from_vertex->selection_reason == "corner_yaw_vertex") ||
        (to_vertex && to_vertex->selection_reason == "corner_yaw_vertex");
    const double corner_flex_multiplier = touches_corner_vertex ? 0.45 : 1.0;
    return support *
           turn_rigidity *
           corner_flex_multiplier *
           (kCorrectionGradientAngularCenterMultiplier +
            kCorrectionGradientAngularEndpointExtraMultiplier *
                (1.0 - bounded_flexibility));
}

double EdgeTurnTranslationRigidity(const PoseGraphEdge& edge)
{
    const double yaw = std::abs(YawFromTransform(edge.relative_T_from_to));
    const Eigen::Vector3d rotation_delta =
        RotationVector(edge.relative_T_from_to.block<3, 3>(0, 0));
    const double roll_pitch =
        std::sqrt(rotation_delta.x() * rotation_delta.x() +
                  rotation_delta.y() * rotation_delta.y());
    const double turn = std::max(yaw, roll_pitch);
    if (turn <= kTurnTranslationRelaxStartRad)
    {
        return 1.0;
    }
    if (turn >= kTurnTranslationRelaxFullRad)
    {
        return kTurnTranslationMinMultiplier;
    }
    const double alpha =
        (turn - kTurnTranslationRelaxStartRad) /
        (kTurnTranslationRelaxFullRad - kTurnTranslationRelaxStartRad);
    return 1.0 - alpha * (1.0 - kTurnTranslationMinMultiplier);
}

double SparseEdgeTranslationRigidity(const PoseGraphProblem& graph,
                                     const PoseGraphEdge& edge)
{
    const PoseGraphVertex* from_vertex = FindVertex(graph, edge.from_keyframe_id);
    const PoseGraphVertex* to_vertex = FindVertex(graph, edge.to_keyframe_id);
    if (!from_vertex || !to_vertex)
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
            from_vertex->initial_world_T_kf,
            to_vertex->initial_world_T_kf);
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
    const double sparse_relax =
        kSparseEdgeTranslationMaxRelax * center_flexibility * low_support *
        projection_conflict;
    return std::max(
        kSparseEdgeTranslationMinMultiplier,
        1.0 - sparse_relax);
}

double EdgeTranslationRigidity(const PoseGraphProblem& graph,
                               const PoseGraphEdge& edge)
{
    const double target_rigidity = TargetNeighborhoodEdgeRigidity(graph, edge);
    if (target_rigidity > 1.0)
    {
        return target_rigidity;
    }

    return EdgeTurnTranslationRigidity(edge) *
           SparseEdgeTranslationRigidity(graph, edge) *
           EdgeSupportRigidity(edge);
}

double CornerHingeRigidity(const PoseGraphProblem& graph,
                           const PoseGraphEdge& previous_edge,
                           const PoseGraphEdge& next_edge,
                           double expected_turn)
{
    const double alpha =
        CorrectionAlphaForVertex(graph, previous_edge.to_keyframe_id);
    const double endpoint_proximity =
        std::max(alpha * alpha, (1.0 - alpha) * (1.0 - alpha));
    const double endpoint_rigidity =
        kCornerHingeMinEndpointRigidity +
        kCornerHingeEndpointRigidity * endpoint_proximity * endpoint_proximity;
    const double mean_support =
        0.5 * (previous_edge.support_rigidity_multiplier +
               next_edge.support_rigidity_multiplier);
    const double support = SupportReliabilityRigidity(mean_support);
    const double turn_multiplier =
        std::abs(expected_turn) >= 0.30 ? 1.0 : kCornerHingeStraightMultiplier;
    const double reliable_corner =
        (std::abs(expected_turn) >= 0.60 &&
         mean_support >= kReliableSupportRigidityThreshold)
            ? kReliableCornerExtraRigidity
            : 1.0;
    return endpoint_rigidity * support * turn_multiplier * reliable_corner;
}

Eigen::VectorXd BuildPoseGraphResidualVector(
    const PoseGraphProblem& graph,
    const std::map<RawKeyFrameId, PoseState6D>& states)
{
    // 1J: este vector es la funcion objetivo del dry-run. ComputeCost()
    // debe medir los mismos bloques conceptuales para que decision y solver no
    // optimicen criterios distintos.
    const auto poses = BuildPoseMapFromStates(states);
    std::vector<double> residuals;
    residuals.reserve(graph.priors.size() * 4U + graph.edges.size() * 4U);

    for (const auto& prior : graph.priors)
    {
        const auto pose_it = poses.find(prior.keyframe_id);
        if (pose_it == poses.end())
        {
            continue;
        }

        const double soft_scale = SmoothCurrentPriorScale(graph, prior);
        const double wt =
            std::sqrt(std::max(
                0.0,
                prior.weight_translation * soft_scale *
                    (prior.prior_type == PoseGraphPriorType::CurrentPoseSoft
                         ? kCurrentPoseSoftTranslationScale
                         : 1.0)));
        const double wr =
            std::sqrt(std::max(
                0.0,
                prior.weight_rotation * soft_scale *
                    PriorRotationResidualScale(prior)));
        const Eigen::Vector3d delta_t =
            pose_it->second.block<3, 1>(0, 3) -
            prior.target_world_T_kf.block<3, 1>(0, 3);
        const Eigen::Vector3d rotation_delta =
            RotationVector(prior.target_world_T_kf.block<3, 3>(0, 0).transpose() *
                           pose_it->second.block<3, 3>(0, 0));
        residuals.push_back(wt * delta_t.x());
        residuals.push_back(wt * delta_t.y());
        residuals.push_back(wt * delta_t.z());
        residuals.push_back(wr * rotation_delta.x());
        residuals.push_back(wr * rotation_delta.y());
        residuals.push_back(wr * rotation_delta.z());
    }

    const auto target_tail_pose_locks = BuildTargetTailPoseLocks(graph);
    for (const auto& [keyframe_id, locked_pose] : target_tail_pose_locks)
    {
        const auto pose_it = poses.find(keyframe_id);
        if (pose_it == poses.end())
        {
            continue;
        }
        const Eigen::Vector3d delta_t =
            pose_it->second.block<3, 1>(0, 3) -
            locked_pose.block<3, 1>(0, 3);
        const Eigen::Vector3d rotation_delta =
            RotationVector(locked_pose.block<3, 3>(0, 0).transpose() *
                           pose_it->second.block<3, 3>(0, 0));
        residuals.push_back(kTargetTailPoseLockTranslationScale * delta_t.x());
        residuals.push_back(kTargetTailPoseLockTranslationScale * delta_t.y());
        residuals.push_back(kTargetTailPoseLockTranslationScale * delta_t.z());
        residuals.push_back(kTargetTailPoseLockRotationScale * rotation_delta.x());
        residuals.push_back(kTargetTailPoseLockRotationScale * rotation_delta.y());
        residuals.push_back(kTargetTailPoseLockRotationScale * rotation_delta.z());
    }
    const auto previous_anchor_tail_pose_locks =
        BuildPreviousAnchorTailPoseLocks(graph);
    for (const auto& [keyframe_id, locked_pose] : previous_anchor_tail_pose_locks)
    {
        const auto pose_it = poses.find(keyframe_id);
        if (pose_it == poses.end())
        {
            continue;
        }
        const Eigen::Vector3d delta_t =
            pose_it->second.block<3, 1>(0, 3) -
            locked_pose.block<3, 1>(0, 3);
        const Eigen::Vector3d rotation_delta =
            RotationVector(locked_pose.block<3, 3>(0, 0).transpose() *
                           pose_it->second.block<3, 3>(0, 0));
        residuals.push_back(kTargetTailPoseLockTranslationScale * delta_t.x());
        residuals.push_back(kTargetTailPoseLockTranslationScale * delta_t.y());
        residuals.push_back(kTargetTailPoseLockTranslationScale * delta_t.z());
        residuals.push_back(kTargetTailPoseLockRotationScale * rotation_delta.x());
        residuals.push_back(kTargetTailPoseLockRotationScale * rotation_delta.y());
        residuals.push_back(kTargetTailPoseLockRotationScale * rotation_delta.z());
    }

    for (const auto& edge : graph.edges)
    {
        const auto from_it = poses.find(edge.from_keyframe_id);
        const auto to_it = poses.find(edge.to_keyframe_id);
        const auto from_state_it = states.find(edge.from_keyframe_id);
        const auto to_state_it = states.find(edge.to_keyframe_id);
        if (from_it == poses.end() || to_it == poses.end() ||
            from_state_it == states.end() || to_state_it == states.end())
        {
            continue;
        }

        const Eigen::Matrix4d current_relative =
            from_it->second.inverse() * to_it->second;
        const double w = std::sqrt(std::max(0.0, edge.weight));
        const Eigen::Vector3d current_t =
            current_relative.block<3, 1>(0, 3);
        const Eigen::Vector3d expected_t =
            edge.relative_T_from_to.block<3, 1>(0, 3);
        const double angular_rigidity =
            ParabolicAngularRigidity(graph, edge);
        const double translation_rigidity =
            EdgeTranslationRigidity(graph, edge);
        const double spatial_length_residual =
            current_t.norm() - expected_t.norm();
        const double planar_length_residual =
            current_t.head<2>().norm() - expected_t.head<2>().norm();
        const Eigen::Vector2d planar_direction_residual =
            current_t.head<2>() - expected_t.head<2>();
        const double vertical_residual = current_t.z() - expected_t.z();
        const double global_vertical_residual =
            GlobalTranslationDelta(from_it->second, to_it->second).z() -
            GlobalTranslationDelta(
                from_state_it->second.initial_world_T_kf,
                to_state_it->second.initial_world_T_kf).z();
        residuals.push_back(
            translation_rigidity * kEdgePlanarLengthResidualScale * w *
            planar_length_residual);
        residuals.push_back(
            translation_rigidity * kEdgePlanarDirectionResidualScale * w *
            planar_direction_residual.x());
        residuals.push_back(
            translation_rigidity * kEdgePlanarDirectionResidualScale * w *
            planar_direction_residual.y());
        residuals.push_back(
            translation_rigidity * kEdgeVerticalResidualScale * w *
            vertical_residual);
        residuals.push_back(
            translation_rigidity * kEdgeGlobalVerticalResidualScale * w *
            global_vertical_residual);
        residuals.push_back(
            translation_rigidity * kEdgeSpatialLengthResidualScale * w *
            spatial_length_residual);
        const Eigen::Vector3d rotation_delta =
            RotationVector(edge.relative_T_from_to.block<3, 3>(0, 0).transpose() *
                           current_relative.block<3, 3>(0, 0));
        residuals.push_back(
            angular_rigidity * kEdgeRotationResidualScale * w * rotation_delta.x());
        residuals.push_back(
            angular_rigidity * kEdgeRotationResidualScale * w * rotation_delta.y());
        residuals.push_back(
            angular_rigidity * kEdgeRotationResidualScale * w * rotation_delta.z());
    }

    // 1J: bloque experimental desactivado por defecto. Los KeyFrames
    // intermedios podrian aportar rigidez de forma, pero las pruebas offline
    // mostraron que un residuo directo contra la pose mapa cruda empeora el
    // coste y no corrige la zona central. Mantener las escalas en 0.0 evita que
    // afecte a apply mientras se diseña una restriccion mas selectiva.
    for (const auto& entry : graph.propagation_plan)
    {
        if (entry.mode != PoseGraphPropagationMode::PathSegment ||
            !entry.control_vertex_b)
        {
            continue;
        }
        const auto a_it = poses.find(entry.control_vertex_a);
        const auto b_it = poses.find(entry.control_vertex_b.value());
        const auto* debug = FindDebugKeyFrame(graph, entry.affected_keyframe_id);
        if (a_it == poses.end() || b_it == poses.end() || !debug)
        {
            continue;
        }
        const Eigen::Matrix4d current =
            InterpolatePose(a_it->second, b_it->second, entry.segment_alpha);
        const Eigen::Vector3d translation_residual =
            current.block<3, 1>(0, 3) -
            debug->map_world_T_kf.block<3, 1>(0, 3);
        const Eigen::Vector3d rotation_residual =
            RotationVector(
                debug->map_world_T_kf.block<3, 3>(0, 0).transpose() *
                current.block<3, 3>(0, 0));
        residuals.push_back(
            kPropagatedKeyFrameShapeTranslationScale *
            translation_residual.x());
        residuals.push_back(
            kPropagatedKeyFrameShapeTranslationScale *
            translation_residual.y());
        residuals.push_back(
            kPropagatedKeyFrameShapeTranslationScale *
            translation_residual.z());
        residuals.push_back(
            kPropagatedKeyFrameShapeRotationScale * rotation_residual.x());
        residuals.push_back(
            kPropagatedKeyFrameShapeRotationScale * rotation_residual.y());
        residuals.push_back(
            kPropagatedKeyFrameShapeRotationScale * rotation_residual.z());
    }

    for (const auto& edge : graph.edges)
    {
        const auto from_state_it = states.find(edge.from_keyframe_id);
        const auto to_state_it = states.find(edge.to_keyframe_id);
        if (from_state_it == states.end() || to_state_it == states.end())
        {
            continue;
        }

        // 1J: regularizamos el campo de correccion, no la pose absoluta. Cerca
        // de ambos fiduciales la diferencia de correccion entre vertices debe
        // ser pequeña; en el centro de la ventana se permite que aparezca la
        // mayor curvatura. El soporte por KFs aumenta la rigidez, pero no anula
        // la parabola espacial.
        const double w = std::sqrt(std::max(0.0, edge.weight));
        const double translation_gradient_rigidity =
            CorrectionGradientSupportRigidity(edge);
        const double angular_gradient_rigidity =
            ParabolicCorrectionAngularGradientRigidity(graph, edge);
        Vector6d gradient = Vector6d::Zero();
        gradient.segment<3>(0) =
            (to_state_it->second.state.segment<3>(0) -
             to_state_it->second.initial_world_T_kf.block<3, 1>(0, 3)) -
            (from_state_it->second.state.segment<3>(0) -
             from_state_it->second.initial_world_T_kf.block<3, 1>(0, 3));
        gradient.segment<3>(3) =
            to_state_it->second.state.segment<3>(3) -
            from_state_it->second.state.segment<3>(3);
        residuals.push_back(
            translation_gradient_rigidity * kCorrectionGradientTranslationScale * w *
            gradient.x());
        residuals.push_back(
            translation_gradient_rigidity * kCorrectionGradientTranslationScale * w *
            gradient.y());
        residuals.push_back(
            translation_gradient_rigidity * kCorrectionGradientTranslationScale * w *
            gradient.z());
        residuals.push_back(
            angular_gradient_rigidity * kCorrectionGradientRotationScale * w *
            gradient(3));
        residuals.push_back(
            angular_gradient_rigidity * kCorrectionGradientRotationScale * w *
            gradient(4));
        residuals.push_back(
            angular_gradient_rigidity * kCorrectionGradientRotationScale * w *
            gradient(5));
    }

    for (size_t i = 1; i < graph.edges.size(); ++i)
    {
        const auto& previous_edge = graph.edges[i - 1];
        const auto& next_edge = graph.edges[i];
        if (!(previous_edge.to_keyframe_id == next_edge.from_keyframe_id))
        {
            continue;
        }
        const auto a_it = poses.find(previous_edge.from_keyframe_id);
        const auto b_it = poses.find(previous_edge.to_keyframe_id);
        const auto c_it = poses.find(next_edge.to_keyframe_id);
        const auto a_state_it = states.find(previous_edge.from_keyframe_id);
        const auto b_state_it = states.find(previous_edge.to_keyframe_id);
        const auto c_state_it = states.find(next_edge.to_keyframe_id);
        if (a_it == poses.end() || b_it == poses.end() || c_it == poses.end() ||
            a_state_it == states.end() || b_state_it == states.end() ||
            c_state_it == states.end())
        {
            continue;
        }

        const double expected_turn =
            SignedPlanarTurn(
                a_state_it->second.initial_world_T_kf,
                b_state_it->second.initial_world_T_kf,
                c_state_it->second.initial_world_T_kf);
        const double current_turn =
            SignedPlanarTurn(a_it->second, b_it->second, c_it->second);
        const double w =
            std::sqrt(
                std::max(
                    0.0,
                    0.5 * (previous_edge.weight + next_edge.weight)));
        const double hinge_rigidity =
            CornerHingeRigidity(graph, previous_edge, next_edge, expected_turn);
        residuals.push_back(
            kCornerHingeResidualScale * hinge_rigidity * w *
            NormalizeAngle(current_turn - expected_turn));
    }

    Eigen::VectorXd r(static_cast<Eigen::Index>(residuals.size()));
    for (size_t i = 0; i < residuals.size(); ++i)
    {
        r(static_cast<Eigen::Index>(i)) = residuals[i];
    }
    return r;
}

struct PoseGraphSolveResult
{
    bool success = false;
    uint64_t iterations = 0;
    std::map<RawKeyFrameId, Eigen::Matrix4d> poses;
};

PoseGraphSolveResult SolveWeightedPlanarPoseGraph(
    const PoseGraphProblem& graph)
{
    // 1J: solver del dry-run. Trabaja sobre copias locales de las poses del
    // grafo y devuelve una propuesta; no escribe en GlobalPoseStore ni toca
    // RawMapDatabase.
    PoseGraphSolveResult result;
    auto base_states = BuildInitialPoseStates(graph);
    const auto seed_states = base_states;
    const auto variable_ids = CollectVariableVertexIds(graph);
    if (variable_ids.empty())
    {
        PinFiducialTargetState(graph, base_states);
        PinFiducialNeighborhoodPoseLocks(graph, base_states);
        result.poses = BuildPoseMapFromStates(base_states);
        result.success = true;
        return result;
    }

    Eigen::VectorXd x = FlattenVariableStates(variable_ids, base_states);
    SeedVariablesWithFiducialElasticGuess(graph, variable_ids, seed_states, x);
    SeedVariablesWithTargetBackPropagation(graph, variable_ids, seed_states, x);
    PinFiducialTargetState(graph, base_states);
    PinFiducialNeighborhoodPoseLocks(graph, base_states);
    constexpr uint64_t kMaxIterations = 35;
    constexpr double kFiniteDiffTranslation = 1e-4;
    constexpr double kFiniteDiffRotation = 1e-5;
    constexpr double kMinAcceptedImprovement = 1e-7;
    double damping = 1e-3;

    double previous_cost = std::numeric_limits<double>::infinity();
    for (uint64_t iteration = 0; iteration < kMaxIterations; ++iteration)
    {
        const auto states = StatesWithVariableVector(base_states, variable_ids, x);
        const Eigen::VectorXd residual = BuildPoseGraphResidualVector(graph, states);
        if (residual.size() == 0)
        {
            break;
        }
        const double cost = 0.5 * residual.squaredNorm();
        if (std::isfinite(previous_cost) &&
            std::abs(previous_cost - cost) < kMinAcceptedImprovement)
        {
            break;
        }
        previous_cost = cost;

        Eigen::MatrixXd jacobian(
            residual.size(),
            static_cast<Eigen::Index>(x.size()));
        for (Eigen::Index col = 0; col < x.size(); ++col)
        {
            Eigen::VectorXd x_perturbed = x;
            const double eps = (col % 6 >= 3)
                ? kFiniteDiffRotation
                : kFiniteDiffTranslation;
            x_perturbed(col) += eps;
            const auto perturbed_states =
                StatesWithVariableVector(base_states, variable_ids, x_perturbed);
            const Eigen::VectorXd perturbed_residual =
                BuildPoseGraphResidualVector(graph, perturbed_states);
            if (perturbed_residual.size() != residual.size())
            {
                jacobian.col(col).setZero();
                continue;
            }
            jacobian.col(col) = (perturbed_residual - residual) / eps;
        }

        const Eigen::MatrixXd hessian =
            jacobian.transpose() * jacobian +
            damping * Eigen::MatrixXd::Identity(x.size(), x.size());
        const Eigen::VectorXd gradient = jacobian.transpose() * residual;
        Eigen::VectorXd step = -hessian.ldlt().solve(gradient);
        if (!step.allFinite())
        {
            break;
        }

        // 1J: los errores fiduciales pueden ser grandes, pero limitamos cada
        // paso para evitar saltos numericos que separen la nube antes de que
        // las edges repartan la deformacion.
        for (Eigen::Index i = 0; i < step.size(); i += 6)
        {
            const double translation_norm = step.segment<3>(i).norm();
            constexpr double kMaxTranslationStep = 2.0;
            if (translation_norm > kMaxTranslationStep)
            {
                step.segment<3>(i) *= kMaxTranslationStep / translation_norm;
            }
            const double rotation_norm = step.segment<3>(i + 3).norm();
            constexpr double kMaxRotationStep = 0.35;
            if (rotation_norm > kMaxRotationStep)
            {
                step.segment<3>(i + 3) *= kMaxRotationStep / rotation_norm;
            }
        }

        bool accepted = false;
        double alpha = 1.0;
        for (int backtrack = 0; backtrack < 6; ++backtrack)
        {
            Eigen::VectorXd candidate = x + alpha * step;
            const auto candidate_states =
                StatesWithVariableVector(base_states, variable_ids, candidate);
            const Eigen::VectorXd candidate_residual =
                BuildPoseGraphResidualVector(graph, candidate_states);
            const double candidate_cost =
                0.5 * candidate_residual.squaredNorm();
            if (candidate_cost < cost && std::isfinite(candidate_cost))
            {
                x = candidate;
                accepted = true;
                ++result.iterations;
                damping = std::max(1e-6, damping * 0.5);
                break;
            }
            alpha *= 0.5;
            damping = std::min(1e3, damping * 2.0);
        }

        if (!accepted)
        {
            break;
        }
    }

    const auto final_states = StatesWithVariableVector(base_states, variable_ids, x);
    result.poses = BuildPoseMapFromStates(final_states);
    result.success = true;
    return result;
}

bool IsProtectedEndpoint(const PoseGraphProblem& graph,
                         const RawKeyFrameId& keyframe_id)
{
    const PoseGraphVertex* vertex = FindVertex(graph, keyframe_id);
    if (!vertex)
    {
        return false;
    }
    return IsHardFixedVertex(*vertex);
}

double CorrectionAlphaForVertex(const PoseGraphProblem& graph,
                                const RawKeyFrameId& keyframe_id)
{
    if (keyframe_id == graph.target_keyframe_id)
    {
        return 1.0;
    }

    const uint64_t target_id = graph.target_keyframe_id.local_kf_id;
    const PoseGraphVertex* best_anchor = nullptr;
    for (const auto& vertex : graph.vertices)
    {
        if (vertex.selection_reason != "previous_fiducial_anchor")
        {
            continue;
        }
        const uint64_t anchor_id = vertex.keyframe_id.local_kf_id;
        const bool same_branch =
            (anchor_id <= target_id && keyframe_id.local_kf_id <= target_id &&
             keyframe_id.local_kf_id >= anchor_id) ||
            (anchor_id >= target_id && keyframe_id.local_kf_id >= target_id &&
             keyframe_id.local_kf_id <= anchor_id);
        if (!same_branch)
        {
            continue;
        }
        if (!best_anchor ||
            std::abs(static_cast<double>(anchor_id) -
                     static_cast<double>(keyframe_id.local_kf_id)) <
                std::abs(static_cast<double>(best_anchor->keyframe_id.local_kf_id) -
                         static_cast<double>(keyframe_id.local_kf_id)))
        {
            best_anchor = &vertex;
        }
    }
    if (!best_anchor)
    {
        return 1.0;
    }

    const double anchor_id =
        static_cast<double>(best_anchor->keyframe_id.local_kf_id);
    const double denominator = static_cast<double>(target_id) - anchor_id;
    if (std::abs(denominator) <= 1e-9)
    {
        return 1.0;
    }
    const double alpha =
        (static_cast<double>(keyframe_id.local_kf_id) - anchor_id) / denominator;
    return std::max(0.0, std::min(1.0, alpha));
}

bool IsDeformableInternalEdgeForAbsurdPartial(
    const PoseGraphProblem& graph,
    const PoseGraphEdge& edge,
    double before_residual_t,
    double after_residual_t,
    double after_residual_yaw,
    const OptimizationDryRunResult& dry_run,
    const PostApplyValidationResult& result,
    const OptimizationManagerConfig& config)
{
    // 1L: una arista temporal de ORB-SLAM3 no siempre es una restriccion
    // fisica fuerte. En paredes pobres en textura puede codificar una esquina
    // falsa; si un fiducial absurdo baja mucho, esa arista debe poder ceder.
    const bool absurd_fiducial_progress =
        result.error.before_error_t >= config.post_apply_fiducial_absurd_error_t &&
        result.primary_error_improved &&
        (result.final_error_ok ||
         result.error.improvement_ratio >= config.dryrun_partial_min_improvement_ratio);
    const bool rebuilt_partial_progress =
        graph.rebuilt_from_partial_checkpoint &&
        result.primary_error_improved &&
        result.final_error_ok;
    if (!absurd_fiducial_progress && !rebuilt_partial_progress)
    {
        return false;
    }

    if (IsProtectedEndpoint(graph, edge.from_keyframe_id) &&
        IsProtectedEndpoint(graph, edge.to_keyframe_id))
    {
        return false;
    }

    const uint64_t kf_gap =
        edge.to_keyframe_id.local_kf_id >= edge.from_keyframe_id.local_kf_id
            ? edge.to_keyframe_id.local_kf_id - edge.from_keyframe_id.local_kf_id
            : edge.from_keyframe_id.local_kf_id - edge.to_keyframe_id.local_kf_id;
    const bool sparse_or_long_temporal_edge = kf_gap > 1U || edge.weight <= 10.0;
    const bool big_yaw_artifact = after_residual_yaw >= config.dryrun_max_allowed_delta_yaw;
    const bool close_to_main_correction =
        edge.from_keyframe_id == graph.target_keyframe_id ||
        edge.to_keyframe_id == graph.target_keyframe_id ||
        after_residual_t > before_residual_t + config.post_apply_internal_max_growth_t;
    const bool subdivision_context =
        graph.anchor_preservation.subdivision_candidates > 0;

    return sparse_or_long_temporal_edge || big_yaw_artifact ||
           close_to_main_correction || subdivision_context;
}

std::map<RawKeyFrameId, Eigen::Matrix4d> BuildGraphInitialPoseMap(
    const PoseGraphProblem& graph)
{
    std::map<RawKeyFrameId, Eigen::Matrix4d> poses;
    for (const auto& vertex : graph.vertices)
    {
        poses[vertex.keyframe_id] = vertex.initial_world_T_kf;
    }
    return poses;
}

std::map<RawKeyFrameId, Eigen::Matrix4d> BuildGraphCurrentPoseMap(
    const PoseGraphProblem& graph,
    const GlobalPoseStore& pose_store)
{
    std::map<RawKeyFrameId, Eigen::Matrix4d> poses;
    for (const auto& vertex : graph.vertices)
    {
        const auto pose = pose_store.GetWorldPose(vertex.keyframe_id);
        if (pose)
        {
            poses[vertex.keyframe_id] = pose.value();
        }
    }
    return poses;
}

RetrySuggestion BuildRetrySuggestion(const PostApplyValidationResult& result)
{
    RetrySuggestion retry;
    retry.retry_allowed = true;
    if (!result.anchor_preservation_ok)
    {
        retry.strategy = "DIFFERENT_VERTEX_SELECTION";
        retry.reason = result.anchor_preservation_check.reason.empty()
                           ? "anchor_preservation_failed"
                           : result.anchor_preservation_check.reason;
    }
    else if (result.fixed_check.hard_fixed_moved)
    {
        retry.retry_allowed = false;
        retry.strategy = "BUG_FIXED_SELECTION";
        retry.reason = "hard_fixed_moved";
    }
    else if (!result.primary_error_improved)
    {
        retry.strategy = "WIDER_WINDOW";
        retry.reason = "primary_error_not_improved";
    }
    else if (result.decision == PostApplyDecision::PartialKeepForNextPass)
    {
        retry.strategy = "REBUILD_GRAPH_FROM_PARTIAL_STATE";
        retry.reason = "partial_needs_second_pass";
    }
    else if (!result.internal_edges_ok)
    {
        retry.strategy = "STRONGER_INTERNAL_EDGES";
        retry.reason = "internal_edges_broken";
    }
    else if (!result.propagation_ok)
    {
        retry.strategy = "DIFFERENT_VERTEX_SELECTION";
        retry.reason = "propagation_discontinuity";
    }
    else
    {
        retry.strategy = "CHECK_FIDUCIAL_ASSOCIATION";
        retry.reason = result.reason.empty() ? "post_apply_rejected" : result.reason;
    }
    return retry;
}

double ComputeCost(const PoseGraphProblem& graph,
                   const std::map<RawKeyFrameId, Eigen::Matrix4d>& poses)
{
    double cost = 0.0;
    for (const auto& prior : graph.priors)
    {
        const auto pose_it = poses.find(prior.keyframe_id);
        if (pose_it == poses.end())
        {
            continue;
        }
        const double t = TranslationError(pose_it->second, prior.target_world_T_kf);
        const Eigen::Vector3d rotation_delta =
            RotationVector(prior.target_world_T_kf.block<3, 3>(0, 0).transpose() *
                           pose_it->second.block<3, 3>(0, 0));
        const double soft_rotation_scale =
            PriorRotationResidualScale(prior);
        const double soft_translation_scale =
            prior.prior_type == PoseGraphPriorType::CurrentPoseSoft
                ? kCurrentPoseSoftTranslationScale
                : 1.0;
        cost += prior.weight_translation * soft_translation_scale * t * t +
                prior.weight_rotation * soft_rotation_scale *
                    rotation_delta.squaredNorm();
    }

    const auto target_tail_pose_locks = BuildTargetTailPoseLocks(graph);
    for (const auto& [keyframe_id, locked_pose] : target_tail_pose_locks)
    {
        const auto pose_it = poses.find(keyframe_id);
        if (pose_it == poses.end())
        {
            continue;
        }
        const double t = TranslationError(pose_it->second, locked_pose);
        const Eigen::Vector3d rotation_delta =
            RotationVector(locked_pose.block<3, 3>(0, 0).transpose() *
                           pose_it->second.block<3, 3>(0, 0));
        cost += kTargetTailPoseLockTranslationScale *
                    kTargetTailPoseLockTranslationScale * t * t +
                kTargetTailPoseLockRotationScale *
                kTargetTailPoseLockRotationScale *
                    rotation_delta.squaredNorm();
    }
    const auto previous_anchor_tail_pose_locks =
        BuildPreviousAnchorTailPoseLocks(graph);
    for (const auto& [keyframe_id, locked_pose] : previous_anchor_tail_pose_locks)
    {
        const auto pose_it = poses.find(keyframe_id);
        if (pose_it == poses.end())
        {
            continue;
        }
        const double t = TranslationError(pose_it->second, locked_pose);
        const Eigen::Vector3d rotation_delta =
            RotationVector(locked_pose.block<3, 3>(0, 0).transpose() *
                           pose_it->second.block<3, 3>(0, 0));
        cost += kTargetTailPoseLockTranslationScale *
                    kTargetTailPoseLockTranslationScale * t * t +
                kTargetTailPoseLockRotationScale *
                    kTargetTailPoseLockRotationScale *
                    rotation_delta.squaredNorm();
    }

    for (const auto& edge : graph.edges)
    {
        const auto from_it = poses.find(edge.from_keyframe_id);
        const auto to_it = poses.find(edge.to_keyframe_id);
        if (from_it == poses.end() || to_it == poses.end())
        {
            continue;
        }
        const Eigen::Matrix4d current_relative =
            from_it->second.inverse() * to_it->second;
        const PoseGraphVertex* from_vertex =
            FindVertex(graph, edge.from_keyframe_id);
        const PoseGraphVertex* to_vertex =
            FindVertex(graph, edge.to_keyframe_id);
        if (!from_vertex || !to_vertex)
        {
            continue;
        }
        const Eigen::Vector3d current_t =
            current_relative.block<3, 1>(0, 3);
        const Eigen::Vector3d expected_t =
            edge.relative_T_from_to.block<3, 1>(0, 3);
        const double spatial_length_delta =
            current_t.norm() - expected_t.norm();
        const double planar_length_delta =
            current_t.head<2>().norm() - expected_t.head<2>().norm();
        const Eigen::Vector2d planar_direction_delta =
            current_t.head<2>() - expected_t.head<2>();
        const double vertical_delta = current_t.z() - expected_t.z();
        const double global_vertical_delta =
            GlobalTranslationDelta(from_it->second, to_it->second).z() -
            GlobalTranslationDelta(
                from_vertex->initial_world_T_kf,
                to_vertex->initial_world_T_kf).z();
        const double translation_rigidity =
            EdgeTranslationRigidity(graph, edge);
        const double translation_cost =
            translation_rigidity * translation_rigidity *
            (
            kEdgePlanarLengthResidualScale * kEdgePlanarLengthResidualScale *
                planar_length_delta * planar_length_delta +
            kEdgePlanarDirectionResidualScale * kEdgePlanarDirectionResidualScale *
                planar_direction_delta.squaredNorm() +
            kEdgeVerticalResidualScale * kEdgeVerticalResidualScale *
                vertical_delta * vertical_delta +
            kEdgeGlobalVerticalResidualScale *
                kEdgeGlobalVerticalResidualScale *
                global_vertical_delta * global_vertical_delta +
            kEdgeSpatialLengthResidualScale * kEdgeSpatialLengthResidualScale *
                spatial_length_delta * spatial_length_delta);
        const Eigen::Vector3d rotation_delta =
            RotationVector(edge.relative_T_from_to.block<3, 3>(0, 0).transpose() *
                           current_relative.block<3, 3>(0, 0));
        const double angular_rigidity =
            ParabolicAngularRigidity(graph, edge);
        const double rotation_cost =
            angular_rigidity * angular_rigidity *
            kEdgeRotationResidualScale * kEdgeRotationResidualScale *
            rotation_delta.squaredNorm();
        cost += edge.weight * (translation_cost + rotation_cost);
    }

    for (const auto& entry : graph.propagation_plan)
    {
        if (entry.mode != PoseGraphPropagationMode::PathSegment ||
            !entry.control_vertex_b)
        {
            continue;
        }
        const auto a_it = poses.find(entry.control_vertex_a);
        const auto b_it = poses.find(entry.control_vertex_b.value());
        const auto* debug = FindDebugKeyFrame(graph, entry.affected_keyframe_id);
        if (a_it == poses.end() || b_it == poses.end() || !debug)
        {
            continue;
        }
        const Eigen::Matrix4d current =
            InterpolatePose(a_it->second, b_it->second, entry.segment_alpha);
        const Eigen::Vector3d translation_residual =
            current.block<3, 1>(0, 3) -
            debug->map_world_T_kf.block<3, 1>(0, 3);
        const Eigen::Vector3d rotation_residual =
            RotationVector(
                debug->map_world_T_kf.block<3, 3>(0, 0).transpose() *
                current.block<3, 3>(0, 0));
        cost += kPropagatedKeyFrameShapeTranslationScale *
                    kPropagatedKeyFrameShapeTranslationScale *
                    translation_residual.squaredNorm() +
                kPropagatedKeyFrameShapeRotationScale *
                    kPropagatedKeyFrameShapeRotationScale *
                    rotation_residual.squaredNorm();
    }

    std::map<RawKeyFrameId, PoseState6D> states_by_id;
    for (const auto& vertex : graph.vertices)
    {
        PoseState6D state = MakePoseState6D(vertex.initial_world_T_kf);
        const auto pose_it = poses.find(vertex.keyframe_id);
        if (pose_it != poses.end())
        {
            state.state.segment<3>(0) =
                pose_it->second.block<3, 1>(0, 3);
            const Eigen::Vector3d rotation_delta =
                RotationVector(
                    pose_it->second.block<3, 3>(0, 0) *
                    vertex.initial_world_T_kf.block<3, 3>(0, 0).transpose());
            state.state.segment<3>(3) = rotation_delta;
        }
        states_by_id[vertex.keyframe_id] = state;
    }
    for (const auto& edge : graph.edges)
    {
        const auto from_state_it = states_by_id.find(edge.from_keyframe_id);
        const auto to_state_it = states_by_id.find(edge.to_keyframe_id);
        if (from_state_it == states_by_id.end() ||
            to_state_it == states_by_id.end())
        {
            continue;
        }
        const double translation_gradient_rigidity =
            CorrectionGradientSupportRigidity(edge);
        const double angular_gradient_rigidity =
            ParabolicCorrectionAngularGradientRigidity(graph, edge);
        Vector6d gradient = Vector6d::Zero();
        gradient.segment<3>(0) =
            (to_state_it->second.state.segment<3>(0) -
             to_state_it->second.initial_world_T_kf.block<3, 1>(0, 3)) -
            (from_state_it->second.state.segment<3>(0) -
             from_state_it->second.initial_world_T_kf.block<3, 1>(0, 3));
        gradient.segment<3>(3) =
            to_state_it->second.state.segment<3>(3) -
            from_state_it->second.state.segment<3>(3);
        const double translation_cost =
            kCorrectionGradientTranslationScale *
            kCorrectionGradientTranslationScale *
            gradient.segment<3>(0).squaredNorm();
        const double rotation_cost =
            kCorrectionGradientRotationScale *
            kCorrectionGradientRotationScale *
            gradient.segment<3>(3).squaredNorm();
        cost += edge.weight *
                (translation_gradient_rigidity * translation_gradient_rigidity *
                     translation_cost +
                 angular_gradient_rigidity * angular_gradient_rigidity *
                     rotation_cost);
    }
    for (size_t i = 1; i < graph.edges.size(); ++i)
    {
        const auto& previous_edge = graph.edges[i - 1];
        const auto& next_edge = graph.edges[i];
        if (!(previous_edge.to_keyframe_id == next_edge.from_keyframe_id))
        {
            continue;
        }
        const auto a_it = poses.find(previous_edge.from_keyframe_id);
        const auto b_it = poses.find(previous_edge.to_keyframe_id);
        const auto c_it = poses.find(next_edge.to_keyframe_id);
        if (a_it == poses.end() || b_it == poses.end() || c_it == poses.end())
        {
            continue;
        }
        const auto a_state_it = states_by_id.find(previous_edge.from_keyframe_id);
        const auto b_state_it = states_by_id.find(previous_edge.to_keyframe_id);
        const auto c_state_it = states_by_id.find(next_edge.to_keyframe_id);
        if (a_state_it == states_by_id.end() ||
            b_state_it == states_by_id.end() ||
            c_state_it == states_by_id.end())
        {
            continue;
        }
        const double expected_turn =
            SignedPlanarTurn(
                a_state_it->second.initial_world_T_kf,
                b_state_it->second.initial_world_T_kf,
                c_state_it->second.initial_world_T_kf);
        const double current_turn =
            SignedPlanarTurn(a_it->second, b_it->second, c_it->second);
        const double delta_turn =
            NormalizeAngle(current_turn - expected_turn);
        const double hinge_rigidity =
            CornerHingeRigidity(graph, previous_edge, next_edge, expected_turn);
        cost += 0.5 * (previous_edge.weight + next_edge.weight) *
                kCornerHingeResidualScale * kCornerHingeResidualScale *
                hinge_rigidity * hinge_rigidity *
                delta_turn * delta_turn;
    }
    return cost;
}

}  // namespace

OptimizationManager::OptimizationManager(const OptimizationManagerConfig& config)
    : config_(config)
{
}

void OptimizationManager::Configure(const OptimizationManagerConfig& config)
{
    config_ = config;
}

const OptimizationManagerConfig& OptimizationManager::GetConfig() const
{
    return config_;
}

OptimizationDryRunResult OptimizationManager::RunDryRun(
    const PoseGraphProblem& graph,
    const RawMapDatabase& raw_db,
    const GlobalPoseStore& pose_store) const
{
    return RunDryRunInternal(graph, &raw_db, &pose_store, true);
}

OptimizationDryRunResult OptimizationManager::RunDryRunGraphOnly(
    const PoseGraphProblem& graph) const
{
    return RunDryRunInternal(graph, nullptr, nullptr, false);
}

OptimizationDryRunResult OptimizationManager::RunDryRunInternal(
    const PoseGraphProblem& graph,
    const RawMapDatabase* raw_db,
    const GlobalPoseStore* pose_store,
    bool include_propagation) const
{
    OptimizationDryRunResult result;
    result.task_id = graph.task_id;
    result.task_type = graph.task_type;
    result.anchor_preservation_required = graph.anchor_preservation.required;
    result.anchor_preservation_graph_satisfied = graph.anchor_preservation.satisfied;
    result.previous_fiducial_fixed_count =
        graph.anchor_preservation.previous_fiducial_fixed_count;
    result.previous_fiducial_neighborhood_fixed_count =
        graph.anchor_preservation.previous_fiducial_neighborhood_fixed_count;

    const PoseGraphPrior* target_prior = FindTargetPrior(graph);
    if (graph.vertices.empty())
    {
        result.reason = "no_vertices";
        return result;
    }
    if (graph.variable_keyframes.empty())
    {
        result.reason = "no_variable_vertices";
        return result;
    }
    if (!target_prior)
    {
        result.reason = "no_target_prior";
        return result;
    }
    for (const auto& prior : graph.priors)
    {
        if (prior.weight_translation <= 0.0 || prior.weight_rotation <= 0.0)
        {
            result.reason = "non_positive_prior_weight";
            return result;
        }
        if (!IsFiniteTransform(prior.target_world_T_kf))
        {
            result.reason = "non_finite_prior";
            return result;
        }
    }
    for (const auto& edge : graph.edges)
    {
        if (edge.weight <= 0.0)
        {
            result.reason = "non_positive_edge_weight";
            return result;
        }
        if (!IsFiniteTransform(edge.relative_T_from_to))
        {
            result.reason = "non_finite_edge";
            return result;
        }
    }
    for (const auto& vertex : graph.vertices)
    {
        if (!IsFiniteTransform(vertex.initial_world_T_kf))
        {
            result.reason = "non_finite_vertex_pose";
            return result;
        }
        if (vertex.is_hard_fiducial && vertex.is_variable)
        {
            result.reason = "hard_fiducial_marked_variable";
            return result;
        }
    }
    for (const auto& entry : graph.propagation_plan)
    {
        if ((include_propagation &&
             (!raw_db || !raw_db->HasKeyFrame(entry.affected_keyframe_id))) ||
            !HasVertex(graph, entry.control_vertex_a) ||
            (entry.control_vertex_b && !HasVertex(graph, entry.control_vertex_b.value())))
        {
            result.reason = "invalid_propagation_reference";
            return result;
        }
    }

    result.precheck_ok = true;

    std::map<RawKeyFrameId, Eigen::Matrix4d> poses_before;
    std::map<RawKeyFrameId, Eigen::Matrix4d> poses_trial;
    std::set<RawKeyFrameId> fixed_keyframes;
    std::set<RawKeyFrameId> variable_keyframes;
    for (const auto& vertex : graph.vertices)
    {
        poses_before[vertex.keyframe_id] = vertex.initial_world_T_kf;
        poses_trial[vertex.keyframe_id] = vertex.initial_world_T_kf;
        if (IsHardFixedVertex(vertex))
        {
            fixed_keyframes.insert(vertex.keyframe_id);
        }
        if (vertex.is_variable)
        {
            variable_keyframes.insert(vertex.keyframe_id);
        }
    }
    result.copied_poses = poses_before.size();
    result.fixed_kfs = fixed_keyframes.size();

    result.initial_cost = ComputeCost(graph, poses_before);
    const auto target_pose_before_it = poses_before.find(target_prior->keyframe_id);
    if (target_pose_before_it == poses_before.end())
    {
        result.reason = "target_pose_missing";
        return result;
    }
    result.before_error_t =
        TranslationError(target_pose_before_it->second, target_prior->target_world_T_kf);
    result.before_error_yaw =
        std::abs(NormalizeAngle(
            YawFromTransform(target_prior->target_world_T_kf) -
            YawFromTransform(target_pose_before_it->second)));

    const auto solve_result = SolveWeightedPlanarPoseGraph(graph);
    if (!solve_result.success)
    {
        result.reason = "pose_graph_solver_failed";
        return result;
    }
    poses_trial = solve_result.poses;
    result.solver_iterations = solve_result.iterations;
    result.final_cost = ComputeCost(graph, poses_trial);
    const auto target_pose_after_it = poses_trial.find(target_prior->keyframe_id);
    result.after_error_t =
        TranslationError(target_pose_after_it->second, target_prior->target_world_T_kf);
    result.after_error_yaw =
        std::abs(NormalizeAngle(
            YawFromTransform(target_prior->target_world_T_kf) -
            YawFromTransform(target_pose_after_it->second)));

    double total_delta_t = 0.0;
    for (const auto& vertex : graph.vertices)
    {
        OptimizationPoseProposal proposal;
        proposal.keyframe_id = vertex.keyframe_id;
        proposal.before_world_T_kf = poses_before[vertex.keyframe_id];
        proposal.proposed_world_T_kf = poses_trial[vertex.keyframe_id];
        proposal.variable_vertex = vertex.is_variable;
        proposal.fixed_vertex = IsHardFixedVertex(vertex);
        proposal.delta_t_m =
            TranslationError(proposal.before_world_T_kf, proposal.proposed_world_T_kf);
        proposal.delta_yaw_rad =
            std::abs(NormalizeAngle(
                YawFromTransform(proposal.proposed_world_T_kf) -
                YawFromTransform(proposal.before_world_T_kf)));
        if (!proposal.proposed_world_T_kf.allFinite())
        {
            result.has_nan = true;
        }
        if (proposal.fixed_vertex && proposal.delta_t_m > 1e-9)
        {
            result.hard_fixed_moved = true;
        }
        if (proposal.delta_t_m > 1e-6 || proposal.delta_yaw_rad > 1e-6)
        {
            ++result.moved_kfs;
        }
        result.max_delta_t = std::max(result.max_delta_t, proposal.delta_t_m);
        result.max_delta_yaw = std::max(result.max_delta_yaw, proposal.delta_yaw_rad);
        // 1J: 1I puede marcar fiduciales previos como frontera fija de rama.
        // El dry-run debe exponer si esos anchors permanecen quietos antes de
        // que 1K considere cualquier escritura persistente.
        if (vertex.selection_reason == "previous_fiducial_anchor")
        {
            result.max_previous_fiducial_delta_t =
                std::max(result.max_previous_fiducial_delta_t, proposal.delta_t_m);
            result.max_previous_fiducial_delta_yaw =
                std::max(result.max_previous_fiducial_delta_yaw, proposal.delta_yaw_rad);
        }
        if (vertex.is_anchor_neighborhood)
        {
            result.max_previous_fiducial_neighborhood_delta_t =
                std::max(result.max_previous_fiducial_neighborhood_delta_t,
                         proposal.delta_t_m);
            result.max_previous_fiducial_neighborhood_delta_yaw =
                std::max(result.max_previous_fiducial_neighborhood_delta_yaw,
                         proposal.delta_yaw_rad);
        }
        total_delta_t += proposal.delta_t_m;
        result.proposed_vertex_poses.push_back(proposal);
    }
    if (!graph.vertices.empty())
    {
        result.mean_delta_t = total_delta_t / static_cast<double>(graph.vertices.size());
    }

    double propagated_total_delta_t = 0.0;
    if (include_propagation && pose_store)
    {
        for (const auto& entry : graph.propagation_plan)
        {
            const auto before = pose_store->GetWorldPose(entry.affected_keyframe_id);
            const auto control_before_it = poses_before.find(entry.control_vertex_a);
            const auto control_after_it = poses_trial.find(entry.control_vertex_a);
            if (!before ||
                control_before_it == poses_before.end() ||
                control_after_it == poses_trial.end())
            {
                continue;
            }

            Eigen::Matrix4d proposed_pose =
                ApplyScaledEndpointDelta(
                    before.value(),
                    control_before_it->second,
                    control_after_it->second,
                    1.0);
            if (entry.mode == PoseGraphPropagationMode::PathSegment &&
                entry.control_vertex_b)
            {
                const auto control_b_before_it =
                    poses_before.find(entry.control_vertex_b.value());
                const auto control_b_after_it =
                    poses_trial.find(entry.control_vertex_b.value());
                if (control_b_before_it != poses_before.end() &&
                    control_b_after_it != poses_trial.end())
                {
                    // 1K/propuesta de propagacion: PoseGraphBuilder calcula
                    // alpha con longitud acumulada del tramo. Reusarlo evita
                    // que la propagacion dependa solo del ID local cuando se
                    // insertan vertices por esquinas o splits de aristas.
                    const double alpha = entry.segment_alpha;
                    proposed_pose = ApplyInterpolatedControlDelta(
                        before.value(),
                        control_before_it->second,
                        control_after_it->second,
                        control_b_before_it->second,
                        control_b_after_it->second,
                        alpha);
                }
            }

            OptimizationPoseProposal proposal;
            proposal.keyframe_id = entry.affected_keyframe_id;
            proposal.before_world_T_kf = before.value();
            proposal.proposed_world_T_kf = proposed_pose;
            proposal.propagated = true;
            proposal.delta_t_m =
                TranslationError(proposal.before_world_T_kf, proposal.proposed_world_T_kf);
            proposal.delta_yaw_rad =
                std::abs(NormalizeAngle(
                    YawFromTransform(proposal.proposed_world_T_kf) -
                    YawFromTransform(proposal.before_world_T_kf)));
            result.max_propagated_delta_t =
                std::max(result.max_propagated_delta_t, proposal.delta_t_m);
            propagated_total_delta_t += proposal.delta_t_m;
            ++result.propagated_kfs;
            result.proposed_propagated_poses.push_back(proposal);
        }
    }
    if (result.propagated_kfs > 0)
    {
        result.mean_propagated_delta_t =
            propagated_total_delta_t / static_cast<double>(result.propagated_kfs);
    }

    if (result.before_error_t > 1e-9)
    {
        result.improvement_ratio =
            (result.before_error_t - result.after_error_t) / result.before_error_t;
    }

    result.success = true;
    result.reason = "dryrun_completed";

    const bool cost_ok =
        !config_.dryrun_require_cost_decrease ||
        result.final_cost < result.initial_cost;
    const bool error_ok = result.after_error_t < result.before_error_t;
    const bool improvement_ok =
        result.improvement_ratio >= config_.dryrun_min_improvement_ratio;
    const bool solver_yaw_ok =
        result.max_delta_yaw <= config_.dryrun_max_allowed_delta_yaw ||
        (result.solver_iterations > 1 &&
         result.after_error_t <= config_.dryrun_max_final_error_t &&
         result.max_delta_yaw <= kPi);
    const bool delta_ok =
        result.max_delta_t <= config_.dryrun_max_allowed_delta_t &&
        solver_yaw_ok;
    const bool partial_delta_t_ok =
        result.max_delta_t <= config_.dryrun_partial_max_allowed_delta_t;
    const bool final_error_ok =
        result.after_error_t <= config_.dryrun_max_final_error_t;
    const bool partial_improvement_ok =
        result.improvement_ratio >= config_.dryrun_partial_min_improvement_ratio;
    result.anchor_preservation_ok =
        !result.anchor_preservation_required ||
        (result.anchor_preservation_graph_satisfied &&
        result.previous_fiducial_fixed_count > 0 &&
         result.max_previous_fiducial_delta_t <= 1e-9 &&
         result.max_previous_fiducial_delta_yaw <= 1e-9 &&
         !result.hard_fixed_moved);
    result.useful =
        cost_ok &&
        error_ok &&
        improvement_ok &&
        delta_ok &&
        final_error_ok &&
        result.anchor_preservation_ok &&
        !result.hard_fixed_moved &&
        !result.has_nan;
    // 1J: los errores extremos que bajan mucho pero siguen fuera del umbral
    // no son aceptables para apply normal. El umbral yaw de `useful` es muy
    // estricto para aceptacion directa; aqui se marca un candidato parcial
    // para que 1K lo aplique solo con backup y 1L valide el estado real.
    result.partial_candidate =
        !result.useful &&
        cost_ok &&
        error_ok &&
        partial_improvement_ok &&
        partial_delta_t_ok &&
        !final_error_ok &&
        result.anchor_preservation_ok &&
        !result.hard_fixed_moved &&
        !result.has_nan;
    if (result.partial_candidate)
    {
        result.partial_reason = "large_error_reduced_but_above_threshold";
    }

    if (result.useful)
    {
        result.decision_reason = "error_reduced";
    }
    else if (result.partial_candidate)
    {
        result.decision_reason = result.partial_reason;
    }
    else if (!result.anchor_preservation_ok)
    {
        result.decision_reason = "anchor_preservation_failed";
    }
    else if (!cost_ok)
    {
        result.decision_reason = "cost_not_reduced";
    }
    else if (!error_ok)
    {
        result.decision_reason = "task_error_not_reduced";
    }
    else if (!improvement_ok)
    {
        result.decision_reason = "insufficient_improvement";
    }
    else if (!delta_ok)
    {
        result.decision_reason = "delta_too_large";
    }
    else if (!final_error_ok)
    {
        result.decision_reason = "final_error_too_high";
    }
    else if (result.hard_fixed_moved)
    {
        result.decision_reason = "hard_fixed_would_move";
    }
    else if (result.has_nan)
    {
        result.decision_reason = "non_finite_result";
    }
    else
    {
        result.decision_reason = "not_useful";
    }
    return result;
}

OptimizationApplyResult OptimizationManager::ApplyUsefulResult(
    const OptimizationDryRunResult& dry_run,
    const PoseGraphProblem& graph,
    const RawMapDatabase& raw_db,
    GlobalPoseStore& pose_store) const
{
    return ApplyCandidateResult(
        dry_run,
        graph,
        raw_db,
        pose_store,
        false);
}

OptimizationApplyResult OptimizationManager::ApplyCandidateResult(
    const OptimizationDryRunResult& dry_run,
    const PoseGraphProblem& graph,
    const RawMapDatabase& raw_db,
    GlobalPoseStore& pose_store,
    bool allow_partial_candidate) const
{
    // 1K: esta funcion cruza la frontera que 1J no podia cruzar: escribe
    // poses persistentes del servidor. Los parciales solo entran si el servidor
    // preparo backup; 1L observara despues el estado real y decidira commit,
    // segundo pase o rollback.
    OptimizationApplyResult result;
    result.task_id = dry_run.task_id;
    result.task_type = dry_run.task_type;
    result.fixed_kfs = dry_run.fixed_kfs;
    result.hard_fixed_moved = dry_run.hard_fixed_moved;
    result.max_delta_t = dry_run.max_delta_t;
    result.mean_delta_t = dry_run.mean_delta_t;
    result.max_delta_yaw = dry_run.max_delta_yaw;
    result.raw_db_modified = false;

    if (dry_run.task_id != graph.task_id)
    {
        result.reason = "task_id_mismatch";
        return result;
    }
    if (!dry_run.success)
    {
        result.reason = "dryrun_not_successful";
        return result;
    }
    const bool anchor_preservation_required =
        dry_run.anchor_preservation_required || graph.anchor_preservation.required;
    const bool anchor_preservation_satisfied =
        dry_run.anchor_preservation_graph_satisfied &&
        (!graph.anchor_preservation.required || graph.anchor_preservation.satisfied);
    const uint64_t previous_fiducial_fixed_count =
        std::max(
            dry_run.previous_fiducial_fixed_count,
            graph.anchor_preservation.previous_fiducial_fixed_count);
    // 1K: el apply es la primera escritura persistente. Si 1I/1J
    // dijeron que habia que preservar un fiducial previo, esta ruta no puede
    // aceptar un grafo que vuelva a llegar sin ese anclaje fijo demostrado.
    if (anchor_preservation_required)
    {
        if (!anchor_preservation_satisfied ||
            previous_fiducial_fixed_count == 0 ||
            dry_run.fixed_kfs == 0)
        {
            result.reason = "missing_previous_fiducial_anchor";
            return result;
        }
        if (!dry_run.anchor_preservation_ok)
        {
            result.reason = "anchor_preservation_failed";
            return result;
        }
    }
    const bool applying_partial =
        allow_partial_candidate && dry_run.partial_candidate && !dry_run.useful;
    // 1K: un error fiducial absurdo puede requerir una correccion grande. El
    // apply la etiqueta como checkpoint progresivo para que 1L pueda exigir
    // segundo pase o rollback. Los KFs no-vertice ya existentes tambien se
    // propagan para que la nube publicada no quede partida.
    const bool applying_progressive_checkpoint =
        applying_partial ||
        (allow_partial_candidate &&
         dry_run.before_error_t >= config_.post_apply_fiducial_absurd_error_t &&
         dry_run.max_delta_t > config_.dryrun_max_final_error_t);
    if (dry_run.partial_candidate && !dry_run.useful && !applying_partial)
    {
        result.reason = "partial_candidate_requires_f1l";
        return result;
    }
    if (!dry_run.useful && !applying_partial)
    {
        result.reason = "dryrun_not_useful";
        return result;
    }
    if (config_.dryrun_require_cost_decrease &&
        !(dry_run.final_cost < dry_run.initial_cost))
    {
        result.reason = "cost_not_reduced";
        return result;
    }
    if (!(dry_run.after_error_t < dry_run.before_error_t))
    {
        result.reason = "task_error_not_reduced";
        return result;
    }
    if (dry_run.has_nan)
    {
        result.reason = "non_finite_dryrun";
        return result;
    }
    if (dry_run.hard_fixed_moved)
    {
        result.reason = "hard_fixed_would_move";
        return result;
    }
    const bool solver_yaw_ok =
        dry_run.max_delta_yaw <= config_.dryrun_max_allowed_delta_yaw ||
        (dry_run.solver_iterations > 1 &&
         dry_run.after_error_t <= config_.dryrun_max_final_error_t &&
         dry_run.max_delta_yaw <= kPi);
    if (!applying_partial &&
        (dry_run.max_delta_t > config_.dryrun_max_allowed_delta_t ||
         !solver_yaw_ok))
    {
        result.reason = "delta_too_large";
        return result;
    }
    if (applying_partial &&
        dry_run.max_delta_t > config_.dryrun_partial_max_allowed_delta_t)
    {
        result.reason = "partial_delta_too_large";
        return result;
    }
    for (const auto& proposal : dry_run.proposed_vertex_poses)
    {
        if (!raw_db.HasKeyFrame(proposal.keyframe_id))
        {
            result.reason = "raw_keyframe_missing";
            return result;
        }
        if (!pose_store.HasWorldPose(proposal.keyframe_id))
        {
            result.reason = "world_pose_missing";
            return result;
        }
        if (!IsFiniteTransform(proposal.proposed_world_T_kf))
        {
            result.reason = "non_finite_vertex_proposal";
            return result;
        }
        if (proposal.fixed_vertex && proposal.delta_t_m > 1e-9)
        {
            result.hard_fixed_moved = true;
            result.reason = "fixed_vertex_would_move";
            return result;
        }
        if (pose_store.IsHardFiducialKeyFrame(proposal.keyframe_id) &&
            proposal.delta_t_m > 1e-9)
        {
            result.hard_fixed_moved = true;
            result.reason = "hard_fiducial_would_move";
            return result;
        }
    }
    for (const auto& proposal : dry_run.proposed_propagated_poses)
    {
        if (!raw_db.HasKeyFrame(proposal.keyframe_id))
        {
            result.reason = "raw_propagated_keyframe_missing";
            return result;
        }
        if (!pose_store.HasWorldPose(proposal.keyframe_id))
        {
            result.reason = "propagated_world_pose_missing";
            return result;
        }
        if (!IsFiniteTransform(proposal.proposed_world_T_kf))
        {
            result.reason = "non_finite_propagated_proposal";
            return result;
        }
        if (pose_store.IsHardFiducialKeyFrame(proposal.keyframe_id) &&
            proposal.delta_t_m > 1e-9)
        {
            result.hard_fixed_moved = true;
            result.reason = "hard_fiducial_propagation_would_move";
            return result;
        }
    }

    result.precheck_ok = true;

    constexpr double kMoveEpsilon = 1e-9;
    bool has_newest_applied_keyframe = false;
    RawKeyFrameId newest_applied_keyframe;
    auto note_applied_keyframe = [&](const RawKeyFrameId& keyframe_id)
    {
        if (!has_newest_applied_keyframe ||
            keyframe_id.local_kf_id > newest_applied_keyframe.local_kf_id)
        {
            newest_applied_keyframe = keyframe_id;
            has_newest_applied_keyframe = true;
        }
    };

    for (const auto& proposal : dry_run.proposed_vertex_poses)
    {
        if (!proposal.variable_vertex ||
            proposal.fixed_vertex ||
            proposal.delta_t_m <= kMoveEpsilon)
        {
            continue;
        }

        const std::string source = applying_progressive_checkpoint
            ? "SERVER_OPTIMIZATION_PARTIAL"
            : "SERVER_OPTIMIZATION";
        const auto pose_result =
            pose_store.SetOptimizedKeyFramePose(
                proposal.keyframe_id,
                proposal.proposed_world_T_kf,
                raw_db,
                source);
        auto record = MakeApplyRecord(proposal, pose_result, false, source);
        result.optimized_records.push_back(record);
        if (!pose_result.success)
        {
            result.reason = pose_result.reason;
            return result;
        }
        ++result.optimized_kfs;
        note_applied_keyframe(proposal.keyframe_id);
    }

    for (const auto& proposal : dry_run.proposed_propagated_poses)
    {
        if (proposal.delta_t_m <= kMoveEpsilon)
        {
            continue;
        }

        const std::string source = applying_progressive_checkpoint
            ? "SERVER_OPTIMIZATION_PARTIAL_PROPAGATED"
            : "SERVER_OPTIMIZATION_PROPAGATED";
        const auto pose_result =
            pose_store.SetPropagatedKeyFramePose(
                proposal.keyframe_id,
                proposal.proposed_world_T_kf,
                raw_db,
                source);
        auto record = MakeApplyRecord(proposal, pose_result, true, source);
        result.propagated_records.push_back(record);
        if (!pose_result.success)
        {
            result.reason = pose_result.reason;
            return result;
        }
        ++result.propagated_kfs;
        note_applied_keyframe(proposal.keyframe_id);
    }

    if (result.optimized_kfs == 0 && result.propagated_kfs == 0)
    {
        result.reason = "no_moved_keyframes_to_apply";
        return result;
    }

    if (has_newest_applied_keyframe)
    {
        const std::string source = applying_progressive_checkpoint
            ? "SERVER_OPTIMIZATION_PARTIAL_FUTURE_INHERIT"
            : "SERVER_OPTIMIZATION_FUTURE_INHERIT";
        const auto correction_result =
            pose_store.SetSubmapLastServerCorrectionFromKeyFrame(
                newest_applied_keyframe,
                source);
        if (!correction_result.success)
        {
            result.reason = correction_result.reason;
            return result;
        }
        result.last_correction_set = true;
        result.last_correction_submap = correction_result.submap_id;
        result.last_correction_from_keyframe = newest_applied_keyframe;
    }

    result.applied = true;
    result.global_pose_store_modified = true;
    result.reason = "applied";
    return result;
}

PostApplyValidationResult OptimizationManager::ValidatePostApply(
    const OptimizationDryRunResult& dry_run,
    const PoseGraphProblem& graph,
    const OptimizationApplyResult& apply,
    const GlobalPoseStore& pose_store,
    bool force_reject) const
{
    // 1L: esta validacion mira el estado real persistente tras el apply. No
    // reutiliza las poses temporales salvo como prediccion contra la que medir.
    PostApplyValidationResult result;
    result.task_id = dry_run.task_id;
    result.task_type = dry_run.task_type;
    result.forced_reject = force_reject;
    result.error.before_error_t = dry_run.before_error_t;
    result.error.predicted_after_error_t = dry_run.after_error_t;
    result.error.before_error_yaw = dry_run.before_error_yaw;
    result.error.predicted_after_error_yaw = dry_run.after_error_yaw;

    if (!apply.applied)
    {
        result.reason = "apply_not_applied";
        result.retry = BuildRetrySuggestion(result);
        return result;
    }

    const PoseGraphPrior* target_prior = FindTargetPrior(graph);
    if (!target_prior)
    {
        result.reason = "target_prior_missing";
        result.retry = BuildRetrySuggestion(result);
        return result;
    }

    const auto target_pose = pose_store.GetWorldPose(target_prior->keyframe_id);
    if (!target_pose)
    {
        result.reason = "target_world_pose_missing";
        result.retry = BuildRetrySuggestion(result);
        return result;
    }

    result.error.real_after_error_t =
        TranslationError(target_pose.value(), target_prior->target_world_T_kf);
    result.error.real_after_error_yaw =
        std::abs(NormalizeAngle(
            YawFromTransform(target_prior->target_world_T_kf) -
            YawFromTransform(target_pose.value())));
    if (result.error.before_error_t > 1e-9)
    {
        result.error.improvement_ratio =
            (result.error.before_error_t - result.error.real_after_error_t) /
            result.error.before_error_t;
    }
    result.primary_error_improved =
        result.error.real_after_error_t < result.error.before_error_t;
    result.final_error_ok =
        result.error.real_after_error_t <= config_.dryrun_max_final_error_t;

    const auto before_poses = BuildGraphInitialPoseMap(graph);
    const auto after_poses = BuildGraphCurrentPoseMap(graph, pose_store);
    const auto before_edges = ComputeEdgeResidualStats(graph, before_poses);
    const auto after_edges = ComputeEdgeResidualStats(graph, after_poses);
    result.internal_error.internal_mean_before = before_edges.mean_t;
    result.internal_error.internal_mean_after = after_edges.mean_t;
    result.internal_error.internal_max_before = before_edges.max_t;
    result.internal_error.internal_max_after = after_edges.max_t;
    for (size_t i = 0;
         i < before_edges.residuals_t.size() &&
         i < after_edges.residuals_t.size() &&
         i < graph.edges.size();
         ++i)
    {
        const bool broken =
            after_edges.residuals_t[i] > config_.post_apply_internal_broken_edge_t;
        const bool deformable =
            broken &&
            IsDeformableInternalEdgeForAbsurdPartial(
                graph,
                graph.edges[i],
                before_edges.residuals_t[i],
                after_edges.residuals_t[i],
                i < after_edges.residuals_yaw.size()
                    ? after_edges.residuals_yaw[i]
                    : 0.0,
                dry_run,
                result,
                config_);
        if (deformable)
        {
            ++result.internal_error.deformable_edges_checked;
            result.internal_error.max_deformable_internal_after =
                std::max(result.internal_error.max_deformable_internal_after,
                         after_edges.residuals_t[i]);
        }
        else
        {
            ++result.internal_error.strong_edges_checked;
            result.internal_error.max_strong_internal_after =
                std::max(result.internal_error.max_strong_internal_after,
                         after_edges.residuals_t[i]);
        }
        if (after_edges.residuals_t[i] >
            before_edges.residuals_t[i] + config_.post_apply_internal_max_growth_t)
        {
            ++result.internal_error.num_edges_worse;
        }
        if (broken)
        {
            ++result.internal_error.num_edges_broken;
            if (deformable)
            {
                ++result.internal_error.deformable_edges_broken;
            }
            else
            {
                ++result.internal_error.strong_edges_broken;
            }
        }
    }
    const bool strict_internal_edges_ok =
        result.internal_error.num_edges_broken == 0 &&
        result.internal_error.internal_max_after <=
            result.internal_error.internal_max_before +
                config_.post_apply_internal_max_growth_t;
    result.internal_edges_ok = strict_internal_edges_ok;

    constexpr double kFixedMoveEpsilonT = 1e-6;
    constexpr double kFixedMoveEpsilonYaw = 1e-6;
    for (const auto& vertex : graph.vertices)
    {
        if (!IsHardFixedVertex(vertex))
        {
            continue;
        }
        const auto pose = pose_store.GetWorldPose(vertex.keyframe_id);
        if (!pose)
        {
            result.fixed_check.ok = false;
            ++result.fixed_check.fixed_moved_count;
            continue;
        }
        const double delta_t =
            TranslationError(vertex.initial_world_T_kf, pose.value());
        const double delta_yaw =
            std::abs(NormalizeAngle(
                YawFromTransform(pose.value()) -
                YawFromTransform(vertex.initial_world_T_kf)));
        result.fixed_check.max_fixed_delta_t =
            std::max(result.fixed_check.max_fixed_delta_t, delta_t);
        result.fixed_check.max_fixed_delta_yaw =
            std::max(result.fixed_check.max_fixed_delta_yaw, delta_yaw);
        if (delta_t > kFixedMoveEpsilonT || delta_yaw > kFixedMoveEpsilonYaw)
        {
            result.fixed_check.ok = false;
            ++result.fixed_check.fixed_moved_count;
            if (IsHardFixedVertex(vertex) ||
                pose_store.IsHardFiducialKeyFrame(vertex.keyframe_id))
            {
                result.fixed_check.hard_fixed_moved = true;
            }
        }
    }
    result.fixed_ok = result.fixed_check.ok && !result.fixed_check.hard_fixed_moved;

    const bool partial_propagation_skipped =
        dry_run.partial_candidate && !dry_run.useful &&
        apply.skipped_propagated_kfs > 0;
    double propagated_total_delta = 0.0;
    for (const auto& proposal : dry_run.proposed_propagated_poses)
    {
        result.propagation_check.propagated_max_delta_t =
            std::max(result.propagation_check.propagated_max_delta_t,
                     proposal.delta_t_m);
        propagated_total_delta += proposal.delta_t_m;
        if (partial_propagation_skipped)
        {
            // 1L: si el parcial no se propaga por seguridad visual, no se
            // compara contra la propuesta temporal porque no debe estar escrita
            // en GlobalPoseStore. El segundo pase debe reconstruir el grafo.
            ++result.propagation_check.skipped_propagated_count;
            continue;
        }

        const auto pose = pose_store.GetWorldPose(proposal.keyframe_id);
        ++result.propagation_check.propagated_count;
        if (!pose)
        {
            result.propagation_check.ok = false;
            continue;
        }
        const double mismatch_t =
            TranslationError(proposal.proposed_world_T_kf, pose.value());
        const double mismatch_yaw =
            std::abs(NormalizeAngle(
                YawFromTransform(pose.value()) -
                YawFromTransform(proposal.proposed_world_T_kf)));
        result.propagation_check.propagation_discontinuity_max_t =
            std::max(
                result.propagation_check.propagation_discontinuity_max_t,
                mismatch_t);
        result.propagation_check.propagation_discontinuity_max_yaw =
            std::max(
                result.propagation_check.propagation_discontinuity_max_yaw,
                mismatch_yaw);
        if (mismatch_t > 1e-6 || mismatch_yaw > 1e-6)
        {
            result.propagation_check.ok = false;
        }
    }
    const uint64_t propagation_stat_count =
        result.propagation_check.propagated_count +
        result.propagation_check.skipped_propagated_count;
    if (propagation_stat_count > 0)
    {
        result.propagation_check.propagated_mean_delta_t =
            propagated_total_delta /
            static_cast<double>(propagation_stat_count);
    }
    result.propagation_ok = result.propagation_check.ok;

    // 1L: el fixed-check generico puede pasar de forma vacia si el
    // grafo llega sin anclaje previo. Esta comprobacion valida explicitamente
    // los fiduciales frontera elegidos por 1I antes de conservar un apply.
    constexpr double kAnchorMoveEpsilonT = 1e-6;
    constexpr double kAnchorMoveEpsilonYaw = 1e-6;
    result.anchor_preservation_check.required =
        dry_run.anchor_preservation_required ||
        graph.anchor_preservation.required;
    result.anchor_preservation_check.graph_satisfied =
        dry_run.anchor_preservation_graph_satisfied &&
        (!graph.anchor_preservation.required ||
         graph.anchor_preservation.satisfied);
    result.anchor_preservation_check.previous_fiducial_fixed_count =
        std::max(
            dry_run.previous_fiducial_fixed_count,
            graph.anchor_preservation.previous_fiducial_fixed_count);
    result.anchor_preservation_check.previous_fiducial_neighborhood_fixed_count =
        std::max(
            dry_run.previous_fiducial_neighborhood_fixed_count,
            graph.anchor_preservation.previous_fiducial_neighborhood_fixed_count);
    result.anchor_preservation_check.independent_branches =
        graph.anchor_preservation.independent_branches;
    result.anchor_preservation_check.subdivision_candidates =
        graph.anchor_preservation.subdivision_candidates;
    result.anchor_preservation_check.reason =
        result.anchor_preservation_check.required ? "ok" : "not_required";

    for (const auto& vertex : graph.vertices)
    {
        if (vertex.selection_reason != "previous_fiducial_anchor")
        {
            continue;
        }
        ++result.anchor_preservation_check.checked_branch_anchors;
        const auto pose = pose_store.GetWorldPose(vertex.keyframe_id);
        if (!pose)
        {
            result.anchor_preservation_check.ok = false;
            result.anchor_preservation_check.reason = "missing_branch_anchor";
            continue;
        }
        const double delta_t =
            TranslationError(vertex.initial_world_T_kf, pose.value());
        const double delta_yaw =
            std::abs(NormalizeAngle(
                YawFromTransform(pose.value()) -
                YawFromTransform(vertex.initial_world_T_kf)));
        result.anchor_preservation_check.max_previous_fiducial_delta_t =
            std::max(
                result.anchor_preservation_check.max_previous_fiducial_delta_t,
                delta_t);
        result.anchor_preservation_check.max_previous_fiducial_delta_yaw =
            std::max(
                result.anchor_preservation_check.max_previous_fiducial_delta_yaw,
                delta_yaw);
        if (delta_t > kAnchorMoveEpsilonT ||
            delta_yaw > kAnchorMoveEpsilonYaw)
        {
            result.anchor_preservation_check.ok = false;
            result.anchor_preservation_check.reason = "previous_fiducial_moved";
        }
    }
    for (const auto& vertex : graph.vertices)
    {
        if (!vertex.is_anchor_neighborhood)
        {
            continue;
        }
        const auto pose = pose_store.GetWorldPose(vertex.keyframe_id);
        if (!pose)
        {
            result.anchor_preservation_check.ok = false;
            result.anchor_preservation_check.reason =
                "missing_previous_fiducial_neighborhood";
            continue;
        }
        const double delta_t =
            TranslationError(vertex.initial_world_T_kf, pose.value());
        const double delta_yaw =
            std::abs(NormalizeAngle(
                YawFromTransform(pose.value()) -
                YawFromTransform(vertex.initial_world_T_kf)));
        result.anchor_preservation_check.max_previous_fiducial_neighborhood_delta_t =
            std::max(
                result.anchor_preservation_check
                    .max_previous_fiducial_neighborhood_delta_t,
                delta_t);
        result.anchor_preservation_check.max_previous_fiducial_neighborhood_delta_yaw =
            std::max(
                result.anchor_preservation_check
                    .max_previous_fiducial_neighborhood_delta_yaw,
                delta_yaw);
    }
    if (result.anchor_preservation_check.required)
    {
        if (!result.anchor_preservation_check.graph_satisfied ||
            result.anchor_preservation_check.previous_fiducial_fixed_count == 0 ||
            result.anchor_preservation_check.checked_branch_anchors == 0)
        {
            result.anchor_preservation_check.ok = false;
            result.anchor_preservation_check.reason =
                result.anchor_preservation_check.subdivision_candidates > 0
                    ? "unconfirmed_subdivision"
                    : "missing_branch_anchor";
        }
    }
    else
    {
        result.anchor_preservation_check.ok = true;
    }
    result.anchor_preservation_ok = result.anchor_preservation_check.ok;

    const bool hard_invariants_ok =
        result.fixed_ok &&
        result.anchor_preservation_ok &&
        !dry_run.has_nan &&
        !force_reject;
    const bool absurd_fiducial_progress =
        result.error.before_error_t >= config_.post_apply_fiducial_absurd_error_t &&
        result.primary_error_improved &&
        (result.final_error_ok ||
         result.error.improvement_ratio >= config_.dryrun_partial_min_improvement_ratio);
    const bool deformable_only_internal_break =
        absurd_fiducial_progress &&
        result.internal_error.num_edges_broken > 0 &&
        result.internal_error.strong_edges_broken == 0 &&
        result.internal_error.deformable_edges_broken ==
            result.internal_error.num_edges_broken;
    // 1L: las aristas internas y la propagacion amplia de keyframes no
    // vertices son diagnostico de calidad, no condicion de rollback. El
    // commit debe depender de que el fiducial objetivo llegue a su pose,
    // no se muevan vertices hard-fixed ni se rompan anchors/invariantes duras.
    const bool partial_invariants_ok = hard_invariants_ok;
    const bool accept_invariants_ok = hard_invariants_ok;
    const bool partial_safe =
        result.primary_error_improved &&
        (dry_run.partial_candidate || absurd_fiducial_progress) &&
        (result.final_error_ok ||
         result.error.improvement_ratio >=
             config_.dryrun_partial_min_improvement_ratio) &&
        partial_invariants_ok;

    if (dry_run.useful &&
        result.primary_error_improved &&
        result.final_error_ok &&
        accept_invariants_ok)
    {
        result.decision = PostApplyDecision::Accept;
        result.reason = "real_error_reduced_and_confirmed";
    }
    else if (dry_run.partial_candidate &&
             result.final_error_ok &&
             accept_invariants_ok)
    {
        result.decision = PostApplyDecision::Accept;
        result.reason = "partial_entered_accept_threshold";
    }
    else if (partial_safe)
    {
        result.decision = PostApplyDecision::PartialKeepForNextPass;
        result.needs_second_pass = true;
        if (absurd_fiducial_progress && deformable_only_internal_break)
        {
            result.reason = "absurd_error_reduced_with_internal_deformation";
        }
        else
        {
            result.reason = dry_run.partial_reason.empty()
                                ? "partial_real_improvement_safe"
                                : dry_run.partial_reason;
        }
    }
    else
    {
        result.decision = PostApplyDecision::RejectRollback;
        if (force_reject)
        {
            result.reason = "debug_forced_reject";
        }
        else if (!result.primary_error_improved)
        {
            result.reason = "real_error_not_reduced";
        }
        else if (!result.fixed_ok)
        {
            result.reason = "fixed_keyframe_moved";
        }
        else if (!result.anchor_preservation_ok)
        {
            result.reason = "anchor_preservation_failed";
        }
        else if (!result.internal_edges_ok &&
                 result.internal_error.strong_edges_broken > 0)
        {
            result.reason = "internal_edges_broken";
        }
        else if (!result.internal_edges_ok)
        {
            result.reason = "deformable_edges_broken_without_safe_partial";
        }
        else if (!result.propagation_ok)
        {
            result.reason = "propagation_discontinuity";
        }
        else
        {
            result.reason = "post_apply_threshold_not_met";
        }
    }

    result.success = true;
    result.retry = BuildRetrySuggestion(result);
    if (result.decision == PostApplyDecision::Accept)
    {
        result.retry.retry_allowed = false;
        result.retry.strategy = "NONE";
        result.retry.reason = "accepted";
    }
    return result;
}

const char* ToString(PostApplyDecision decision)
{
    switch (decision)
    {
        case PostApplyDecision::RejectRollback:
            return "REJECT_ROLLBACK";
        case PostApplyDecision::Accept:
            return "ACCEPT";
        case PostApplyDecision::PartialKeepForNextPass:
            return "PARTIAL_KEEP_FOR_NEXT_PASS";
    }
    return "UNKNOWN";
}

}  // namespace orbslam3_multi

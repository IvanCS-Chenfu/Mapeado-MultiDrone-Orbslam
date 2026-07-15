#include "orbslam3_multi/fiducial_anchor_manager.hpp"

#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose.hpp>

#include <cmath>
#include <algorithm>

namespace orbslam3_multi
{
namespace
{

bool IsFiniteTransform(const Eigen::Matrix4d& transform)
{
    return transform.allFinite();
}

bool PoseMsgToMatrix(const geometry_msgs::msg::Pose& pose, Eigen::Matrix4d& out)
{
    const Eigen::Quaterniond q_raw(
        pose.orientation.w,
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z);

    if (!std::isfinite(q_raw.w()) ||
        !std::isfinite(q_raw.x()) ||
        !std::isfinite(q_raw.y()) ||
        !std::isfinite(q_raw.z()) ||
        q_raw.norm() < 1e-9)
    {
        return false;
    }

    const Eigen::Quaterniond q = q_raw.normalized();
    out = Eigen::Matrix4d::Identity();
    out.block<3, 3>(0, 0) = q.toRotationMatrix();
    out(0, 3) = pose.position.x;
    out(1, 3) = pose.position.y;
    out(2, 3) = pose.position.z;
    return out.allFinite();
}

RawSubmapId SubmapOf(const FiducialObservation& observation)
{
    return RawSubmapId{observation.drone_id, observation.map_epoch};
}

RawKeyFrameId KeyFrameOf(const FiducialObservation& observation)
{
    return RawKeyFrameId{
        observation.drone_id,
        observation.map_epoch,
        observation.local_keyframe_id};
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

double YawFromTransform(const Eigen::Matrix4d& transform)
{
    return std::atan2(transform(1, 0), transform(0, 0));
}

double RotationErrorRad(const Eigen::Matrix4d& estimated_world_T_kf,
                        const Eigen::Matrix4d& target_world_T_kf)
{
    const Eigen::Matrix3d delta_R =
        estimated_world_T_kf.block<3, 3>(0, 0).transpose() *
        target_world_T_kf.block<3, 3>(0, 0);
    Eigen::AngleAxisd angle_axis(delta_R);
    return std::abs(NormalizeAngle(angle_axis.angle()));
}

double TranslationErrorM(const Eigen::Matrix4d& estimated_world_T_kf,
                         const Eigen::Matrix4d& target_world_T_kf)
{
    return (estimated_world_T_kf.block<3, 1>(0, 3) -
            target_world_T_kf.block<3, 1>(0, 3)).norm();
}

}  // namespace

void FiducialAnchorManager::ConfigureRevisitThresholds(
    const FiducialRevisitConfig& config)
{
    // F1H: los umbrales son configuracion de diagnostico/tarea, no una forma
    // de mover el mapa. Se acotan a valores no negativos para evitar decisiones
    // imposibles si el launch trae parametros erroneos.
    revisit_config_ = config;
    revisit_config_.error_threshold_m = std::max(0.0, revisit_config_.error_threshold_m);
    revisit_config_.yaw_threshold_rad = std::max(0.0, revisit_config_.yaw_threshold_rad);
    revisit_config_.rot_threshold_rad = std::max(0.0, revisit_config_.rot_threshold_rad);
}

FiducialAnchorResult FiducialAnchorManager::RegisterFiducialObservation(
    const FiducialObservation& observation,
    const RawMapDatabase& raw_db,
    GlobalPoseStore& pose_store)
{
    // F1E: esta funcion es el unico punto backend que convierte una observacion
    // absoluta fiducial en anchor. No toca poses locales raw y no crea loops.
    FiducialAnchorResult result;
    result.submap_id = SubmapOf(observation);
    result.keyframe_id = KeyFrameOf(observation);

    ++stats_.observations;
    if (observation.source == "REPLAY_RECORDED_FIDUCIAL")
    {
        ++stats_.replay_observations;
    }

    const auto* keyframe = raw_db.GetKeyFrame(result.keyframe_id);
    if (!keyframe)
    {
        ++stats_.rejected;
        result.reason = "raw_keyframe_missing";
        return result;
    }
    if (!IsFiniteTransform(observation.world_T_body_fiducial))
    {
        ++stats_.rejected;
        result.reason = "world_T_body_fiducial_invalid";
        return result;
    }

    Eigen::Matrix4d local_T_kf = Eigen::Matrix4d::Identity();
    if (!PoseMsgToMatrix(keyframe->pose, local_T_kf))
    {
        ++stats_.rejected;
        result.reason = "local_T_kf_invalid";
        return result;
    }

    result.observation_accepted = true;
    ++stats_.accepted;
    result.submap_already_anchored = pose_store.HasSubmapAnchor(result.submap_id);
    result.world_T_camera_fiducial =
        pose_store.TransformBodyPoseToCameraPose(observation.world_T_body_fiducial);
    if (!IsFiniteTransform(result.world_T_camera_fiducial))
    {
        ++stats_.rejected;
        --stats_.accepted;
        result.observation_accepted = false;
        result.reason = "world_T_camera_fiducial_invalid";
        return result;
    }

    result.world_T_local = result.world_T_camera_fiducial * local_T_kf.inverse();
    if (!IsFiniteTransform(result.world_T_local))
    {
        ++stats_.rejected;
        --stats_.accepted;
        result.observation_accepted = false;
        result.reason = "world_T_local_invalid";
        return result;
    }
    result.threshold_t_m = revisit_config_.error_threshold_m;
    result.threshold_yaw_rad = revisit_config_.yaw_threshold_rad;
    result.threshold_rot_rad = revisit_config_.rot_threshold_rad;

    if (!result.submap_already_anchored)
    {
        const auto anchor_result =
            pose_store.AnchorSubmap(
                result.submap_id,
                result.world_T_local,
                raw_db,
                observation.source);
        if (!anchor_result.success)
        {
            ++stats_.rejected;
            --stats_.accepted;
            result.observation_accepted = false;
            result.reason = anchor_result.reason;
            return result;
        }

        result.first_anchor_created = true;
        ++stats_.anchors_created;
    }
    else
    {
        // F1H: una segunda observacion fiducial de un submapa ya anclado no
        // recalcula `world_T_local`. Primero garantizamos que el KF tenga pose
        // world derivada del anchor existente y despues medimos el residual
        // absoluto contra la nueva observacion.
        result.revisit = true;
        ++stats_.revisit_observations;
        if (!pose_store.HasWorldPose(result.keyframe_id))
        {
            const auto new_kf_result =
                pose_store.RegisterNewKeyFrameIfAnchored(
                    result.keyframe_id,
                    raw_db,
                    observation.source);
            if (new_kf_result.status != GlobalPoseNewKeyFrameStatus::PoseSet &&
                new_kf_result.status != GlobalPoseNewKeyFrameStatus::AlreadyHadPose &&
                new_kf_result.status != GlobalPoseNewKeyFrameStatus::CorrectionInherited)
            {
                ++stats_.rejected;
                --stats_.accepted;
                ++stats_.revisit_no_pose;
                result.observation_accepted = false;
                result.reason = new_kf_result.reason;
                return result;
            }
        }

        const auto estimated_world_T_kf = pose_store.GetWorldPose(result.keyframe_id);
        if (!estimated_world_T_kf || !IsFiniteTransform(*estimated_world_T_kf))
        {
            ++stats_.rejected;
            --stats_.accepted;
            ++stats_.revisit_no_pose;
            result.observation_accepted = false;
            result.reason = "estimated_world_pose_missing";
            return result;
        }

        result.estimated_world_T_kf = *estimated_world_T_kf;
        result.error_t_m =
            TranslationErrorM(result.estimated_world_T_kf, result.world_T_camera_fiducial);
        result.error_rot_rad =
            RotationErrorRad(result.estimated_world_T_kf, result.world_T_camera_fiducial);
        result.error_yaw_rad = std::abs(NormalizeAngle(
            YawFromTransform(result.estimated_world_T_kf) -
            YawFromTransform(result.world_T_camera_fiducial)));
        result.pose_error_valid =
            std::isfinite(result.error_t_m) &&
            std::isfinite(result.error_rot_rad) &&
            std::isfinite(result.error_yaw_rad);
        if (!result.pose_error_valid)
        {
            ++stats_.rejected;
            --stats_.accepted;
            result.observation_accepted = false;
            result.reason = "pose_error_invalid";
            return result;
        }

        result.revisit_ok =
            result.error_t_m <= revisit_config_.error_threshold_m &&
            result.error_rot_rad <= revisit_config_.rot_threshold_rad &&
            result.error_yaw_rad <= revisit_config_.yaw_threshold_rad;
        if (result.revisit_ok)
        {
            ++stats_.revisit_ok;
            result.reason = "fiducial_revisit_ok";
        }
        else
        {
            ++stats_.revisit_high_error;
            const TaskGroupKey task_key{
                result.submap_id,
                observation.fiducial_id};
            const auto duplicate_it = task_ids_by_key_.find(task_key);
            if (duplicate_it != task_ids_by_key_.end())
            {
                const auto existing_it = std::find_if(
                    pending_tasks_.begin(),
                    pending_tasks_.end(),
                    [task_id = duplicate_it->second](const FiducialOptimizationTask& task)
                    {
                        return task.task_id == task_id;
                    });
                if (existing_it != pending_tasks_.end())
                {
                    existing_it->keyframe_id = result.keyframe_id;
                    existing_it->estimated_world_T_kf = result.estimated_world_T_kf;
                    existing_it->target_world_T_kf = result.world_T_camera_fiducial;
                    existing_it->error_t_m = result.error_t_m;
                    existing_it->error_rot_rad = result.error_rot_rad;
                    existing_it->error_yaw_rad = result.error_yaw_rad;
                    existing_it->latest_arrival_id = observation.arrival_id;
                    existing_it->latest_source = observation.source;
                    existing_it->update_count += 1;
                    existing_it->status = "PENDING";
                    existing_it->source = observation.source;
                    existing_it->arrival_id = observation.arrival_id;
                    result.duplicate_task = true;
                    result.task_id = existing_it->task_id;
                    ++stats_.task_duplicates;
                    result.reason = "fiducial_task_updated";
                }
                else
                {
                    result.duplicate_task = true;
                    result.task_id = duplicate_it->second;
                    ++stats_.task_duplicates;
                    result.reason = "fiducial_task_duplicate";
                }
            }
            else
            {
                FiducialOptimizationTask task;
                task.task_id = next_task_id_++;
                task.task_generation = 1;
                task.keyframe_id = result.keyframe_id;
                task.submap_id = result.submap_id;
                task.drone_id = observation.drone_id;
                task.map_epoch = observation.map_epoch;
                task.fiducial_id = observation.fiducial_id;
                task.estimated_world_T_kf = result.estimated_world_T_kf;
                task.target_world_T_kf = result.world_T_camera_fiducial;
                task.error_t_m = result.error_t_m;
                task.error_rot_rad = result.error_rot_rad;
                task.error_yaw_rad = result.error_yaw_rad;
                task.threshold_t_m = revisit_config_.error_threshold_m;
                task.threshold_rot_rad = revisit_config_.rot_threshold_rad;
                task.threshold_yaw_rad = revisit_config_.yaw_threshold_rad;
                task.created_arrival_id = observation.arrival_id;
                task.latest_arrival_id = observation.arrival_id;
                task.status = "PENDING";
                task.created_source = observation.source;
                task.latest_source = observation.source;
                task.arrival_id = observation.arrival_id;
                task.source = observation.source;
                pending_tasks_.push_back(task);
                task_ids_by_key_[task_key] = task.task_id;
                result.task_created = true;
                result.task_id = task.task_id;
                ++stats_.tasks_created;
                result.reason = "fiducial_task_created";
            }
        }
    }

    if (result.first_anchor_created || result.revisit_ok)
    {
        const bool already_hard = pose_store.IsHardFiducialKeyFrame(result.keyframe_id);
        result.keyframe_marked_hard_fiducial =
            pose_store.MarkHardFiducialKeyFrame(result.keyframe_id, observation.source);
        if (result.keyframe_marked_hard_fiducial && !already_hard)
        {
            hard_fiducial_keyframes_.insert(result.keyframe_id);
            stats_.hard_fiducial_keyframes = hard_fiducial_keyframes_.size();
        }
    }

    if (result.first_anchor_created)
    {
        result.reason = "first_anchor_created";
    }
    else if (!result.revisit)
    {
        result.reason = "observation_registered";
    }
    return result;
}

FiducialAnchorStats FiducialAnchorManager::GetStats() const
{
    return stats_;
}

std::vector<FiducialOptimizationTask>
FiducialAnchorManager::GetPendingFiducialOptimizationTasks() const
{
    return pending_tasks_;
}

uint64_t FiducialAnchorManager::PendingFiducialOptimizationTaskCount() const
{
    return pending_tasks_.size();
}

}  // namespace orbslam3_multi

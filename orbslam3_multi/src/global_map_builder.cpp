#include "orbslam3_multi/global_map_builder.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose.hpp>

#include <algorithm>
#include <cmath>
#include <optional>

namespace orbslam3_multi
{
namespace
{

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

bool GetRawLocalKeyFramePose(
    const RawMapDatabase& raw_db,
    const RawKeyFrameId& keyframe_id,
    Eigen::Matrix4d& local_T_kf)
{
    const auto* keyframe = raw_db.GetKeyFrame(keyframe_id);
    if (!keyframe || keyframe->is_bad)
    {
        return false;
    }
    return PoseMsgToMatrix(keyframe->pose, local_T_kf);
}

std::optional<RawKeyFrameId> SelectMapPointPublishingKeyFrame(
    const RawSubmapId& submap_id,
    const RawMapDatabase& raw_db,
    const GlobalPoseStore& pose_store,
    const orbslam3_msgs::msg::OrbMapPoint& mappoint)
{
    auto usable_keyframe = [&](uint64_t local_kf_id) -> std::optional<RawKeyFrameId>
    {
        const RawKeyFrameId keyframe_id{
            submap_id.drone_id,
            submap_id.map_epoch,
            local_kf_id};
        if (!raw_db.HasKeyFrame(keyframe_id) || !pose_store.GetWorldPose(keyframe_id))
        {
            return std::nullopt;
        }
        Eigen::Matrix4d raw_local_T_kf = Eigen::Matrix4d::Identity();
        if (!GetRawLocalKeyFramePose(raw_db, keyframe_id, raw_local_T_kf))
        {
            return std::nullopt;
        }
        return keyframe_id;
    };

    if (const auto reference = usable_keyframe(mappoint.reference_keyframe_id))
    {
        return reference;
    }

    // 1K: al publicar puntos tras una optimizacion del servidor preferimos el
    // KF de referencia de ORB-SLAM3 y, si no existe pose global valida, caemos
    // al primer observador util. El MapPoint raw no se modifica.
    for (const auto& observation : mappoint.observations)
    {
        if (const auto observed = usable_keyframe(observation.keyframe_id))
        {
            return observed;
        }
    }

    return std::nullopt;
}

}  // namespace

GlobalMapBuildResult GlobalMapBuilder::Build(
    const RawMapDatabase& raw_db,
    const GlobalPoseStore& pose_store,
    const LandmarkScoreManager& score_manager,
    float min_score_to_publish) const
{
    // F1F: se recorren submapas completos porque la decisión clave es si el
    // submapa está anclado. Submapas sin anchor se saltan enteros para evitar
    // publicar puntos sin frame global válido.
    GlobalMapBuildResult result;
    min_score_to_publish = std::max(0.0F, min_score_to_publish);

    const auto submap_ids = raw_db.GetSubmapIds();
    result.stats.total_submaps = submap_ids.size();

    double score_sum = 0.0;
    bool first_score = true;

    for (const auto& submap_id : submap_ids)
    {
        const auto world_T_local = pose_store.GetSubmapWorldTransform(submap_id);
        if (!world_T_local)
        {
            ++result.stats.skipped_unanchored_submaps;
            continue;
        }
        ++result.stats.anchored_submaps;

        const auto mappoint_ids = raw_db.GetMapPointIdsForSubmap(submap_id);
        result.stats.raw_points += mappoint_ids.size();

        for (const auto& mappoint_id : mappoint_ids)
        {
            const auto* mappoint = raw_db.GetMapPoint(mappoint_id);
            if (!mappoint)
            {
                continue;
            }
            if (mappoint->is_bad)
            {
                ++result.stats.bad_skipped;
                continue;
            }

            const float score = score_manager.GetScoreOrDefault(mappoint_id);
            if (score < min_score_to_publish)
            {
                ++result.stats.below_score_skipped;
                continue;
            }

            const Eigen::Vector4d local_point(
                mappoint->position.x,
                mappoint->position.y,
                mappoint->position.z,
                1.0);
            if (!local_point.allFinite())
            {
                ++result.stats.invalid_pose_skipped;
                continue;
            }

            Eigen::Vector4d world_point = world_T_local.value() * local_point;

            const auto publishing_keyframe =
                SelectMapPointPublishingKeyFrame(
                    submap_id,
                    raw_db,
                    pose_store,
                    *mappoint);
            if (publishing_keyframe)
            {
                Eigen::Matrix4d raw_local_T_kf = Eigen::Matrix4d::Identity();
                const auto final_world_T_kf =
                    pose_store.GetWorldPose(publishing_keyframe.value());
                if (!final_world_T_kf ||
                    !GetRawLocalKeyFramePose(
                        raw_db,
                        publishing_keyframe.value(),
                        raw_local_T_kf))
                {
                    ++result.stats.invalid_pose_skipped;
                    continue;
                }

                const Eigen::Vector4d kf_point =
                    raw_local_T_kf.inverse() * local_point;
                world_point = final_world_T_kf.value() * kf_point;
                ++result.stats.keyframe_projected_points;
            }
            else
            {
                // 1K: si un submapa ya tiene correccion del servidor, no
                // podemos publicar un MapPoint sin KF de referencia usando
                // solo `world_T_local`; quedaria como un punto suelto en la
                // posicion preoptimizacion. En submapas sin correccion se
                // conserva el fallback legacy.
                if (pose_store.GetSubmapLastServerCorrection(submap_id))
                {
                    ++result.stats.invalid_pose_skipped;
                    continue;
                }
                ++result.stats.fallback_submap_points;
            }
            if (!world_point.allFinite())
            {
                ++result.stats.invalid_pose_skipped;
                continue;
            }
            if (publishing_keyframe &&
                pose_store.GetKeyFrameServerCorrection(publishing_keyframe.value()))
            {
                ++result.stats.server_corrected_points;
            }

            ++result.stats.candidate_points;

            GlobalSparsePoint point;
            point.global_mappoint_id = MakeGlobalMapPointId(mappoint_id);
            point.drone_id = mappoint_id.drone_id;
            point.map_epoch = mappoint_id.map_epoch;
            point.local_mappoint_id = mappoint_id.local_mp_id;
            point.x = world_point.x();
            point.y = world_point.y();
            point.z = world_point.z();
            point.score = score;
            point.observations = mappoint->observations_count;
            point.from_anchored_submap = true;
            point.is_fused = false;
            result.points.push_back(point);

            if (first_score)
            {
                result.stats.score_min = score;
                result.stats.score_max = score;
                first_score = false;
            }
            else
            {
                result.stats.score_min = std::min(result.stats.score_min, score);
                result.stats.score_max = std::max(result.stats.score_max, score);
            }
            score_sum += static_cast<double>(score);
        }
    }

    result.stats.returned_points = result.points.size();
    if (!result.points.empty())
    {
        result.stats.score_mean =
            static_cast<float>(score_sum / static_cast<double>(result.points.size()));
    }
    return result;
}

uint64_t GlobalMapBuilder::MakeGlobalMapPointId(const RawMapPointId& id) const
{
    // F1F: ID compacto para PointCloud/logs. La identidad canonica sigue siendo
    // `(drone_id, map_epoch, local_mp_id)`.
    return static_cast<uint64_t>(id.drone_id) * 1000000000000ULL +
           id.map_epoch * 1000000ULL +
           id.local_mp_id;
}

}  // namespace orbslam3_multi

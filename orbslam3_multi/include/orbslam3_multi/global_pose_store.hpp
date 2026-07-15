#pragma once

#include "orbslam3_multi/raw_map_database.hpp"
#include "orbslam3_multi/raw_map_types.hpp"

#include <Eigen/Core>

#include <cstdint>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <vector>

namespace orbslam3_multi
{

struct GlobalPoseStoreStats
{
    uint64_t anchored_submaps = 0;
    uint64_t keyframe_world_poses = 0;
    uint64_t optimized_keyframes = 0;
    uint64_t propagated_keyframes = 0;
    uint64_t rebased_keyframes = 0;
    uint64_t submap_corrections = 0;
    uint64_t hard_fiducial_keyframes = 0;
};

struct GlobalSubmapPoseStats
{
    RawSubmapId submap_id;
    bool has_anchor = false;
    bool has_last_correction = false;
    uint64_t keyframe_world_poses = 0;
    std::string anchor_source;
    std::string correction_source;
};

struct GlobalPoseAnchorResult
{
    bool success = false;
    bool replaced_existing_anchor = false;
    uint64_t anchored_keyframes = 0;
    std::string reason;
};

enum class GlobalPoseNewKeyFrameStatus : uint8_t
{
    MissingRawKeyFrame = 0,
    UnanchoredSubmap = 1,
    AlreadyHadPose = 2,
    PoseSet = 3,
    CorrectionInherited = 4,
};

struct GlobalPoseNewKeyFrameResult
{
    GlobalPoseNewKeyFrameStatus status = GlobalPoseNewKeyFrameStatus::MissingRawKeyFrame;
    RawKeyFrameId keyframe_id;
    std::string source;
    std::string reason;
};

struct GlobalPoseOptimizedResult
{
    bool success = false;
    RawSubmapId submap_id;
    Eigen::Matrix4d correction_T_latest = Eigen::Matrix4d::Identity();
    std::string reason;
};

struct PoseStoreReconcileEvent
{
    RawKeyFrameId keyframe_id;
    std::string action;
    std::string reason;
    double raw_delta_translation_m = 0.0;
    double raw_delta_yaw_rad = 0.0;
};

struct PoseStoreReconcileResult
{
    uint64_t raw_pose_changed = 0;
    uint64_t anchor_rebased = 0;
    uint64_t optimized_kept = 0;
    uint64_t no_world_pose = 0;
    uint64_t failed = 0;
    std::set<RawSubmapId> optimized_submaps_affected;
    std::vector<PoseStoreReconcileEvent> events;
};

struct BodyCameraTransformConfig
{
    double x = 0.10;
    double y = 0.03;
    double z = 0.03;
    double roll_deg = 0.0;
    double pitch_deg = -90.0;
    double yaw_deg = 90.0;
    bool use_camera_optical_frame_convention = true;
    std::string source = "default";
};

struct ApplyBackupKeyFrameState
{
    RawKeyFrameId keyframe_id;
    bool had_world_pose = false;
    Eigen::Matrix4d world_T_kf = Eigen::Matrix4d::Identity();
    std::string world_source;
    bool world_optimized = false;
    bool world_propagated = false;
    bool world_rebased = false;

    bool had_optimized_pose = false;
    Eigen::Matrix4d optimized_world_T_kf = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d correction_T_kf = Eigen::Matrix4d::Identity();
    std::string optimized_source;
    bool optimized_propagated = false;
    bool optimized_rebased = false;

    bool was_hard_fiducial = false;
    std::string hard_fiducial_source;
};

struct ApplyBackupSubmapCorrectionState
{
    RawSubmapId submap_id;
    bool had_correction = false;
    Eigen::Matrix4d correction_T_latest = Eigen::Matrix4d::Identity();
    std::string source;
    bool had_from_keyframe = false;
    RawKeyFrameId from_keyframe;
};

struct ApplyBackupResult
{
    bool success = false;
    uint64_t task_id = 0;
    uint64_t affected_keyframes = 0;
    uint64_t affected_submaps = 0;
    std::string reason;
};

struct RollbackResult
{
    bool success = false;
    uint64_t task_id = 0;
    uint64_t restored_world_poses = 0;
    uint64_t removed_world_poses = 0;
    uint64_t restored_optimized_poses = 0;
    uint64_t removed_optimized_poses = 0;
    uint64_t restored_submap_corrections = 0;
    uint64_t removed_submap_corrections = 0;
    std::string reason;
};

// F1D: almacena estado global ligero de poses sin duplicar datos ORB grandes.
// Entradas relevantes: IDs raw `(drone_id, map_epoch, local_kf_id)`, anchors
// `world_T_local`, y poses optimizadas ya calculadas por pruebas o fases futuras.
// Salida/efecto: consultas de pose world y correcciones heredables; nunca muta
// RawMapDatabase ni inventa pose world para submapas sin anchor.
class GlobalPoseStore
{
public:
    GlobalPoseStore();

    void ConfigureBodyCameraTransform(const BodyCameraTransformConfig& config);
    Eigen::Matrix4d GetBodyCameraTransform() const;
    Eigen::Matrix4d TransformBodyPoseToCameraPose(
        const Eigen::Matrix4d& world_T_body) const;
    BodyCameraTransformConfig GetBodyCameraTransformConfig() const;

    GlobalPoseAnchorResult AnchorSubmap(const RawSubmapId& submap_id,
                                        const Eigen::Matrix4d& world_T_local,
                                        const RawMapDatabase& raw_db,
                                        const std::string& source);

    bool HasSubmapAnchor(const RawSubmapId& submap_id) const;
    std::optional<Eigen::Matrix4d> GetSubmapWorldTransform(
        const RawSubmapId& submap_id) const;
    bool HasWorldPose(const RawKeyFrameId& keyframe_id) const;
    std::optional<Eigen::Matrix4d> GetWorldPose(const RawKeyFrameId& keyframe_id) const;

    bool MarkHardFiducialKeyFrame(const RawKeyFrameId& keyframe_id,
                                  const std::string& source);
    bool IsHardFiducialKeyFrame(const RawKeyFrameId& keyframe_id) const;

    GlobalPoseNewKeyFrameResult RegisterNewKeyFrameIfAnchored(
        const RawKeyFrameId& keyframe_id,
        const RawMapDatabase& raw_db,
        const std::string& source);

    GlobalPoseOptimizedResult SetOptimizedKeyFramePose(
        const RawKeyFrameId& keyframe_id,
        const Eigen::Matrix4d& optimized_world_T_kf,
        const RawMapDatabase& raw_db,
        const std::string& source);

    GlobalPoseOptimizedResult SetPropagatedKeyFramePose(
        const RawKeyFrameId& keyframe_id,
        const Eigen::Matrix4d& propagated_world_T_kf,
        const RawMapDatabase& raw_db,
        const std::string& source);

    std::optional<Eigen::Matrix4d> GetKeyFrameServerCorrection(
        const RawKeyFrameId& keyframe_id) const;
    std::optional<Eigen::Matrix4d> GetSubmapLastServerCorrection(
        const RawSubmapId& submap_id) const;
    GlobalPoseOptimizedResult SetSubmapLastServerCorrectionFromKeyFrame(
        const RawKeyFrameId& keyframe_id,
        const std::string& source);

    // F1L: captura y restaura el estado tocado por un apply. El backup vive
    // dentro de GlobalPoseStore porque ahi estan las fuentes, flags y
    // correcciones heredables que deben volver exactamente a su valor previo.
    ApplyBackupResult CreateApplyBackup(
        uint64_t task_id,
        const std::vector<RawKeyFrameId>& affected_keyframes);
    RollbackResult RestoreApplyBackup(uint64_t task_id);
    bool ConfirmApply(uint64_t task_id);

    PoseStoreReconcileResult ReconcileAfterRawIngestResult(
        const RawInsertResult& ingest_result,
        const RawMapDatabase& raw_db,
        const std::string& source);

    GlobalPoseStoreStats GetPoseStoreStats() const;
    GlobalSubmapPoseStats GetSubmapPoseStats(const RawSubmapId& submap_id) const;

private:
    struct SubmapAnchor
    {
        Eigen::Matrix4d world_T_local = Eigen::Matrix4d::Identity();
        std::string source;
    };

    struct KeyFrameWorldPose
    {
        Eigen::Matrix4d world_T_kf = Eigen::Matrix4d::Identity();
        std::string source;
        bool optimized = false;
        bool propagated = false;
        bool rebased = false;
    };

    struct OptimizedKeyFramePose
    {
        Eigen::Matrix4d optimized_world_T_kf = Eigen::Matrix4d::Identity();
        Eigen::Matrix4d correction_T_kf = Eigen::Matrix4d::Identity();
        std::string source;
        bool propagated = false;
        bool rebased = false;
    };

    GlobalPoseOptimizedResult SetServerControlledKeyFramePose(
        const RawKeyFrameId& keyframe_id,
        const Eigen::Matrix4d& world_T_kf,
        const RawMapDatabase& raw_db,
        const std::string& source,
        bool propagated,
        bool rebased);

    struct SubmapCorrection
    {
        Eigen::Matrix4d correction_T_latest = Eigen::Matrix4d::Identity();
        std::string source;
        bool has_from_keyframe = false;
        RawKeyFrameId from_keyframe;
    };

    struct ApplyBackup
    {
        uint64_t task_id = 0;
        std::vector<ApplyBackupKeyFrameState> keyframes;
        std::vector<ApplyBackupSubmapCorrectionState> submap_corrections;
    };

    bool ComputeRawWorldPose(const RawKeyFrameId& keyframe_id,
                             const RawMapDatabase& raw_db,
                             Eigen::Matrix4d& raw_world_T_kf,
                             std::string* reason = nullptr) const;

    std::map<RawSubmapId, SubmapAnchor> submap_anchors_;
    std::map<RawKeyFrameId, KeyFrameWorldPose> keyframe_world_poses_;
    std::map<RawKeyFrameId, OptimizedKeyFramePose> optimized_keyframes_;
    std::map<RawSubmapId, SubmapCorrection> submap_last_corrections_;
    std::map<RawKeyFrameId, std::string> hard_fiducial_keyframes_;
    std::map<uint64_t, ApplyBackup> pending_apply_backups_;
    BodyCameraTransformConfig body_camera_config_;
    Eigen::Matrix4d body_T_camera_ = Eigen::Matrix4d::Identity();
};

const char* ToString(GlobalPoseNewKeyFrameStatus status);

}  // namespace orbslam3_multi

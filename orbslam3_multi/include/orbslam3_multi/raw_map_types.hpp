#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace orbslam3_multi
{

// F1C: identidad raw de submapa. Siempre combina `drone_id` y `map_epoch`
// para no mezclar mapas locales distintos producidos por el mismo dron.
struct RawSubmapId
{
    uint32_t drone_id = 0;
    uint64_t map_epoch = 0;

    bool operator<(const RawSubmapId& other) const
    {
        if (drone_id != other.drone_id)
        {
            return drone_id < other.drone_id;
        }
        return map_epoch < other.map_epoch;
    }

    bool operator==(const RawSubmapId& other) const
    {
        return drone_id == other.drone_id && map_epoch == other.map_epoch;
    }
};

// F1C: identidad logica de un KeyFrame raw. El `local_kf_id` nunca es global
// por si solo; pertenece siempre al submapa `(drone_id, map_epoch)`.
struct RawKeyFrameId
{
    uint32_t drone_id = 0;
    uint64_t map_epoch = 0;
    uint64_t local_kf_id = 0;

    bool operator<(const RawKeyFrameId& other) const
    {
        if (drone_id != other.drone_id)
        {
            return drone_id < other.drone_id;
        }
        if (map_epoch != other.map_epoch)
        {
            return map_epoch < other.map_epoch;
        }
        return local_kf_id < other.local_kf_id;
    }

    bool operator==(const RawKeyFrameId& other) const
    {
        return drone_id == other.drone_id &&
               map_epoch == other.map_epoch &&
               local_kf_id == other.local_kf_id;
    }
};

// F1C: identidad logica de un MapPoint raw. Se guarda separada de cualquier
// futuro fused landmark para que optimizacion/fusion no sobrescriban raw.
struct RawMapPointId
{
    uint32_t drone_id = 0;
    uint64_t map_epoch = 0;
    uint64_t local_mp_id = 0;

    bool operator<(const RawMapPointId& other) const
    {
        if (drone_id != other.drone_id)
        {
            return drone_id < other.drone_id;
        }
        if (map_epoch != other.map_epoch)
        {
            return map_epoch < other.map_epoch;
        }
        return local_mp_id < other.local_mp_id;
    }

    bool operator==(const RawMapPointId& other) const
    {
        return drone_id == other.drone_id &&
               map_epoch == other.map_epoch &&
               local_mp_id == other.local_mp_id;
    }
};

enum class RawJournalEntryKind : uint8_t
{
    Delta = 0,
    FullSnapshot = 1,
};

struct RawDatabaseStats
{
    uint64_t journal_entries = 0;
    uint64_t delta_entries = 0;
    uint64_t full_snapshot_entries = 0;
    uint64_t fiducial_observations = 0;
    uint64_t submaps = 0;
    uint64_t keyframes = 0;
    uint64_t mappoints = 0;
    uint64_t last_arrival_id = 0;
};

struct RawPoseChange
{
    RawKeyFrameId keyframe_id;
    double delta_translation_m = 0.0;
    double delta_yaw_rad = 0.0;
    bool large_change = false;
};

struct RawInsertResult
{
    bool new_submap = false;
    RawSubmapId submap_id;
    RawJournalEntryKind kind = RawJournalEntryKind::Delta;
    uint64_t arrival_id = 0;
    uint64_t new_keyframes = 0;
    uint64_t updated_keyframes = 0;
    uint64_t new_mappoints = 0;
    uint64_t updated_mappoints = 0;
    uint64_t bad_keyframes = 0;
    uint64_t bad_mappoints = 0;
    uint64_t large_raw_pose_changed_keyframes = 0;
    std::vector<RawPoseChange> raw_pose_changed_keyframes;
    RawDatabaseStats stats;
};

// F1E: observacion fiducial persistida junto al journal raw. No sustituye a
// los datos ORB-SLAM3 ni usa GT durante replay; solo conserva la medicion
// absoluta simulada que ya fue aceptada en vivo y asociada a un `arrival_id`.
struct RecordedFiducialObservation
{
    uint64_t arrival_id = 0;
    uint32_t drone_id = 0;
    uint64_t map_epoch = 0;
    uint64_t local_keyframe_id = 0;
    uint64_t global_keyframe_id = 0;
    int32_t fiducial_id = 0;
    double world_x = 0.0;
    double world_y = 0.0;
    double world_z = 0.0;
    double world_qx = 0.0;
    double world_qy = 0.0;
    double world_qz = 0.0;
    double world_qw = 1.0;
    double keyframe_stamp_sec = 0.0;
    double gt_stamp_sec = 0.0;
    double association_dt_sec = 0.0;
    double distance_to_fiducial_m = 0.0;
    std::string source;
};

std::string ToString(const RawSubmapId& id);
const char* ToString(RawJournalEntryKind kind);

}  // namespace orbslam3_multi

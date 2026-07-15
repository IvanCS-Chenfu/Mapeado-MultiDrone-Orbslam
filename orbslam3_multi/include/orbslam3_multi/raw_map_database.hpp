#pragma once

#include "orbslam3_multi/raw_map_types.hpp"
#include "orbslam3_msgs/msg/orb_key_frame.hpp"
#include "orbslam3_msgs/msg/orb_map.hpp"
#include "orbslam3_msgs/msg/orb_map_point.hpp"

#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace orbslam3_multi
{

// F1C: entrada de journal raw. Guarda el `OrbMap` completo recibido para poder
// reproducir el orden de llegada real mediante `arrival_id`.
struct RawJournalEntry
{
    uint64_t arrival_id = 0;
    RawJournalEntryKind kind = RawJournalEntryKind::Delta;
    orbslam3_msgs::msg::OrbMap map;
};

// F1C: estado raw actual de un submapa. Es deliberadamente local y no contiene
// poses globales, anchors, loops globales ni datos fusionados.
struct RawSubmap
{
    RawSubmapId id;
    std::string drone_name;
    std::string map_frame;
    uint64_t last_map_sequence = 0;
    uint64_t last_arrival_id = 0;
    uint64_t delta_count = 0;
    uint64_t full_snapshot_count = 0;
    float fx = 0.0F;
    float fy = 0.0F;
    float cx = 0.0F;
    float cy = 0.0F;
    float bf = 0.0F;
    uint32_t image_width = 0;
    uint32_t image_height = 0;
    std::map<uint64_t, orbslam3_msgs::msg::OrbKeyFrame> keyframes;
    std::map<uint64_t, orbslam3_msgs::msg::OrbMapPoint> mappoints;
};

// F1C: base de datos raw pura. Su responsabilidad es conservar lo que llega de
// ORB-SLAM3 y el journal ordenado; no corrige, fusiona ni transforma a world.
class RawMapDatabase
{
public:
    RawMapDatabase() = default;

    RawInsertResult InsertDelta(uint64_t arrival_id, const orbslam3_msgs::msg::OrbMap& delta);
    RawInsertResult InsertFullSnapshot(uint64_t arrival_id, const orbslam3_msgs::msg::OrbMap& snapshot);

    bool HasSubmap(const RawSubmapId& id) const;
    bool HasKeyFrame(const RawKeyFrameId& id) const;
    bool HasMapPoint(const RawMapPointId& id) const;
    const RawSubmap* GetSubmap(const RawSubmapId& id) const;
    const orbslam3_msgs::msg::OrbKeyFrame* GetKeyFrame(const RawKeyFrameId& id) const;
    const orbslam3_msgs::msg::OrbMapPoint* GetMapPoint(const RawMapPointId& id) const;
    std::vector<RawSubmapId> GetSubmapIds() const;
    std::vector<RawKeyFrameId> GetKeyFrameIdsForSubmap(const RawSubmapId& id) const;
    std::vector<RawMapPointId> GetMapPointIdsForSubmap(const RawSubmapId& id) const;

    RawDatabaseStats GetDatabaseStats() const;
    std::vector<RawJournalEntry> GetJournalCopy() const;
    const std::vector<RawJournalEntry>& Journal() const;

    void AddFiducialObservation(const RecordedFiducialObservation& observation);
    std::vector<RecordedFiducialObservation> GetFiducialObservationJournalCopy() const;
    std::vector<RecordedFiducialObservation> GetFiducialObservationsForArrival(
        uint64_t arrival_id) const;

    void Clear();

    bool SaveToPath(const std::string& path, std::string* error_message = nullptr) const;
    bool LoadFromPath(const std::string& path, std::string* error_message = nullptr);

private:
    RawInsertResult InsertMap(uint64_t arrival_id,
                              RawJournalEntryKind kind,
                              const orbslam3_msgs::msg::OrbMap& map);
    RawDatabaseStats ComputeStatsLocked() const;

    std::map<RawSubmapId, RawSubmap> submaps_;
    std::vector<RawJournalEntry> journal_;
    std::vector<RecordedFiducialObservation> fiducial_observation_journal_;
};

}  // namespace orbslam3_multi

#pragma once

#include "orbslam3_multi/raw_map_types.hpp"
#include "orbslam3_msgs/msg/orb_map_point.hpp"

#include <cstdint>
#include <map>
#include <optional>
#include <string>

namespace orbslam3_multi
{

enum class ScoreEventType : uint8_t
{
    NewMapPoint = 0,
    OrbSlamQualityUpdate = 1,
    MarkedBad = 2,

    // F1F: eventos reservados para subfases futuras. En 1F no tienen uso
    // funcional para evitar mezclar score raw con fusión, loops u optimización.
    FusionConfirmed = 3,
    Reobserved = 4,
    NotReobserved = 5,
    OptimizationAccepted = 6,
    GeometryInconsistent = 7,
};

struct LandmarkScoreRecord
{
    RawMapPointId id;
    float score = 0.0F;
    uint32_t observations = 0;
    float found_ratio = 0.0F;
    bool is_bad = false;
    bool descriptor_valid = false;
    ScoreEventType last_event = ScoreEventType::NewMapPoint;
    std::string reason;
};

struct LandmarkScoreUpdateResult
{
    bool created = false;
    bool updated = false;
    LandmarkScoreRecord record;
};

struct LandmarkScoreStats
{
    uint64_t tracked_points = 0;
    uint64_t bad_points = 0;
    float score_min = 0.0F;
    float score_mean = 0.0F;
    float score_max = 0.0F;
};

// F1F: centraliza el score inicial de landmarks. Entradas: eventos semánticos
// derivados de MapPoints raw. Salida: score acotado [0, 1]. Existe para que ni
// el servidor ni el builder escriban scores a mano.
class LandmarkScoreManager
{
public:
    LandmarkScoreUpdateResult ApplyOrbSlamQuality(
        const RawMapPointId& id,
        const orbslam3_msgs::msg::OrbMapPoint& mappoint);

    std::optional<LandmarkScoreRecord> GetScoreRecord(const RawMapPointId& id) const;
    float GetScoreOrDefault(const RawMapPointId& id) const;
    LandmarkScoreStats GetStats() const;

private:
    float ComputeRawScore(const orbslam3_msgs::msg::OrbMapPoint& mappoint,
                          bool descriptor_valid) const;
    bool HasValidDescriptor(const orbslam3_msgs::msg::OrbMapPoint& mappoint) const;

    std::map<RawMapPointId, LandmarkScoreRecord> records_;
};

const char* ToString(ScoreEventType event);

}  // namespace orbslam3_multi

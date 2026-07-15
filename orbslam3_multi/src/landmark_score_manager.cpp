#include "orbslam3_multi/landmark_score_manager.hpp"

#include <algorithm>

namespace orbslam3_multi
{
namespace
{

float Clamp01(float value)
{
    return std::max(0.0F, std::min(1.0F, value));
}

}  // namespace

const char* ToString(ScoreEventType event)
{
    switch (event)
    {
        case ScoreEventType::NewMapPoint:
            return "NewMapPoint";
        case ScoreEventType::OrbSlamQualityUpdate:
            return "OrbSlamQualityUpdate";
        case ScoreEventType::MarkedBad:
            return "MarkedBad";
        case ScoreEventType::FusionConfirmed:
            return "FusionConfirmed";
        case ScoreEventType::Reobserved:
            return "Reobserved";
        case ScoreEventType::NotReobserved:
            return "NotReobserved";
        case ScoreEventType::OptimizationAccepted:
            return "OptimizationAccepted";
        case ScoreEventType::GeometryInconsistent:
            return "GeometryInconsistent";
    }
    return "Unknown";
}

LandmarkScoreUpdateResult LandmarkScoreManager::ApplyOrbSlamQuality(
    const RawMapPointId& id,
    const orbslam3_msgs::msg::OrbMapPoint& mappoint)
{
    // F1F: el score inicial solo usa calidad local exportada por ORB-SLAM3. No
    // incorpora GT, loops, fusión ni optimización.
    LandmarkScoreUpdateResult result;
    const bool descriptor_valid = HasValidDescriptor(mappoint);
    const auto it = records_.find(id);
    result.created = it == records_.end();
    result.updated = !result.created;

    LandmarkScoreRecord record;
    if (!result.created)
    {
        record = it->second;
    }

    record.id = id;
    record.observations = mappoint.observations_count;
    record.found_ratio = mappoint.found_ratio;
    record.is_bad = mappoint.is_bad;
    record.descriptor_valid = descriptor_valid;
    record.score = ComputeRawScore(mappoint, descriptor_valid);
    record.last_event = mappoint.is_bad ? ScoreEventType::MarkedBad :
                        (result.created ? ScoreEventType::NewMapPoint :
                                          ScoreEventType::OrbSlamQualityUpdate);
    record.reason = "orbslam_raw_quality";

    records_[id] = record;
    result.record = record;
    return result;
}

std::optional<LandmarkScoreRecord> LandmarkScoreManager::GetScoreRecord(
    const RawMapPointId& id) const
{
    const auto it = records_.find(id);
    if (it == records_.end())
    {
        return std::nullopt;
    }
    return it->second;
}

float LandmarkScoreManager::GetScoreOrDefault(const RawMapPointId& id) const
{
    const auto record = GetScoreRecord(id);
    return record ? record->score : 0.0F;
}

LandmarkScoreStats LandmarkScoreManager::GetStats() const
{
    LandmarkScoreStats stats;
    stats.tracked_points = records_.size();
    if (records_.empty())
    {
        return stats;
    }

    bool first = true;
    double sum = 0.0;
    for (const auto& [_, record] : records_)
    {
        if (record.is_bad)
        {
            ++stats.bad_points;
        }
        if (first)
        {
            stats.score_min = record.score;
            stats.score_max = record.score;
            first = false;
        }
        else
        {
            stats.score_min = std::min(stats.score_min, record.score);
            stats.score_max = std::max(stats.score_max, record.score);
        }
        sum += static_cast<double>(record.score);
    }
    stats.score_mean = static_cast<float>(sum / static_cast<double>(records_.size()));
    return stats;
}

float LandmarkScoreManager::ComputeRawScore(
    const orbslam3_msgs::msg::OrbMapPoint& mappoint,
    bool descriptor_valid) const
{
    if (mappoint.is_bad)
    {
        return 0.0F;
    }

    const float observation_score =
        0.55F * Clamp01(static_cast<float>(mappoint.observations_count) / 8.0F);
    const float found_score = 0.35F * Clamp01(mappoint.found_ratio);
    const float descriptor_score = descriptor_valid ? 0.10F : 0.0F;
    return Clamp01(observation_score + found_score + descriptor_score);
}

bool LandmarkScoreManager::HasValidDescriptor(
    const orbslam3_msgs::msg::OrbMapPoint& mappoint) const
{
    for (const auto value : mappoint.descriptor.data)
    {
        if (value != 0U)
        {
            return true;
        }
    }
    return false;
}

}  // namespace orbslam3_multi

#pragma once

#include "orbslam3_multi/global_pose_store.hpp"
#include "orbslam3_multi/global_sparse_point.hpp"
#include "orbslam3_multi/landmark_score_manager.hpp"
#include "orbslam3_multi/raw_map_database.hpp"

#include <cstdint>
#include <vector>

namespace orbslam3_multi
{

struct GlobalMapBuildStats
{
    uint64_t total_submaps = 0;
    uint64_t anchored_submaps = 0;
    uint64_t skipped_unanchored_submaps = 0;
    uint64_t raw_points = 0;
    uint64_t candidate_points = 0;
    uint64_t returned_points = 0;
    uint64_t server_corrected_points = 0;
    uint64_t keyframe_projected_points = 0;
    uint64_t fallback_submap_points = 0;
    uint64_t bad_skipped = 0;
    uint64_t invalid_pose_skipped = 0;
    uint64_t below_score_skipped = 0;
    float score_min = 0.0F;
    float score_mean = 0.0F;
    float score_max = 0.0F;
};

struct GlobalMapBuildResult
{
    std::vector<GlobalSparsePoint> points;
    GlobalMapBuildStats stats;
};

// F1F: construye la nube sparse global publicable a partir de raw + poses +
// scores. No calcula score, no crea anchors y no modifica RawMapDatabase.
class GlobalMapBuilder
{
public:
    GlobalMapBuildResult Build(
        const RawMapDatabase& raw_db,
        const GlobalPoseStore& pose_store,
        const LandmarkScoreManager& score_manager,
        float min_score_to_publish) const;

private:
    uint64_t MakeGlobalMapPointId(const RawMapPointId& id) const;
};

}  // namespace orbslam3_multi

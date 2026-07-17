#pragma once

#include "orbslam3_multi/covisibility_database.hpp"
#include "orbslam3_multi/global_pose_store.hpp"
#include "orbslam3_multi/loop_candidate.hpp"
#include "orbslam3_multi/raw_map_database.hpp"

#include <cstdint>

namespace orbslam3_multi
{

struct LoopDetectorConfig
{
    uint64_t min_kf_gap_same_submap = 20;
    uint64_t max_candidates = 10;
    uint64_t max_candidates_per_submap = 3;
};

// F1N: frontera segura para detección de candidatos. El formato BoW no está
// presente en este checkout; por tanto no fabrica scores ni candidatos. La
// clase deja estable la API, el despacho y el filtro de covisibilidad para que
// el workspace completo conecte la comparación BoW real posteriormente.
class LoopDetector
{
public:
    LoopDetector() = default;
    explicit LoopDetector(const LoopDetectorConfig& config);

    void Configure(const LoopDetectorConfig& config);
    const LoopDetectorConfig& GetConfig() const;

    LoopCandidateResult ProcessNewKeyFrame(
        const RawKeyFrameId& query_kf_id,
        const RawMapDatabase& raw_db,
        const GlobalPoseStore* pose_store,
        const CovisibilityDatabase& covisibility_db) const;

private:
    LoopDetectorConfig config_;
};

}  // namespace orbslam3_multi

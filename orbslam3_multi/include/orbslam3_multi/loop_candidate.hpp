#pragma once

#include "orbslam3_multi/raw_map_types.hpp"

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace orbslam3_multi
{

// F1N: candidato visual provisional. En esta entrega no se inventa un score:
// se reserva para cuando el contrato de BoW del wrapper este disponible.
struct LoopCandidate
{
    RawKeyFrameId query_kf_id;
    RawKeyFrameId candidate_kf_id;
    RawSubmapId query_submap_id;
    RawSubmapId candidate_submap_id;
    double bow_score = 0.0;
    uint64_t rank = 0;
    bool same_drone = false;
    bool same_submap = false;
    uint64_t kf_gap = 0;
    bool candidate_has_world_pose = false;
    bool candidate_is_anchored = false;
    bool candidate_is_bad = false;
    bool already_confirmed_covisibility = false;
    std::string source = "BOW";
    std::string rejection_reason;
};

struct LoopCandidateResult
{
    RawKeyFrameId query_kf_id;
    bool processed = false;
    std::string reason;
    uint64_t indexed_keyframes = 0;
    uint64_t compared_keyframes = 0;
    uint64_t candidates_raw = 0;
    uint64_t candidates_after_filter = 0;
    uint64_t skipped_confirmed_covisibility = 0;
    std::vector<LoopCandidate> candidates;
    std::optional<LoopCandidate> best_candidate;
};

}  // namespace orbslam3_multi

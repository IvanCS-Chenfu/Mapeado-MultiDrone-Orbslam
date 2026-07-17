#include "orbslam3_multi/loop_detector.hpp"

namespace orbslam3_multi
{

LoopDetector::LoopDetector(const LoopDetectorConfig& config)
    : config_(config)
{
}

void LoopDetector::Configure(const LoopDetectorConfig& config)
{
    config_ = config;
}

const LoopDetectorConfig& LoopDetector::GetConfig() const
{
    return config_;
}

LoopCandidateResult LoopDetector::ProcessNewKeyFrame(
    const RawKeyFrameId& query_kf_id,
    const RawMapDatabase& raw_db,
    const GlobalPoseStore* pose_store,
    const CovisibilityDatabase& covisibility_db) const
{
    (void)pose_store;
    (void)covisibility_db;

    LoopCandidateResult result;
    result.query_kf_id = query_kf_id;
    if (!raw_db.HasKeyFrame(query_kf_id))
    {
        result.reason = "query_keyframe_missing";
        return result;
    }

    // El contrato de orbslam3_msgs no está incluido en este repositorio. No se
    // puede leer un BoW/FeatureVector de forma fiable sin inventar campos o una
    // métrica. La integración posterior debe sustituir esta salida conservadora
    // por indexación y comparación BoW real.
    result.reason = "bow_data_unavailable_in_current_checkout";
    return result;
}

}  // namespace orbslam3_multi

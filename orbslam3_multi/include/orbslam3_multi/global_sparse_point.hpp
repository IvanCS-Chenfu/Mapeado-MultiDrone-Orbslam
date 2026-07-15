#pragma once

#include <cstdint>

namespace orbslam3_multi
{

// F1F: punto sparse global publicable producido por el backend nuevo. Mantiene
// la identidad raw de origen para no confundirlo con futuros fused landmarks.
struct GlobalSparsePoint
{
    uint64_t global_mappoint_id = 0;
    uint32_t drone_id = 0;
    uint64_t map_epoch = 0;
    uint64_t local_mappoint_id = 0;

    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

    float score = 0.0F;
    uint32_t observations = 0;
    bool from_anchored_submap = false;
    bool is_fused = false;
};

}  // namespace orbslam3_multi

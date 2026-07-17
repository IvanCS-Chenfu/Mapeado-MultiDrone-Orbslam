#pragma once

#include "orbslam3_multi/raw_map_database.hpp"

#include <Eigen/Core>

#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace orbslam3_multi
{

// F1M: solo se almacenan relaciones ya confirmadas. Los candidatos BoW y sus
// estados transitorios pertenecen al detector de loops, no a esta base.
enum class CovisibilityEdgeSource : uint8_t
{
    Orbslam3Native = 0,
    ServerLoopGeometric = 1,
};

struct CovisibilityEdge
{
    RawKeyFrameId kf_a;
    RawKeyFrameId kf_b;
    double weight = 0.0;
    CovisibilityEdgeSource source = CovisibilityEdgeSource::Orbslam3Native;
    // Medicion inmutable en la direccion canonica kf_a_T_kf_b.
    Eigen::Matrix4d relative_pose_measured = Eigen::Matrix4d::Identity();
    // Pose relativa vigente, actualizable solo despues de un apply aceptado.
    Eigen::Matrix4d relative_pose_current = Eigen::Matrix4d::Identity();
    double information_weight = 0.0;
    uint64_t shared_mappoints_or_inliers = 0;
    uint64_t created_arrival_id = 0;
    uint64_t updated_revision = 0;
};

struct CovisibilityImportResult
{
    uint64_t arrival_id = 0;
    uint64_t keyframes_examined = 0;
    uint64_t connections_examined = 0;
    uint64_t edges_added = 0;
    uint64_t edges_updated = 0;
    uint64_t edges_skipped_low_weight = 0;
    uint64_t edges_skipped_missing_keyframe = 0;
};

struct CovisibilityDatabaseStats
{
    uint64_t confirmed_edges = 0;
    uint64_t orbslam3_native_edges = 0;
    uint64_t server_loop_geometric_edges = 0;
    uint64_t revision = 0;
};

class CovisibilityDatabase
{
public:
    // Importa solo covisibilidad nativa intra-submapa ya calculada por ORB-SLAM3.
    CovisibilityImportResult ImportOrbslam3Native(
        const RawMapDatabase& raw_db,
        uint64_t arrival_id,
        double min_weight);

    // Inserta una relacion que ya supero la verificacion geometrica del servidor.
    // Devuelve false para entradas invalidas o duplicadas sin cambios de soporte.
    bool AddConfirmedLoopEdge(const CovisibilityEdge& edge, bool* added = nullptr);

    bool HasConfirmedEdge(const RawKeyFrameId& a, const RawKeyFrameId& b) const;
    std::optional<CovisibilityEdge> GetEdge(
        const RawKeyFrameId& a,
        const RawKeyFrameId& b) const;
    std::vector<CovisibilityEdge> GetNeighbors(
        const RawKeyFrameId& keyframe_id,
        double min_weight) const;
    std::vector<CovisibilityEdge> GetEdgesForWindow(
        const std::vector<RawKeyFrameId>& keyframe_ids,
        double min_weight) const;

    // No cambia relative_pose_measured. La revision debe avanzar de forma
    // monotona para que los consumidores puedan invalidar caches si hace falta.
    bool UpdateRelativePoseCurrent(
        const RawKeyFrameId& a,
        const RawKeyFrameId& b,
        const Eigen::Matrix4d& relative_pose_current,
        uint64_t updated_revision);

    void Clear();
    CovisibilityDatabaseStats GetStats() const;

private:
    using EdgeKey = std::pair<RawKeyFrameId, RawKeyFrameId>;

    static bool IsValidEdge(const CovisibilityEdge& edge);
    static void Canonicalize(CovisibilityEdge& edge);
    static EdgeKey MakeKey(const RawKeyFrameId& a, const RawKeyFrameId& b);

    std::map<EdgeKey, CovisibilityEdge> edges_;
    uint64_t revision_ = 0;
};

const char* ToString(CovisibilityEdgeSource source);

}  // namespace orbslam3_multi

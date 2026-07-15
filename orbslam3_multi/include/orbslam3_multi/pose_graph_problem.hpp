#pragma once

#include "orbslam3_multi/raw_map_types.hpp"

#include <Eigen/Core>

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace orbslam3_multi
{

enum class PoseGraphEdgeType : uint8_t
{
    TemporalNeighbor = 0,
    SoftConsistency = 1,
};

enum class PoseGraphPriorType : uint8_t
{
    FiducialHard = 0,
    FiducialTarget = 1,
    CurrentPoseSoft = 2,
};

enum class PoseGraphPropagationMode : uint8_t
{
    NearestControlVertex = 0,
    PathSegment = 1,
    InheritLastCorrection = 2,
};

enum class FiducialConnectivityEdgeStatus : uint8_t
{
    DirectObserved = 0,
    DirectUncertain = 1,
    SubdivisionCandidate = 2,
    SubdividedConfirmed = 3,
    BypassConfirmed = 4,
};

// F1I: vertice temporal de un problema de optimizacion. Conserva la pose
// inicial y la politica variable/fija, pero no representa estado persistente.
struct PoseGraphVertex
{
    RawKeyFrameId keyframe_id;
    RawSubmapId submap_id;
    Eigen::Matrix4d initial_world_T_kf = Eigen::Matrix4d::Identity();
    bool is_variable = false;
    bool is_fixed = false;
    bool is_hard_fiducial = false;
    bool is_anchor_neighborhood = false;
    uint64_t support_count = 0;
    std::string selection_reason;
    double weight = 1.0;
};

// F1I: arista interna del grafo temporal. La medida se guarda como transformada
// relativa entre poses world actuales, suficiente para que el solver futuro no
// tenga que volver a consultar raw al construir el coste inicial.
struct PoseGraphEdge
{
    uint64_t edge_id = 0;
    RawKeyFrameId from_keyframe_id;
    RawKeyFrameId to_keyframe_id;
    PoseGraphEdgeType edge_type = PoseGraphEdgeType::TemporalNeighbor;
    Eigen::Matrix4d relative_T_from_to = Eigen::Matrix4d::Identity();
    double weight = 1.0;
    uint64_t intermediate_keyframe_count = 0;
    uint64_t support_keyframe_count = 0;
    double support_length_m = 0.0;
    double support_density_kfs_per_m = 0.0;
    double support_rigidity_multiplier = 1.0;
    std::string source;
};

// F1I: prior de pose para vertices. Puede ser hard fiducial, objetivo fiducial
// de la tarea, o prior suave a pose actual para evitar deformacion libre.
struct PoseGraphPrior
{
    RawKeyFrameId keyframe_id;
    PoseGraphPriorType prior_type = PoseGraphPriorType::CurrentPoseSoft;
    Eigen::Matrix4d target_world_T_kf = Eigen::Matrix4d::Identity();
    double weight_translation = 1.0;
    double weight_rotation = 1.0;
    bool hard = false;
    std::string source;
};

// F1I: plan de propagacion para KFs afectados que no entran como variables.
// En 1I solo se construye y loggea; la aplicacion real queda para subfases
// posteriores del optimizador.
struct PoseGraphPropagationPlanEntry
{
    RawKeyFrameId affected_keyframe_id;
    RawSubmapId submap_id;
    uint64_t path_id = 0;
    RawKeyFrameId control_vertex_a;
    std::optional<RawKeyFrameId> control_vertex_b;
    PoseGraphPropagationMode mode = PoseGraphPropagationMode::NearestControlVertex;
    // F1L: los KFs no vertices deben interpolarse dentro del tramo real entre
    // controles. Guardamos alpha/distancia desde PoseGraphBuilder para no
    // deducirlo despues con local_kf_id, que fue una fuente de propagaciones
    // incoherentes cuando el grafo incluye vertices de esquina.
    double segment_alpha = 0.0;
    double distance_from_a_m = 0.0;
    double segment_length_m = 0.0;
    uint64_t control_span_kf_gap = 0;
    std::string reason;
};

struct PoseGraphProblemSummary
{
    uint64_t vertices = 0;
    uint64_t variable_vertices = 0;
    uint64_t fixed_vertices = 0;
    uint64_t hard_fiducial_vertices = 0;
    uint64_t edges = 0;
    uint64_t priors = 0;
    uint64_t affected_non_variable_keyframes = 0;
    uint64_t propagation_entries = 0;
};

// F1L/1I-reopen: resumen compacto de cobertura de la ventana de controles.
// Si no se puede representar la ventana con tramos acotados, el builder debe
// rechazar el problema antes del solver en vez de dejar que 1J optimice una
// deformacion mal condicionada.
struct PoseGraphCoverageSummary
{
    uint64_t window_keyframes = 0;
    uint64_t control_vertices = 0;
    uint64_t max_control_span_kfs = 0;
    double max_control_span_m = 0.0;
    uint64_t uncovered_long_segments = 0;
    uint64_t mandatory_vertices_missing = 0;
    bool max_vertices_exhausted = false;
    bool coverage_complete = false;
    std::string reason;
};

// F1L: bloque diagnostico opcional para dumps offline. Lo rellena el servidor
// con poses mapa y GT externo para visualizar/medir; el solver no debe usarlo.
struct PoseGraphDebugKeyFramePose
{
    RawKeyFrameId keyframe_id;
    Eigen::Matrix4d map_world_T_kf = Eigen::Matrix4d::Identity();
    bool has_gt = false;
    Eigen::Matrix4d gt_world_T_kf = Eigen::Matrix4d::Identity();
    double association_dt_sec = -1.0;
    std::string association_quality;
};

// F1I-reopen: diagnostico topologico ligero para elegir fiduciales frontera.
// No es un grafo global permanente; solo viaja con el PoseGraphProblem temporal
// para que logs y fases posteriores sepan por que anchors se fijaron.
struct FiducialConnectivityEdge
{
    int32_t from_fiducial_id = -1;
    int32_t to_fiducial_id = -1;
    RawKeyFrameId from_keyframe_id;
    RawKeyFrameId to_keyframe_id;
    FiducialConnectivityEdgeStatus status =
        FiducialConnectivityEdgeStatus::DirectObserved;
    bool selected_as_branch_anchor = false;
    bool independent_branch = false;
    uint64_t kf_gap = 0;
    std::string reason;
};

struct PoseGraphAnchorPreservationSummary
{
    bool required = false;
    bool satisfied = false;
    uint64_t independent_branches = 0;
    uint64_t branch_anchor_count = 0;
    uint64_t previous_fiducial_fixed_count = 0;
    uint64_t previous_fiducial_neighborhood_fixed_count = 0;
    uint64_t subdivision_candidates = 0;
    uint64_t subdivided_confirmed = 0;
    std::string reason;
};

// F1I: problema temporal completo creado a partir de una tarea concreta. No se
// guarda como grafo global ni modifica RawMapDatabase/GlobalPoseStore.
struct PoseGraphProblem
{
    uint64_t task_id = 0;
    std::string task_type;
    std::string source;
    RawSubmapId submap_id;
    RawKeyFrameId target_keyframe_id;
    bool rebuilt_from_partial_checkpoint = false;
    uint64_t partial_parent_task_id = 0;
    uint64_t partial_retry_count = 0;
    std::vector<PoseGraphVertex> vertices;
    std::vector<PoseGraphEdge> edges;
    std::vector<PoseGraphPrior> priors;
    std::vector<RawKeyFrameId> fixed_keyframes;
    std::vector<RawKeyFrameId> variable_keyframes;
    std::vector<RawKeyFrameId> affected_non_variable_keyframes;
    std::vector<PoseGraphPropagationPlanEntry> propagation_plan;
    std::vector<FiducialConnectivityEdge> fiducial_connectivity_edges;
    std::vector<PoseGraphDebugKeyFramePose> debug_keyframes;
    PoseGraphAnchorPreservationSummary anchor_preservation;
    PoseGraphCoverageSummary coverage;
    PoseGraphProblemSummary summary;
};

const char* ToString(PoseGraphEdgeType type);
const char* ToString(PoseGraphPriorType type);
const char* ToString(PoseGraphPropagationMode mode);
const char* ToString(FiducialConnectivityEdgeStatus status);

}  // namespace orbslam3_multi

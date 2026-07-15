#pragma once

#include "orbslam3_multi/global_pose_store.hpp"
#include "orbslam3_multi/optimization_result.hpp"
#include "orbslam3_multi/pose_graph_problem.hpp"
#include "orbslam3_multi/raw_map_database.hpp"

namespace orbslam3_multi
{

struct OptimizationManagerConfig
{
    double dryrun_min_improvement_ratio = 0.05;
    double dryrun_partial_min_improvement_ratio = 0.70;
    double dryrun_max_allowed_delta_t = 30.0;
    double dryrun_partial_max_allowed_delta_t = 35.0;
    double dryrun_max_allowed_delta_yaw = 1.2;
    double dryrun_max_final_error_t = 0.35;
    bool dryrun_require_cost_decrease = true;
    double solver_step_fraction = 0.95;
    double post_apply_internal_broken_edge_t = 2.50;
    double post_apply_internal_max_growth_t = 1.50;
    double post_apply_fiducial_absurd_error_t = 10.0;
};

// F1J: ejecuta una optimizacion dry-run sobre copias temporales de las poses.
// Entradas: PoseGraphProblem ya construido, RawMapDatabase y GlobalPoseStore
// de solo lectura. Salida: OptimizationDryRunResult en memoria. No aplica ni
// publica correcciones; esa frontera queda reservada a 1K.
class OptimizationManager
{
public:
    OptimizationManager() = default;
    explicit OptimizationManager(const OptimizationManagerConfig& config);

    void Configure(const OptimizationManagerConfig& config);
    const OptimizationManagerConfig& GetConfig() const;

    OptimizationDryRunResult RunDryRun(
        const PoseGraphProblem& graph,
        const RawMapDatabase& raw_db,
        const GlobalPoseStore& pose_store) const;

    // F1L: ruta offline para iterar pesos/solver sobre un PoseGraphProblem
    // serializado sin reconstruir RawMapDatabase ni GlobalPoseStore. Optimiza
    // vertices y omite propuestas propagadas; no sustituye validacion live.
    OptimizationDryRunResult RunDryRunGraphOnly(
        const PoseGraphProblem& graph) const;

    // F1K: aplica un dry-run `useful=true` en GlobalPoseStore. Entradas:
    // resultado de 1J, grafo original y bases raw/global. Salida:
    // OptimizationApplyResult con detalle de KFs escritos. No toca raw DB y
    // rechaza `partial_candidate` hasta que 1L aporte backup/validacion.
    OptimizationApplyResult ApplyUsefulResult(
        const OptimizationDryRunResult& dry_run,
        const PoseGraphProblem& graph,
        const RawMapDatabase& raw_db,
        GlobalPoseStore& pose_store) const;

    // F1L: aplica candidatos normales o parciales bajo la proteccion de
    // backup/validacion del servidor. Los parciales solo se permiten si
    // `allow_partial_candidate` es true; el resultado debe pasar despues por
    // ValidatePostApply antes de confirmarse.
    OptimizationApplyResult ApplyCandidateResult(
        const OptimizationDryRunResult& dry_run,
        const PoseGraphProblem& graph,
        const RawMapDatabase& raw_db,
        GlobalPoseStore& pose_store,
        bool allow_partial_candidate) const;

    PostApplyValidationResult ValidatePostApply(
        const OptimizationDryRunResult& dry_run,
        const PoseGraphProblem& graph,
        const OptimizationApplyResult& apply,
        const GlobalPoseStore& pose_store,
        bool force_reject) const;

private:
    OptimizationDryRunResult RunDryRunInternal(
        const PoseGraphProblem& graph,
        const RawMapDatabase* raw_db,
        const GlobalPoseStore* pose_store,
        bool include_propagation) const;

    OptimizationManagerConfig config_;
};

}  // namespace orbslam3_multi

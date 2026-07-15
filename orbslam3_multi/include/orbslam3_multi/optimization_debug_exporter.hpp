#pragma once

#include "orbslam3_multi/optimization_result.hpp"
#include "orbslam3_multi/pose_graph_problem.hpp"

#include <string>

namespace orbslam3_multi
{

struct OptimizationDebugExportResult
{
    bool success = false;
    std::string path;
    std::string reason;
};

// F1J: exportador opcional de inspeccion. No participa en el flujo normal de
// datos y el servidor no debe leer su salida; solo crea un SVG cuando el
// usuario/parametro lo pide explicitamente.
class OptimizationDebugExporter
{
public:
    OptimizationDebugExportResult ExportDryRun2DPlot(
        const PoseGraphProblem& graph,
        const OptimizationDryRunResult& result,
        const std::string& output_path) const;
};

}  // namespace orbslam3_multi

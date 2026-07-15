#include "orbslam3_multi/optimization_debug_exporter.hpp"

#include <algorithm>
#include <fstream>
#include <limits>

namespace orbslam3_multi
{

OptimizationDebugExportResult OptimizationDebugExporter::ExportDryRun2DPlot(
    const PoseGraphProblem& graph,
    const OptimizationDryRunResult& result,
    const std::string& output_path) const
{
    OptimizationDebugExportResult export_result;
    export_result.path = output_path;

    std::ofstream out(output_path);
    if (!out)
    {
        export_result.reason = "open_failed";
        return export_result;
    }

    double min_x = std::numeric_limits<double>::infinity();
    double max_x = -std::numeric_limits<double>::infinity();
    double min_y = std::numeric_limits<double>::infinity();
    double max_y = -std::numeric_limits<double>::infinity();
    auto observe = [&](const Eigen::Matrix4d& pose)
    {
        min_x = std::min(min_x, pose(0, 3));
        max_x = std::max(max_x, pose(0, 3));
        min_y = std::min(min_y, pose(1, 3));
        max_y = std::max(max_y, pose(1, 3));
    };
    for (const auto& proposal : result.proposed_vertex_poses)
    {
        observe(proposal.before_world_T_kf);
        observe(proposal.proposed_world_T_kf);
    }
    for (const auto& proposal : result.proposed_propagated_poses)
    {
        observe(proposal.before_world_T_kf);
        observe(proposal.proposed_world_T_kf);
    }
    if (!std::isfinite(min_x) || !std::isfinite(min_y))
    {
        export_result.reason = "empty_result";
        return export_result;
    }

    const double margin = 1.0;
    min_x -= margin;
    max_x += margin;
    min_y -= margin;
    max_y += margin;
    const double width = std::max(1.0, max_x - min_x);
    const double height = std::max(1.0, max_y - min_y);
    const double canvas = 900.0;
    auto sx = [&](double x) { return 40.0 + (x - min_x) / width * (canvas - 80.0); };
    auto sy = [&](double y) { return canvas - 40.0 - (y - min_y) / height * (canvas - 80.0); };

    out << "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"900\" height=\"900\" viewBox=\"0 0 900 900\">\n";
    out << "<rect width=\"900\" height=\"900\" fill=\"white\"/>\n";
    out << "<text x=\"24\" y=\"28\" font-size=\"16\">F1J task "
        << graph.task_id << " before=" << result.before_error_t
        << " after=" << result.after_error_t
        << " useful=" << (result.useful ? "true" : "false") << "</text>\n";

    for (const auto& edge : graph.edges)
    {
        const auto before_from = std::find_if(
            result.proposed_vertex_poses.begin(),
            result.proposed_vertex_poses.end(),
            [&edge](const OptimizationPoseProposal& proposal)
            {
                return proposal.keyframe_id == edge.from_keyframe_id;
            });
        const auto before_to = std::find_if(
            result.proposed_vertex_poses.begin(),
            result.proposed_vertex_poses.end(),
            [&edge](const OptimizationPoseProposal& proposal)
            {
                return proposal.keyframe_id == edge.to_keyframe_id;
            });
        if (before_from == result.proposed_vertex_poses.end() ||
            before_to == result.proposed_vertex_poses.end())
        {
            continue;
        }
        out << "<line x1=\"" << sx(before_from->before_world_T_kf(0, 3))
            << "\" y1=\"" << sy(before_from->before_world_T_kf(1, 3))
            << "\" x2=\"" << sx(before_to->before_world_T_kf(0, 3))
            << "\" y2=\"" << sy(before_to->before_world_T_kf(1, 3))
            << "\" stroke=\"red\" stroke-width=\"1\" opacity=\"0.55\"/>\n";
        out << "<line x1=\"" << sx(before_from->proposed_world_T_kf(0, 3))
            << "\" y1=\"" << sy(before_from->proposed_world_T_kf(1, 3))
            << "\" x2=\"" << sx(before_to->proposed_world_T_kf(0, 3))
            << "\" y2=\"" << sy(before_to->proposed_world_T_kf(1, 3))
            << "\" stroke=\"green\" stroke-width=\"1\" opacity=\"0.55\"/>\n";
    }

    for (const auto& proposal : result.proposed_vertex_poses)
    {
        out << "<circle cx=\"" << sx(proposal.before_world_T_kf(0, 3))
            << "\" cy=\"" << sy(proposal.before_world_T_kf(1, 3))
            << "\" r=\"5\" fill=\"red\"/>\n";
        out << "<circle cx=\"" << sx(proposal.proposed_world_T_kf(0, 3))
            << "\" cy=\"" << sy(proposal.proposed_world_T_kf(1, 3))
            << "\" r=\"5\" fill=\"green\"/>\n";
    }
    for (const auto& proposal : result.proposed_propagated_poses)
    {
        out << "<circle cx=\"" << sx(proposal.before_world_T_kf(0, 3))
            << "\" cy=\"" << sy(proposal.before_world_T_kf(1, 3))
            << "\" r=\"3\" fill=\"blue\"/>\n";
        out << "<circle cx=\"" << sx(proposal.proposed_world_T_kf(0, 3))
            << "\" cy=\"" << sy(proposal.proposed_world_T_kf(1, 3))
            << "\" r=\"3\" fill=\"gold\"/>\n";
    }
    out << "<text x=\"24\" y=\"870\" font-size=\"13\">red/green vertices, blue/gold propagated; SVG is debug only, not an internal API.</text>\n";
    out << "</svg>\n";

    export_result.success = true;
    export_result.reason = "exported";
    return export_result;
}

}  // namespace orbslam3_multi

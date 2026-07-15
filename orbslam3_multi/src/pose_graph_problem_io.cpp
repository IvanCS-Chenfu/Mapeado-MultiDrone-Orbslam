#include "orbslam3_multi/pose_graph_problem_io.hpp"

#include <fstream>
#include <iomanip>
#include <sstream>
#include <vector>

namespace orbslam3_multi
{
namespace
{

void WriteKeyFrameId(std::ostream& out, const RawKeyFrameId& id)
{
    out << id.drone_id << '\t'
        << id.map_epoch << '\t'
        << id.local_kf_id;
}

bool ReadKeyFrameId(std::istream& in, RawKeyFrameId& id)
{
    return static_cast<bool>(in >> id.drone_id >> id.map_epoch >> id.local_kf_id);
}

void WriteSubmapId(std::ostream& out, const RawSubmapId& id)
{
    out << id.drone_id << '\t' << id.map_epoch;
}

bool ReadSubmapId(std::istream& in, RawSubmapId& id)
{
    return static_cast<bool>(in >> id.drone_id >> id.map_epoch);
}

void WriteMatrix(std::ostream& out, const Eigen::Matrix4d& matrix)
{
    for (int r = 0; r < 4; ++r)
    {
        for (int c = 0; c < 4; ++c)
        {
            out << '\t' << matrix(r, c);
        }
    }
}

bool ReadMatrix(std::istream& in, Eigen::Matrix4d& matrix)
{
    for (int r = 0; r < 4; ++r)
    {
        for (int c = 0; c < 4; ++c)
        {
            if (!(in >> matrix(r, c)))
            {
                return false;
            }
        }
    }
    return true;
}

std::string EncodeString(std::string value)
{
    for (char& ch : value)
    {
        if (ch == '\t' || ch == '\n' || ch == '\r')
        {
            ch = '_';
        }
    }
    return value.empty() ? "-" : value;
}

std::string DecodeString(const std::string& value)
{
    return value == "-" ? std::string() : value;
}

}  // namespace

PoseGraphProblemIoResult SavePoseGraphProblem(
    const PoseGraphProblem& problem,
    const std::string& path)
{
    PoseGraphProblemIoResult result;
    std::ofstream out(path);
    if (!out)
    {
        result.reason = "open_failed";
        return result;
    }

    out << std::setprecision(17);
    out << "F1L_POSE_GRAPH_DUMP\t1\n";
    out << "problem\t" << problem.task_id << '\t'
        << EncodeString(problem.task_type) << '\t'
        << EncodeString(problem.source) << '\t';
    WriteSubmapId(out, problem.submap_id);
    out << '\t';
    WriteKeyFrameId(out, problem.target_keyframe_id);
    out << '\t' << (problem.rebuilt_from_partial_checkpoint ? 1 : 0)
        << '\t' << problem.partial_parent_task_id
        << '\t' << problem.partial_retry_count << '\n';

    out << "summary\t" << problem.summary.vertices << '\t'
        << problem.summary.variable_vertices << '\t'
        << problem.summary.fixed_vertices << '\t'
        << problem.summary.hard_fiducial_vertices << '\t'
        << problem.summary.edges << '\t'
        << problem.summary.priors << '\t'
        << problem.summary.affected_non_variable_keyframes << '\t'
        << problem.summary.propagation_entries << '\n';

    out << "coverage\t" << problem.coverage.window_keyframes << '\t'
        << problem.coverage.control_vertices << '\t'
        << problem.coverage.max_control_span_kfs << '\t'
        << problem.coverage.max_control_span_m << '\t'
        << problem.coverage.uncovered_long_segments << '\t'
        << problem.coverage.mandatory_vertices_missing << '\t'
        << (problem.coverage.max_vertices_exhausted ? 1 : 0) << '\t'
        << (problem.coverage.coverage_complete ? 1 : 0) << '\t'
        << EncodeString(problem.coverage.reason) << '\n';

    out << "anchor\t" << (problem.anchor_preservation.required ? 1 : 0) << '\t'
        << (problem.anchor_preservation.satisfied ? 1 : 0) << '\t'
        << problem.anchor_preservation.independent_branches << '\t'
        << problem.anchor_preservation.branch_anchor_count << '\t'
        << problem.anchor_preservation.previous_fiducial_fixed_count << '\t'
        << problem.anchor_preservation.previous_fiducial_neighborhood_fixed_count << '\t'
        << problem.anchor_preservation.subdivision_candidates << '\t'
        << problem.anchor_preservation.subdivided_confirmed << '\t'
        << EncodeString(problem.anchor_preservation.reason) << '\n';

    for (const auto& vertex : problem.vertices)
    {
        out << "vertex\t";
        WriteKeyFrameId(out, vertex.keyframe_id);
        out << '\t';
        WriteSubmapId(out, vertex.submap_id);
        WriteMatrix(out, vertex.initial_world_T_kf);
        out << '\t' << (vertex.is_variable ? 1 : 0)
            << '\t' << (vertex.is_fixed ? 1 : 0)
            << '\t' << (vertex.is_hard_fiducial ? 1 : 0)
            << '\t' << (vertex.is_anchor_neighborhood ? 1 : 0)
            << '\t' << vertex.support_count
            << '\t' << vertex.weight
            << '\t' << EncodeString(vertex.selection_reason) << '\n';
    }

    for (const auto& edge : problem.edges)
    {
        out << "edge\t" << edge.edge_id << '\t';
        WriteKeyFrameId(out, edge.from_keyframe_id);
        out << '\t';
        WriteKeyFrameId(out, edge.to_keyframe_id);
        out << '\t' << static_cast<int>(edge.edge_type);
        WriteMatrix(out, edge.relative_T_from_to);
        out << '\t' << edge.weight
            << '\t' << edge.intermediate_keyframe_count
            << '\t' << edge.support_keyframe_count
            << '\t' << edge.support_length_m
            << '\t' << edge.support_density_kfs_per_m
            << '\t' << edge.support_rigidity_multiplier
            << '\t' << EncodeString(edge.source) << '\n';
    }

    for (const auto& prior : problem.priors)
    {
        out << "prior\t";
        WriteKeyFrameId(out, prior.keyframe_id);
        out << '\t' << static_cast<int>(prior.prior_type);
        WriteMatrix(out, prior.target_world_T_kf);
        out << '\t' << prior.weight_translation
            << '\t' << prior.weight_rotation
            << '\t' << (prior.hard ? 1 : 0)
            << '\t' << EncodeString(prior.source) << '\n';
    }

    for (const auto& keyframe_id : problem.fixed_keyframes)
    {
        out << "fixed\t";
        WriteKeyFrameId(out, keyframe_id);
        out << '\n';
    }
    for (const auto& keyframe_id : problem.variable_keyframes)
    {
        out << "variable\t";
        WriteKeyFrameId(out, keyframe_id);
        out << '\n';
    }
    for (const auto& keyframe_id : problem.affected_non_variable_keyframes)
    {
        out << "affected\t";
        WriteKeyFrameId(out, keyframe_id);
        out << '\n';
    }

    for (const auto& debug : problem.debug_keyframes)
    {
        out << "debug_kf\t";
        WriteKeyFrameId(out, debug.keyframe_id);
        WriteMatrix(out, debug.map_world_T_kf);
        out << '\t' << (debug.has_gt ? 1 : 0);
        WriteMatrix(out, debug.gt_world_T_kf);
        out << '\t' << debug.association_dt_sec
            << '\t' << EncodeString(debug.association_quality) << '\n';
    }

    for (const auto& entry : problem.propagation_plan)
    {
        out << "propagation\t";
        WriteKeyFrameId(out, entry.affected_keyframe_id);
        out << '\t';
        WriteSubmapId(out, entry.submap_id);
        out << '\t' << entry.path_id << '\t';
        WriteKeyFrameId(out, entry.control_vertex_a);
        out << '\t' << (entry.control_vertex_b ? 1 : 0);
        if (entry.control_vertex_b)
        {
            out << '\t';
            WriteKeyFrameId(out, entry.control_vertex_b.value());
        }
        else
        {
            out << "\t0\t0\t0";
        }
        out << '\t' << static_cast<int>(entry.mode)
            << '\t' << entry.segment_alpha
            << '\t' << entry.distance_from_a_m
            << '\t' << entry.segment_length_m
            << '\t' << entry.control_span_kf_gap
            << '\t' << EncodeString(entry.reason) << '\n';
    }

    for (const auto& edge : problem.fiducial_connectivity_edges)
    {
        out << "fid_edge\t" << edge.from_fiducial_id << '\t'
            << edge.to_fiducial_id << '\t';
        WriteKeyFrameId(out, edge.from_keyframe_id);
        out << '\t';
        WriteKeyFrameId(out, edge.to_keyframe_id);
        out << '\t' << static_cast<int>(edge.status)
            << '\t' << (edge.selected_as_branch_anchor ? 1 : 0)
            << '\t' << (edge.independent_branch ? 1 : 0)
            << '\t' << edge.kf_gap
            << '\t' << EncodeString(edge.reason) << '\n';
    }

    result.success = true;
    result.reason = "ok";
    return result;
}

PoseGraphProblemIoResult LoadPoseGraphProblem(
    const std::string& path,
    PoseGraphProblem& problem)
{
    PoseGraphProblemIoResult result;
    std::ifstream in(path);
    if (!in)
    {
        result.reason = "open_failed";
        return result;
    }

    problem = PoseGraphProblem{};
    std::string line;
    if (!std::getline(in, line) || line != "F1L_POSE_GRAPH_DUMP\t1")
    {
        result.reason = "bad_header";
        return result;
    }

    while (std::getline(in, line))
    {
        if (line.empty())
        {
            continue;
        }
        std::istringstream row(line);
        std::string tag;
        row >> tag;
        if (tag == "problem")
        {
            std::string task_type;
            std::string source;
            int rebuilt = 0;
            row >> problem.task_id >> task_type >> source;
            problem.task_type = DecodeString(task_type);
            problem.source = DecodeString(source);
            if (!ReadSubmapId(row, problem.submap_id) ||
                !ReadKeyFrameId(row, problem.target_keyframe_id) ||
                !(row >> rebuilt >> problem.partial_parent_task_id >>
                  problem.partial_retry_count))
            {
                result.reason = "bad_problem";
                return result;
            }
            problem.rebuilt_from_partial_checkpoint = rebuilt != 0;
        }
        else if (tag == "summary")
        {
            row >> problem.summary.vertices
                >> problem.summary.variable_vertices
                >> problem.summary.fixed_vertices
                >> problem.summary.hard_fiducial_vertices
                >> problem.summary.edges
                >> problem.summary.priors
                >> problem.summary.affected_non_variable_keyframes
                >> problem.summary.propagation_entries;
        }
        else if (tag == "coverage")
        {
            int max_exhausted = 0;
            int complete = 0;
            std::string reason;
            row >> problem.coverage.window_keyframes
                >> problem.coverage.control_vertices
                >> problem.coverage.max_control_span_kfs
                >> problem.coverage.max_control_span_m
                >> problem.coverage.uncovered_long_segments
                >> problem.coverage.mandatory_vertices_missing
                >> max_exhausted >> complete >> reason;
            problem.coverage.max_vertices_exhausted = max_exhausted != 0;
            problem.coverage.coverage_complete = complete != 0;
            problem.coverage.reason = DecodeString(reason);
        }
        else if (tag == "anchor")
        {
            int required = 0;
            int satisfied = 0;
            std::string reason;
            row >> required >> satisfied
                >> problem.anchor_preservation.independent_branches
                >> problem.anchor_preservation.branch_anchor_count
                >> problem.anchor_preservation.previous_fiducial_fixed_count
                >> problem.anchor_preservation.previous_fiducial_neighborhood_fixed_count
                >> problem.anchor_preservation.subdivision_candidates
                >> problem.anchor_preservation.subdivided_confirmed
                >> reason;
            problem.anchor_preservation.required = required != 0;
            problem.anchor_preservation.satisfied = satisfied != 0;
            problem.anchor_preservation.reason = DecodeString(reason);
        }
        else if (tag == "vertex")
        {
            PoseGraphVertex vertex;
            int variable = 0;
            int fixed = 0;
            int hard = 0;
            int anchor_neighborhood = 0;
            std::string reason;
            if (!ReadKeyFrameId(row, vertex.keyframe_id) ||
                !ReadSubmapId(row, vertex.submap_id) ||
                !ReadMatrix(row, vertex.initial_world_T_kf) ||
                !(row >> variable >> fixed >> hard >> anchor_neighborhood >>
                  vertex.support_count >> vertex.weight >> reason))
            {
                result.reason = "bad_vertex";
                return result;
            }
            vertex.is_variable = variable != 0;
            vertex.is_fixed = fixed != 0;
            vertex.is_hard_fiducial = hard != 0;
            vertex.is_anchor_neighborhood = anchor_neighborhood != 0;
            vertex.selection_reason = DecodeString(reason);
            problem.vertices.push_back(vertex);
        }
        else if (tag == "edge")
        {
            PoseGraphEdge edge;
            int edge_type = 0;
            if (!(row >> edge.edge_id) ||
                !ReadKeyFrameId(row, edge.from_keyframe_id) ||
                !ReadKeyFrameId(row, edge.to_keyframe_id) ||
                !(row >> edge_type) ||
                !ReadMatrix(row, edge.relative_T_from_to) ||
                !(row >> edge.weight))
            {
                result.reason = "bad_edge";
                return result;
            }
            std::vector<std::string> rest;
            std::string token;
            while (row >> token)
            {
                rest.push_back(token);
            }
            if (rest.empty())
            {
                result.reason = "bad_edge";
                return result;
            }
            if (rest.size() >= 6U)
            {
                edge.intermediate_keyframe_count = std::stoull(rest[0]);
                edge.support_keyframe_count = std::stoull(rest[1]);
                edge.support_length_m = std::stod(rest[2]);
                edge.support_density_kfs_per_m = std::stod(rest[3]);
                edge.support_rigidity_multiplier = std::stod(rest[4]);
                edge.source = DecodeString(rest[5]);
            }
            else
            {
                edge.source = DecodeString(rest[0]);
            }
            edge.edge_type = static_cast<PoseGraphEdgeType>(edge_type);
            problem.edges.push_back(edge);
        }
        else if (tag == "prior")
        {
            PoseGraphPrior prior;
            int prior_type = 0;
            int hard = 0;
            std::string source;
            if (!ReadKeyFrameId(row, prior.keyframe_id) ||
                !(row >> prior_type) ||
                !ReadMatrix(row, prior.target_world_T_kf) ||
                !(row >> prior.weight_translation >> prior.weight_rotation >>
                  hard >> source))
            {
                result.reason = "bad_prior";
                return result;
            }
            prior.prior_type = static_cast<PoseGraphPriorType>(prior_type);
            prior.hard = hard != 0;
            prior.source = DecodeString(source);
            problem.priors.push_back(prior);
        }
        else if (tag == "fixed")
        {
            RawKeyFrameId id;
            if (!ReadKeyFrameId(row, id))
            {
                result.reason = "bad_fixed";
                return result;
            }
            problem.fixed_keyframes.push_back(id);
        }
        else if (tag == "variable")
        {
            RawKeyFrameId id;
            if (!ReadKeyFrameId(row, id))
            {
                result.reason = "bad_variable";
                return result;
            }
            problem.variable_keyframes.push_back(id);
        }
        else if (tag == "affected")
        {
            RawKeyFrameId id;
            if (!ReadKeyFrameId(row, id))
            {
                result.reason = "bad_affected";
                return result;
            }
            problem.affected_non_variable_keyframes.push_back(id);
        }
        else if (tag == "debug_kf")
        {
            PoseGraphDebugKeyFramePose debug;
            int has_gt = 0;
            std::string quality;
            if (!ReadKeyFrameId(row, debug.keyframe_id) ||
                !ReadMatrix(row, debug.map_world_T_kf) ||
                !(row >> has_gt) ||
                !ReadMatrix(row, debug.gt_world_T_kf) ||
                !(row >> debug.association_dt_sec >> quality))
            {
                result.reason = "bad_debug_kf";
                return result;
            }
            debug.has_gt = has_gt != 0;
            debug.association_quality = DecodeString(quality);
            problem.debug_keyframes.push_back(debug);
        }
        else if (tag == "propagation")
        {
            PoseGraphPropagationPlanEntry entry;
            int has_b = 0;
            RawKeyFrameId b;
            int mode = 0;
            std::string reason;
            if (!ReadKeyFrameId(row, entry.affected_keyframe_id) ||
                !ReadSubmapId(row, entry.submap_id) ||
                !(row >> entry.path_id) ||
                !ReadKeyFrameId(row, entry.control_vertex_a) ||
                !(row >> has_b) ||
                !ReadKeyFrameId(row, b) ||
                !(row >> mode >> entry.segment_alpha >> entry.distance_from_a_m >>
                  entry.segment_length_m >> entry.control_span_kf_gap >> reason))
            {
                result.reason = "bad_propagation";
                return result;
            }
            if (has_b)
            {
                entry.control_vertex_b = b;
            }
            entry.mode = static_cast<PoseGraphPropagationMode>(mode);
            entry.reason = DecodeString(reason);
            problem.propagation_plan.push_back(entry);
        }
        else if (tag == "fid_edge")
        {
            FiducialConnectivityEdge edge;
            int status = 0;
            int selected = 0;
            int independent = 0;
            std::string reason;
            if (!(row >> edge.from_fiducial_id >> edge.to_fiducial_id) ||
                !ReadKeyFrameId(row, edge.from_keyframe_id) ||
                !ReadKeyFrameId(row, edge.to_keyframe_id) ||
                !(row >> status >> selected >> independent >> edge.kf_gap >> reason))
            {
                result.reason = "bad_fid_edge";
                return result;
            }
            edge.status = static_cast<FiducialConnectivityEdgeStatus>(status);
            edge.selected_as_branch_anchor = selected != 0;
            edge.independent_branch = independent != 0;
            edge.reason = DecodeString(reason);
            problem.fiducial_connectivity_edges.push_back(edge);
        }
    }

    if (problem.vertices.empty() || problem.priors.empty())
    {
        result.reason = "empty_graph";
        return result;
    }

    result.success = true;
    result.reason = "ok";
    return result;
}

}  // namespace orbslam3_multi

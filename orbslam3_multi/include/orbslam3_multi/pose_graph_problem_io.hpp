#pragma once

#include "orbslam3_multi/pose_graph_problem.hpp"

#include <string>

namespace orbslam3_multi
{

struct PoseGraphProblemIoResult
{
    bool success = false;
    std::string reason;
};

PoseGraphProblemIoResult SavePoseGraphProblem(
    const PoseGraphProblem& problem,
    const std::string& path);

PoseGraphProblemIoResult LoadPoseGraphProblem(
    const std::string& path,
    PoseGraphProblem& problem);

}  // namespace orbslam3_multi

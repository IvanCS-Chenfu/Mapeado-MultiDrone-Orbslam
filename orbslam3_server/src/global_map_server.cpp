#include "orbslam3_multi/fiducial_anchor_manager.hpp"
#include "orbslam3_multi/global_map_builder.hpp"
#include "orbslam3_multi/global_pose_store.hpp"
#include "orbslam3_multi/landmark_score_manager.hpp"
#include "orbslam3_multi/optimization_debug_exporter.hpp"
#include "orbslam3_multi/optimization_manager.hpp"
#include "orbslam3_multi/optimization_result.hpp"
#include "orbslam3_multi/pose_graph_builder.hpp"
#include "orbslam3_multi/pose_graph_problem_io.hpp"
#include "orbslam3_multi/raw_map_database.hpp"
#include "orbslam3_msgs/msg/orb_map.hpp"
#include "orbslam3_msgs/srv/get_orb_map.hpp"

#include <Eigen/Geometry>

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/exceptions.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <functional>
#include <map>
#include <mutex>
#include <limits>
#include <optional>
#include <set>
#include <sstream>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

// F1B: servidor ROS 2 minimo para validar que llegan deltas OrbMap desde
// todos los drones. No almacena raw, no publica mapa global y no ejecuta
// fiduciales/loops/optimizacion; esas responsabilidades empiezan en fases
// posteriores sobre una base ya observable.
namespace
{
using OrbMap = orbslam3_msgs::msg::OrbMap;
using RawDatabaseStats = orbslam3_multi::RawDatabaseStats;
using RawKeyFrameId = orbslam3_multi::RawKeyFrameId;
using RawJournalEntry = orbslam3_multi::RawJournalEntry;
using RawJournalEntryKind = orbslam3_multi::RawJournalEntryKind;
using RawMapDatabase = orbslam3_multi::RawMapDatabase;
using RecordedFiducialObservation = orbslam3_multi::RecordedFiducialObservation;
using RawSubmapId = orbslam3_multi::RawSubmapId;
using BodyCameraTransformConfig = orbslam3_multi::BodyCameraTransformConfig;
using FiducialAnchorManager = orbslam3_multi::FiducialAnchorManager;
using FiducialObservation = orbslam3_multi::FiducialObservation;
using FiducialOptimizationTask = orbslam3_multi::FiducialOptimizationTask;
using FiducialRevisitConfig = orbslam3_multi::FiducialRevisitConfig;
using GlobalMapBuildResult = orbslam3_multi::GlobalMapBuildResult;
using GlobalMapBuilder = orbslam3_multi::GlobalMapBuilder;
using GlobalPoseNewKeyFrameStatus = orbslam3_multi::GlobalPoseNewKeyFrameStatus;
using GlobalPoseStore = orbslam3_multi::GlobalPoseStore;
using LandmarkScoreManager = orbslam3_multi::LandmarkScoreManager;
using LandmarkScoreUpdateResult = orbslam3_multi::LandmarkScoreUpdateResult;
using OptimizationDebugExporter = orbslam3_multi::OptimizationDebugExporter;
using OptimizationApplyResult = orbslam3_multi::OptimizationApplyResult;
using OptimizationDryRunResult = orbslam3_multi::OptimizationDryRunResult;
using OptimizationManager = orbslam3_multi::OptimizationManager;
using OptimizationManagerConfig = orbslam3_multi::OptimizationManagerConfig;
using PostApplyDecision = orbslam3_multi::PostApplyDecision;
using PostApplyValidationResult = orbslam3_multi::PostApplyValidationResult;
using PoseGraphBuildResult = orbslam3_multi::PoseGraphBuildResult;
using PoseGraphBuilder = orbslam3_multi::PoseGraphBuilder;
using PoseGraphBuilderConfig = orbslam3_multi::PoseGraphBuilderConfig;
using PoseGraphProblem = orbslam3_multi::PoseGraphProblem;
using RawMapPointId = orbslam3_multi::RawMapPointId;
using GetOrbMap = orbslam3_msgs::srv::GetOrbMap;

// F1B: contadores ligeros por dron para validar recepcion multi-dron sin
// introducir todavia una base de datos raw persistente.
struct DroneRxStats
{
    uint64_t maps = 0;
    uint64_t keyframes = 0;
    uint64_t mappoints = 0;
    uint64_t last_epoch = 0;
    uint64_t last_sequence = 0;
    bool has_last_message = false;
};

struct GroundTruthSample
{
    rclcpp::Time stamp;
    Eigen::Matrix4d world_T_body = Eigen::Matrix4d::Identity();
};

struct DebugGtKeyFramePose
{
    Eigen::Matrix4d world_T_kf_gt = Eigen::Matrix4d::Identity();
    double kf_stamp_sec = 0.0;
    double gt_stamp_sec = 0.0;
    double association_dt_sec = 0.0;
    std::string association_quality;
};

struct DebugGtWindowStats
{
    uint64_t valid_kfs = 0;
    double mean_error_t = 0.0;
    double max_error_t = 0.0;
    std::map<RawKeyFrameId, double> error_t_by_keyframe;
};

struct FiducialConfig
{
    int id = 0;
    Eigen::Matrix4d world_T_fiducial = Eigen::Matrix4d::Identity();
    double radius_m = 0.0;
};
}  // namespace

class GlobalMapServer : public rclcpp::Node
{
public:
    GlobalMapServer()
        : Node("global_orb_map_server")
    {
        // F1B: el constructor fija solo contratos de adaptador ROS. Estos
        // parametros permiten lanzar el mismo nodo desde simulacion o pruebas
        // futuras sin recuperar dependencias del servidor monolitico.
        use_sim_time_ = DeclareOrReadUseSimTime();
        world_frame_ = declare_parameter<std::string>("world_frame", "world");
        namespace_base_ = declare_parameter<std::string>("namespace_base", "dron");
        n_drones_ = declare_parameter<int>("n_drones", 1);
        stats_period_s_ = declare_parameter<double>("f1b_stats_period_s", 2.0);
        rawdb_record_enabled_ = declare_parameter<bool>("rawdb_record_enabled", true);
        rawdb_record_path_ = declare_parameter<std::string>(
            "rawdb_record_path",
            "src/codex/archivos_auxiliares/repeticiones/rawdb_prueba_1.record");
        rawdb_replay_enabled_ = declare_parameter<bool>("rawdb_replay_enabled", false);
        rawdb_replay_path_ = declare_parameter<std::string>(
            "rawdb_replay_path",
            "src/codex/archivos_auxiliares/repeticiones/rawdb_prueba_1.record");
        rawdb_replay_period_sec_ = declare_parameter<double>("rawdb_replay_period_sec", 0.5);
        pose_store_debug_enabled_ = declare_parameter<bool>("pose_store_debug_enabled", false);
        pose_store_debug_anchor_after_deltas_ =
            declare_parameter<int>("pose_store_debug_anchor_after_deltas", 5);
        pose_store_debug_anchor_drone_id_ =
            declare_parameter<int>("pose_store_debug_anchor_drone_id", 1);
        pose_store_debug_anchor_epoch_ =
            declare_parameter<int>("pose_store_debug_anchor_epoch", 0);
        pose_store_debug_anchor_world_x_ =
            declare_parameter<double>("pose_store_debug_anchor_world_x", 2.0);
        pose_store_debug_anchor_world_y_ =
            declare_parameter<double>("pose_store_debug_anchor_world_y", 0.0);
        pose_store_debug_anchor_world_z_ =
            declare_parameter<double>("pose_store_debug_anchor_world_z", 0.0);
        pose_store_debug_anchor_yaw_ =
            declare_parameter<double>("pose_store_debug_anchor_yaw", 0.0);
        pose_store_debug_opt_enabled_ =
            declare_parameter<bool>("pose_store_debug_opt_enabled", false);
        pose_store_debug_opt_after_deltas_ =
            declare_parameter<int>("pose_store_debug_opt_after_deltas", 10);
        pose_store_debug_opt_kf_id_ =
            declare_parameter<int>("pose_store_debug_opt_kf_id", 0);
        pose_store_debug_opt_dx_ =
            declare_parameter<double>("pose_store_debug_opt_dx", 0.15);
        pose_store_debug_opt_dy_ =
            declare_parameter<double>("pose_store_debug_opt_dy", -0.03);
        pose_store_debug_opt_dz_ =
            declare_parameter<double>("pose_store_debug_opt_dz", 0.0);
        pose_store_debug_opt_dyaw_ =
            declare_parameter<double>("pose_store_debug_opt_dyaw", 0.05);
        fiducial_sim_enabled_ = declare_parameter<bool>("fiducial_sim_enabled", true);
        fiducial_gt_max_dt_sec_ =
            declare_parameter<double>("fiducial_gt_max_dt_sec", 1.0);
        fiducial_gt_buffer_max_samples_ =
            declare_parameter<int>("fiducial_gt_buffer_max_samples", 250);
        fiducial_ids_ = declare_parameter<std::vector<int64_t>>("fiducials.ids", {2});
        fiducial_x_ = declare_parameter<std::vector<double>>("fiducials.x", {0.0});
        fiducial_y_ = declare_parameter<std::vector<double>>("fiducials.y", {-9.0});
        fiducial_z_ = declare_parameter<std::vector<double>>("fiducials.z", {1.0});
        fiducial_yaw_ = declare_parameter<std::vector<double>>("fiducials.yaw", {0.0});
        fiducial_radius_ = declare_parameter<std::vector<double>>("fiducials.radius", {2.0});
        fiducial_revisit_error_threshold_m_ =
            declare_parameter<double>("fiducial_revisit_error_threshold_m", 0.35);
        fiducial_revisit_yaw_threshold_rad_ =
            declare_parameter<double>("fiducial_revisit_yaw_threshold_rad", 0.25);
        fiducial_revisit_rot_threshold_rad_ =
            declare_parameter<double>("fiducial_revisit_rot_threshold_rad", 0.35);
        body_T_camera_x_ = declare_parameter<double>("body_T_camera_x", 0.10);
        body_T_camera_y_ = declare_parameter<double>("body_T_camera_y", 0.03);
        body_T_camera_z_ = declare_parameter<double>("body_T_camera_z", 0.03);
        body_T_camera_roll_deg_ =
            declare_parameter<double>("body_T_camera_roll_deg", 0.0);
        body_T_camera_pitch_deg_ =
            declare_parameter<double>("body_T_camera_pitch_deg", -90.0);
        body_T_camera_yaw_deg_ =
            declare_parameter<double>("body_T_camera_yaw_deg", 90.0);
        use_camera_optical_frame_convention_ =
            declare_parameter<bool>("use_camera_optical_frame_convention", true);
        global_sparse_cloud_topic_ =
            declare_parameter<std::string>("global_sparse_cloud_topic", "/global_sparse_cloud");
        global_map_min_score_to_publish_ =
            declare_parameter<double>("global_map_min_score_to_publish", 0.0);
        global_map_publish_period_sec_ =
            declare_parameter<double>("global_map_publish_period_sec", 1.0);
        f1g_full_snapshot_enabled_ =
            declare_parameter<bool>("f1g_full_snapshot_enabled", true);
        f1g_full_snapshot_startup_delay_sec_ =
            declare_parameter<double>("f1g_full_snapshot_startup_delay_sec", 35.0);
        f1g_full_snapshot_period_sec_ =
            declare_parameter<double>("f1g_full_snapshot_period_sec", 35.0);
        f1g_debug_mark_optimized_kf_ =
            declare_parameter<bool>("f1g_debug_mark_optimized_kf", true);
        pose_graph_max_vertices_ =
            declare_parameter<int>("pose_graph_max_vertices", 24);
        pose_graph_min_vertices_ =
            declare_parameter<int>("pose_graph_min_vertices", 2);
        pose_graph_max_path_length_ =
            declare_parameter<int>("pose_graph_max_path_length", 80);
        pose_graph_anchor_stop_enabled_ =
            declare_parameter<bool>("pose_graph_anchor_stop_enabled", true);
        pose_graph_fiducial_connectivity_enabled_ =
            declare_parameter<bool>("pose_graph_fiducial_connectivity_enabled", true);
        pose_graph_branch_selection_enabled_ =
            declare_parameter<bool>("pose_graph_branch_selection_enabled", true);
        pose_graph_subdivision_candidate_radius_m_ =
            declare_parameter<double>("pose_graph_subdivision_candidate_radius_m", 2.0);
        pose_graph_include_temporal_edges_ =
            declare_parameter<bool>("pose_graph_include_temporal_edges", true);
        pose_graph_temporal_edge_weight_ =
            declare_parameter<double>("pose_graph_temporal_edge_weight", 25.0);
        pose_graph_temporal_edge_weight_sparse_ =
            declare_parameter<double>("pose_graph_temporal_edge_weight_sparse", 10.0);
        pose_graph_fiducial_hard_weight_ =
            declare_parameter<double>("pose_graph_fiducial_hard_weight", 1000000.0);
        pose_graph_fiducial_target_translation_weight_ =
            declare_parameter<double>(
                "pose_graph_fiducial_target_translation_weight",
                5000.0);
        pose_graph_fiducial_target_rotation_weight_ =
            declare_parameter<double>(
                "pose_graph_fiducial_target_rotation_weight",
                2500.0);
        pose_graph_current_pose_soft_weight_ =
            declare_parameter<double>("pose_graph_current_pose_soft_weight", 5.0);
        pose_graph_fiducial_neighborhood_radius_m_ =
            declare_parameter<double>("pose_graph_fiducial_neighborhood_radius_m", 4.0);
        pose_graph_fiducial_neighborhood_radius_kfs_ =
            declare_parameter<int>("pose_graph_fiducial_neighborhood_radius_kfs", 3);
        pose_graph_max_temporal_edge_kf_gap_ =
            declare_parameter<int>("pose_graph_max_temporal_edge_kf_gap", 15);
        pose_graph_max_temporal_edge_length_m_ =
            declare_parameter<double>("pose_graph_max_temporal_edge_length_m", 4.0);
        pose_graph_corner_yaw_threshold_rad_ =
            declare_parameter<double>("pose_graph_corner_yaw_threshold_rad", 0.5235987756);
        f1i_debug_force_task_enabled_ =
            declare_parameter<bool>("f1i_debug_force_task_enabled", false);
        f1i_debug_task_dx_ =
            declare_parameter<double>("f1i_debug_task_dx", 0.75);
        f1i_debug_task_dy_ =
            declare_parameter<double>("f1i_debug_task_dy", 0.0);
        f1i_debug_task_dz_ =
            declare_parameter<double>("f1i_debug_task_dz", 0.0);
        f1i_debug_task_dyaw_ =
            declare_parameter<double>("f1i_debug_task_dyaw", 0.20);
        f1j_dryrun_enabled_ =
            declare_parameter<bool>("f1j_dryrun_enabled", true);
        f1j_dryrun_min_improvement_ratio_ =
            declare_parameter<double>("f1j_dryrun_min_improvement_ratio", 0.05);
        f1j_dryrun_partial_min_improvement_ratio_ =
            declare_parameter<double>("f1j_dryrun_partial_min_improvement_ratio", 0.70);
        f1j_dryrun_max_allowed_delta_t_ =
            declare_parameter<double>("f1j_dryrun_max_allowed_delta_t", 30.0);
        f1j_dryrun_partial_max_allowed_delta_t_ =
            declare_parameter<double>("f1j_dryrun_partial_max_allowed_delta_t", 35.0);
        f1j_dryrun_max_allowed_delta_yaw_ =
            declare_parameter<double>("f1j_dryrun_max_allowed_delta_yaw", 1.2);
        f1j_dryrun_max_final_error_t_ =
            declare_parameter<double>("f1j_dryrun_max_final_error_t", 0.35);
        f1j_dryrun_require_cost_decrease_ =
            declare_parameter<bool>("f1j_dryrun_require_cost_decrease", true);
        f1j_solver_step_fraction_ =
            declare_parameter<double>("f1j_solver_step_fraction", 0.95);
        f1j_export_debug_plot_ =
            declare_parameter<bool>("f1j_export_debug_plot", false);
        f1j_debug_output_dir_ =
            declare_parameter<std::string>(
                "f1j_debug_output_dir",
                "src/codex/archivos_auxiliares");
        f1k_apply_enabled_ =
            declare_parameter<bool>("f1k_apply_enabled", true);
        f1l_validation_enabled_ =
            declare_parameter<bool>("f1l_validation_enabled", true);
        f1l_partial_apply_enabled_ =
            declare_parameter<bool>("f1l_partial_apply_enabled", true);
        f1l_max_partial_retries_ =
            declare_parameter<int>("f1l_max_partial_retries", 3);
        f1l_debug_force_reject_once_ =
            declare_parameter<bool>("f1l_debug_force_reject_once", false);
        f1l_debug_force_reject_task_id_ =
            declare_parameter<int>("f1l_debug_force_reject_task_id", -1);
        f1l_post_apply_internal_broken_edge_t_ =
            declare_parameter<double>("f1l_post_apply_internal_broken_edge_t", 2.50);
        f1l_post_apply_internal_max_growth_t_ =
            declare_parameter<double>("f1l_post_apply_internal_max_growth_t", 1.50);
        f1l_post_apply_fiducial_absurd_error_t_ =
            declare_parameter<double>("f1l_post_apply_fiducial_absurd_error_t", 10.0);
        f1l_gt_kf_debug_enabled_ =
            declare_parameter<bool>("f1l_gt_kf_debug_enabled", false);
        f1l_gt_kf_debug_max_dt_sec_ =
            declare_parameter<double>("f1l_gt_kf_debug_max_dt_sec", fiducial_gt_max_dt_sec_);
        f1l_debug_animation_enabled_ =
            declare_parameter<bool>("f1l_debug_animation_enabled", false);
        f1l_debug_animation_output_dir_ =
            declare_parameter<std::string>(
                "f1l_debug_animation_output_dir",
                "src/codex/archivos_auxiliares/html");
        f1l_graph_dump_enabled_ =
            declare_parameter<bool>("f1l_graph_dump_enabled", false);
        f1l_graph_dump_output_dir_ =
            declare_parameter<std::string>(
                "f1l_graph_dump_output_dir",
                "src/codex/archivos_auxiliares/repeticiones");

        // F1B: el servidor minimo debe sobrevivir a configuraciones invalidas
        // para que la simulacion deje evidencia en logs en vez de abortar antes
        // de suscribirse a los topics.
        if (n_drones_ < 1)
        {
            RCLCPP_WARN(
                get_logger(),
                "[F1B-SERVER-PARAMS] invalid_n_drones=%d forcing=1",
                n_drones_);
            n_drones_ = 1;
        }

        // F1B: limitar el periodo evita spam extremo si el launch pasa un valor
        // accidentalmente bajo, pero conserva observabilidad frecuente.
        if (stats_period_s_ < 0.2)
        {
            stats_period_s_ = 0.2;
        }
        if (rawdb_replay_period_sec_ < 0.01)
        {
            rawdb_replay_period_sec_ = 0.01;
        }
        if (pose_store_debug_anchor_after_deltas_ < 1)
        {
            pose_store_debug_anchor_after_deltas_ = 1;
        }
        if (pose_store_debug_opt_after_deltas_ < pose_store_debug_anchor_after_deltas_)
        {
            pose_store_debug_opt_after_deltas_ = pose_store_debug_anchor_after_deltas_ + 1;
        }
        if (pose_store_debug_anchor_drone_id_ < 1)
        {
            pose_store_debug_anchor_drone_id_ = 1;
        }
        if (pose_store_debug_anchor_epoch_ < 0)
        {
            pose_store_debug_anchor_epoch_ = 0;
        }
        if (pose_store_debug_opt_kf_id_ < 0)
        {
            pose_store_debug_opt_kf_id_ = 0;
        }
        if (fiducial_gt_max_dt_sec_ < 0.0)
        {
            fiducial_gt_max_dt_sec_ = 0.0;
        }
        if (fiducial_gt_buffer_max_samples_ < 1)
        {
            fiducial_gt_buffer_max_samples_ = 1;
        }
        if (f1l_gt_kf_debug_max_dt_sec_ < 0.0)
        {
            f1l_gt_kf_debug_max_dt_sec_ = 0.0;
        }
        if (fiducial_revisit_error_threshold_m_ < 0.0)
        {
            fiducial_revisit_error_threshold_m_ = 0.0;
        }
        if (fiducial_revisit_yaw_threshold_rad_ < 0.0)
        {
            fiducial_revisit_yaw_threshold_rad_ = 0.0;
        }
        if (fiducial_revisit_rot_threshold_rad_ < 0.0)
        {
            fiducial_revisit_rot_threshold_rad_ = 0.0;
        }
        if (global_map_min_score_to_publish_ < 0.0)
        {
            global_map_min_score_to_publish_ = 0.0;
        }
        if (global_map_min_score_to_publish_ > 1.0)
        {
            global_map_min_score_to_publish_ = 1.0;
        }
        if (global_map_publish_period_sec_ < 0.2)
        {
            global_map_publish_period_sec_ = 0.2;
        }
        if (f1g_full_snapshot_startup_delay_sec_ < 1.0)
        {
            f1g_full_snapshot_startup_delay_sec_ = 1.0;
        }
        if (f1g_full_snapshot_period_sec_ < 5.0)
        {
            f1g_full_snapshot_period_sec_ = 5.0;
        }
        if (pose_graph_max_vertices_ < 2)
        {
            pose_graph_max_vertices_ = 2;
        }
        if (pose_graph_min_vertices_ < 2)
        {
            pose_graph_min_vertices_ = 2;
        }
        if (pose_graph_min_vertices_ > pose_graph_max_vertices_)
        {
            pose_graph_min_vertices_ = pose_graph_max_vertices_;
        }
        if (pose_graph_max_path_length_ < 1)
        {
            pose_graph_max_path_length_ = 1;
        }
        if (pose_graph_fiducial_neighborhood_radius_kfs_ < 0)
        {
            pose_graph_fiducial_neighborhood_radius_kfs_ = 0;
        }
        if (pose_graph_fiducial_neighborhood_radius_m_ < 0.0)
        {
            pose_graph_fiducial_neighborhood_radius_m_ = 0.0;
        }
        if (pose_graph_max_temporal_edge_kf_gap_ < 1)
        {
            pose_graph_max_temporal_edge_kf_gap_ = 1;
        }
        if (pose_graph_max_temporal_edge_length_m_ < 0.0)
        {
            pose_graph_max_temporal_edge_length_m_ = 0.0;
        }
        if (pose_graph_corner_yaw_threshold_rad_ < 0.0)
        {
            pose_graph_corner_yaw_threshold_rad_ = 0.0;
        }
        if (pose_graph_temporal_edge_weight_ < 0.0)
        {
            pose_graph_temporal_edge_weight_ = 0.0;
        }
        if (pose_graph_subdivision_candidate_radius_m_ < 0.0)
        {
            pose_graph_subdivision_candidate_radius_m_ = 0.0;
        }
        if (pose_graph_temporal_edge_weight_sparse_ < 0.0)
        {
            pose_graph_temporal_edge_weight_sparse_ = 0.0;
        }
        if (pose_graph_fiducial_hard_weight_ < 0.0)
        {
            pose_graph_fiducial_hard_weight_ = 0.0;
        }
        if (pose_graph_fiducial_target_translation_weight_ < 0.0)
        {
            pose_graph_fiducial_target_translation_weight_ = 0.0;
        }
        if (pose_graph_fiducial_target_rotation_weight_ < 0.0)
        {
            pose_graph_fiducial_target_rotation_weight_ = 0.0;
        }
        if (pose_graph_current_pose_soft_weight_ < 0.0)
        {
            pose_graph_current_pose_soft_weight_ = 0.0;
        }
        if (f1j_dryrun_min_improvement_ratio_ < 0.0)
        {
            f1j_dryrun_min_improvement_ratio_ = 0.0;
        }
        if (f1j_dryrun_partial_min_improvement_ratio_ < 0.0)
        {
            f1j_dryrun_partial_min_improvement_ratio_ = 0.0;
        }
        if (f1j_dryrun_max_allowed_delta_t_ < 0.0)
        {
            f1j_dryrun_max_allowed_delta_t_ = 0.0;
        }
        if (f1j_dryrun_partial_max_allowed_delta_t_ < 0.0)
        {
            f1j_dryrun_partial_max_allowed_delta_t_ = 0.0;
        }
        if (f1j_dryrun_max_allowed_delta_yaw_ < 0.0)
        {
            f1j_dryrun_max_allowed_delta_yaw_ = 0.0;
        }
        if (f1j_dryrun_max_final_error_t_ < 0.0)
        {
            f1j_dryrun_max_final_error_t_ = 0.0;
        }
        if (f1j_solver_step_fraction_ < 0.0)
        {
            f1j_solver_step_fraction_ = 0.0;
        }
        if (f1j_solver_step_fraction_ > 1.0)
        {
            f1j_solver_step_fraction_ = 1.0;
        }
        if (f1l_debug_force_reject_task_id_ < -1)
        {
            f1l_debug_force_reject_task_id_ = -1;
        }
        if (f1l_max_partial_retries_ < 0)
        {
            f1l_max_partial_retries_ = 0;
        }
        if (f1l_post_apply_internal_broken_edge_t_ < 0.0)
        {
            f1l_post_apply_internal_broken_edge_t_ = 0.0;
        }
        if (f1l_post_apply_internal_max_growth_t_ < 0.0)
        {
            f1l_post_apply_internal_max_growth_t_ = 0.0;
        }
        if (f1l_post_apply_fiducial_absurd_error_t_ < 0.0)
        {
            f1l_post_apply_fiducial_absurd_error_t_ = 0.0;
        }

        LoadFiducialConfig();
        ConfigureFiducialRevisit();
        ConfigureBodyCameraTransform();
        ConfigurePoseGraphBuilder();
        ConfigureOptimizationManager();
        global_sparse_cloud_pub_ =
            create_publisher<sensor_msgs::msg::PointCloud2>(
                global_sparse_cloud_topic_,
                // F1H-hotfix: RViz2 solo necesita la ultima nube sparse. Con
                // una cola mayor podia recibir nubes antiguas despues de una
                // nueva y verse un parpadeo entre estados consecutivos.
                rclcpp::QoS(rclcpp::KeepLast(1)).reliable());

        RCLCPP_WARN(
            get_logger(),
            "[F1B-SERVER-INIT] node=global_orb_map_server mode=minimal_rx_only");

        RCLCPP_WARN(
            get_logger(),
            "[F1B-SERVER-PARAMS] use_sim_time=%s world_frame=%s namespace_base=%s drones=%d stats_period_s=%.3f",
            use_sim_time_ ? "true" : "false",
            world_frame_.c_str(),
            namespace_base_.c_str(),
            n_drones_,
            stats_period_s_);

        RCLCPP_WARN(
            get_logger(),
            "[F1C-RAWDB-INIT] record_enabled=%s record_path=%s replay_enabled=%s replay_path=%s replay_period_sec=%.3f",
            rawdb_record_enabled_ ? "true" : "false",
            rawdb_record_path_.c_str(),
            rawdb_replay_enabled_ ? "true" : "false",
            rawdb_replay_path_.c_str(),
            rawdb_replay_period_sec_);

        LogPoseStoreInit();
        LogFiducialInit();
        LogScoreInit();
        RCLCPP_WARN(
            get_logger(),
            "[F1F-GLOBALMAP-PUBLISHER] topic=%s frame_id=%s min_score_to_publish=%.3f publish_period_sec=%.3f",
            global_sparse_cloud_topic_.c_str(),
            world_frame_.c_str(),
            global_map_min_score_to_publish_,
            global_map_publish_period_sec_);

        // F1C: en modo replay no nos suscribimos a wrappers reales. El origen
        // de verdad pasa a ser el journal guardado para que la prueba sea
        // repetible sin Gazebo ni ORB-SLAM3 produciendo datos nuevos.
        if (rawdb_replay_enabled_)
        {
            LoadReplayDataset();
        }
        else
        {
            CreateOrbMapSubscriptions();
            if (f1g_full_snapshot_enabled_)
            {
                CreateFullSnapshotClients();
            }
            if (fiducial_sim_enabled_)
            {
                CreateGroundTruthSubscriptions();
            }
        }

        stats_timer_ = create_wall_timer(
            std::chrono::duration<double>(stats_period_s_),
            std::bind(&GlobalMapServer::PublishStatsLog, this));

        // F1F/F1H-hotfix: el timer republica la ultima nube ya construida para
        // que RViz2 pueda engancharse tarde sin alternar una reconstruccion
        // antigua con la actual recibida por delta/snapshot.
        global_map_publish_timer_ = create_wall_timer(
            std::chrono::duration<double>(global_map_publish_period_sec_),
            [this]()
            {
                PublishGlobalSparseCloud("timer");
            });

        // F1C: el guardado periodico evita depender solo del destructor cuando
        // `run_simulation.sh` cierre el launch. La DB sigue siendo raw: guardar
        // no transforma ni optimiza ningun dato.
        if (rawdb_record_enabled_ && !rawdb_replay_enabled_)
        {
            rawdb_save_timer_ = create_wall_timer(
                std::chrono::seconds(5),
                [this]()
                {
                    SaveRawDatabase("periodic");
                });
        }

        if (!rawdb_replay_enabled_ && f1g_full_snapshot_enabled_)
        {
            full_snapshot_startup_timer_ = create_wall_timer(
                std::chrono::duration<double>(f1g_full_snapshot_startup_delay_sec_),
                [this]()
                {
                    if (full_snapshot_startup_timer_)
                    {
                        full_snapshot_startup_timer_->cancel();
                    }
                    RequestFullSnapshots("startup_resync");
                    full_snapshot_periodic_timer_ = create_wall_timer(
                        std::chrono::duration<double>(f1g_full_snapshot_period_sec_),
                        [this]()
                        {
                            RequestFullSnapshots("periodic_resync");
                        });
                });
        }
    }

    ~GlobalMapServer() override
    {
        if (rawdb_record_enabled_ && !rawdb_replay_enabled_)
        {
            SaveRawDatabase("shutdown");
        }
    }

private:
    // F1B: lee `use_sim_time` de forma tolerante porque ROS 2 puede declararlo
    // desde launch antes de que este nodo lo declare manualmente.
    bool DeclareOrReadUseSimTime()
    {
        bool value = false;

        try
        {
            value = declare_parameter<bool>("use_sim_time", false);
        }
        catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&)
        {
            get_parameter("use_sim_time", value);
        }

        return value;
    }

    // F1B: construye el namespace publico esperado por `simulacion_dron`
    // (`/dron_1`, `/dron_2`, ...). La regla evita codificar topics sueltos en
    // varios puntos del servidor.
    std::string DroneNamespace(uint32_t drone_id) const
    {
        return "/" + namespace_base_ + "_" + std::to_string(drone_id);
    }

    RawSubmapId DebugPoseStoreSubmapId() const
    {
        return RawSubmapId{
            static_cast<uint32_t>(pose_store_debug_anchor_drone_id_),
            static_cast<uint64_t>(pose_store_debug_anchor_epoch_)};
    }

    Eigen::Matrix4d PlanarTransform(double x, double y, double z, double yaw_rad) const
    {
        Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
        transform.block<3, 3>(0, 0) =
            Eigen::AngleAxisd(yaw_rad, Eigen::Vector3d::UnitZ()).toRotationMatrix();
        transform(0, 3) = x;
        transform(1, 3) = y;
        transform(2, 3) = z;
        return transform;
    }

    double YawFromTransform(const Eigen::Matrix4d& transform) const
    {
        return std::atan2(transform(1, 0), transform(0, 0));
    }

    double NormalizeAngle(double angle) const
    {
        const double pi = std::acos(-1.0);
        while (angle > pi)
        {
            angle -= 2.0 * pi;
        }
        while (angle < -pi)
        {
            angle += 2.0 * pi;
        }
        return angle;
    }

    double StampToSeconds(const builtin_interfaces::msg::Time& stamp) const
    {
        return static_cast<double>(stamp.sec) + 1e-9 * static_cast<double>(stamp.nanosec);
    }

    bool PoseMsgToMatrix(const geometry_msgs::msg::Pose& pose, Eigen::Matrix4d& out) const
    {
        const Eigen::Quaterniond q_raw(
            pose.orientation.w,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z);
        if (!std::isfinite(q_raw.w()) ||
            !std::isfinite(q_raw.x()) ||
            !std::isfinite(q_raw.y()) ||
            !std::isfinite(q_raw.z()) ||
            q_raw.norm() < 1e-9)
        {
            return false;
        }

        const Eigen::Quaterniond q = q_raw.normalized();
        out = Eigen::Matrix4d::Identity();
        out.block<3, 3>(0, 0) = q.toRotationMatrix();
        out(0, 3) = pose.position.x;
        out(1, 3) = pose.position.y;
        out(2, 3) = pose.position.z;
        return out.allFinite();
    }

    RecordedFiducialObservation ToRecordedObservation(
        const FiducialObservation& observation) const
    {
        const Eigen::Matrix3d rotation =
            observation.world_T_body_fiducial.block<3, 3>(0, 0);
        const Eigen::Quaterniond q(rotation);

        RecordedFiducialObservation recorded;
        recorded.arrival_id = observation.arrival_id;
        recorded.drone_id = observation.drone_id;
        recorded.map_epoch = observation.map_epoch;
        recorded.local_keyframe_id = observation.local_keyframe_id;
        recorded.global_keyframe_id = observation.global_keyframe_id;
        recorded.fiducial_id = observation.fiducial_id;
        recorded.world_x = observation.world_T_body_fiducial(0, 3);
        recorded.world_y = observation.world_T_body_fiducial(1, 3);
        recorded.world_z = observation.world_T_body_fiducial(2, 3);
        recorded.world_qx = q.x();
        recorded.world_qy = q.y();
        recorded.world_qz = q.z();
        recorded.world_qw = q.w();
        recorded.keyframe_stamp_sec = observation.keyframe_stamp_sec;
        recorded.gt_stamp_sec = observation.gt_stamp_sec;
        recorded.association_dt_sec = observation.association_dt_sec;
        recorded.distance_to_fiducial_m = observation.distance_to_fiducial_m;
        recorded.source = observation.source;
        return recorded;
    }

    FiducialObservation FromRecordedObservation(
        const RecordedFiducialObservation& recorded,
        const std::string& source_override) const
    {
        const Eigen::Quaterniond q_raw(
            recorded.world_qw,
            recorded.world_qx,
            recorded.world_qy,
            recorded.world_qz);
        const Eigen::Quaterniond q =
            q_raw.norm() > 1e-9 ? q_raw.normalized() : Eigen::Quaterniond::Identity();

        FiducialObservation observation;
        observation.arrival_id = recorded.arrival_id;
        observation.drone_id = recorded.drone_id;
        observation.map_epoch = recorded.map_epoch;
        observation.local_keyframe_id = recorded.local_keyframe_id;
        observation.global_keyframe_id = recorded.global_keyframe_id;
        observation.fiducial_id = recorded.fiducial_id;
        observation.world_T_body_fiducial = Eigen::Matrix4d::Identity();
        observation.world_T_body_fiducial.block<3, 3>(0, 0) = q.toRotationMatrix();
        observation.world_T_body_fiducial(0, 3) = recorded.world_x;
        observation.world_T_body_fiducial(1, 3) = recorded.world_y;
        observation.world_T_body_fiducial(2, 3) = recorded.world_z;
        observation.keyframe_stamp_sec = recorded.keyframe_stamp_sec;
        observation.gt_stamp_sec = recorded.gt_stamp_sec;
        observation.association_dt_sec = recorded.association_dt_sec;
        observation.distance_to_fiducial_m = recorded.distance_to_fiducial_m;
        observation.source = source_override;
        return observation;
    }

    void ConfigureBodyCameraTransform()
    {
        // F1F-hotfix: la observacion fiducial live/replay persiste la pose GT
        // del cuerpo del dron. El backend de poses necesita convertirla a la
        // camara de ORB-SLAM3 antes de calcular `world_T_local`.
        BodyCameraTransformConfig config;
        config.x = body_T_camera_x_;
        config.y = body_T_camera_y_;
        config.z = body_T_camera_z_;
        config.roll_deg = body_T_camera_roll_deg_;
        config.pitch_deg = body_T_camera_pitch_deg_;
        config.yaw_deg = body_T_camera_yaw_deg_;
        config.use_camera_optical_frame_convention =
            use_camera_optical_frame_convention_;
        config.source = "launch";
        pose_store_.ConfigureBodyCameraTransform(config);

        const Eigen::Matrix4d body_T_camera =
            pose_store_.GetBodyCameraTransform();
        RCLCPP_WARN(
            get_logger(),
            "[F1F-BODY-CAMERA-CONFIG] source=launch use_optical=%s body_T_camera_t=(%.3f,%.3f,%.3f) rpy_deg=(%.3f,%.3f,%.3f) matrix_t=(%.3f,%.3f,%.3f)",
            config.use_camera_optical_frame_convention ? "true" : "false",
            config.x,
            config.y,
            config.z,
            config.roll_deg,
            config.pitch_deg,
            config.yaw_deg,
            body_T_camera(0, 3),
            body_T_camera(1, 3),
            body_T_camera(2, 3));
    }

    void LoadFiducialConfig()
    {
        fiducials_.clear();
        const size_t count = fiducial_ids_.size();
        for (size_t i = 0; i < count; ++i)
        {
            if (i >= fiducial_x_.size() ||
                i >= fiducial_y_.size() ||
                i >= fiducial_z_.size() ||
                i >= fiducial_yaw_.size() ||
                i >= fiducial_radius_.size())
            {
                RCLCPP_WARN(
                    get_logger(),
                    "[F1E-FID-CONFIG] status=skip index=%zu reason=vector_size_mismatch",
                    i);
                continue;
            }

            FiducialConfig config;
            config.id = static_cast<int>(fiducial_ids_[i]);
            config.world_T_fiducial =
                PlanarTransform(fiducial_x_[i], fiducial_y_[i], fiducial_z_[i], fiducial_yaw_[i]);
            config.radius_m = std::max(0.0, fiducial_radius_[i]);
            fiducials_.push_back(config);
        }
    }

    void ConfigureFiducialRevisit()
    {
        // F1H: estos umbrales solo clasifican una segunda observacion fiducial
        // como OK o como tarea pendiente. No mueven anchors ni ejecutan solver.
        FiducialRevisitConfig config;
        config.error_threshold_m = fiducial_revisit_error_threshold_m_;
        config.yaw_threshold_rad = fiducial_revisit_yaw_threshold_rad_;
        config.rot_threshold_rad = fiducial_revisit_rot_threshold_rad_;
        config.source = "launch";
        fiducial_anchor_manager_.ConfigureRevisitThresholds(config);

        RCLCPP_WARN(
            get_logger(),
            "[F1H-FID-REVISIT-CONFIG] threshold_t=%.3f threshold_yaw=%.3f threshold_rot=%.3f source=%s",
            config.error_threshold_m,
            config.yaw_threshold_rad,
            config.rot_threshold_rad,
            config.source.c_str());
    }

    void ConfigurePoseGraphBuilder()
    {
        // F1I: configura solo la construccion del problema temporal. Estos
        // parametros no habilitan solver ni aplicacion de poses.
        PoseGraphBuilderConfig config;
        config.max_vertices = static_cast<uint64_t>(pose_graph_max_vertices_);
        config.min_vertices = static_cast<uint64_t>(pose_graph_min_vertices_);
        config.max_path_length = static_cast<uint64_t>(pose_graph_max_path_length_);
        config.anchor_stop_enabled = pose_graph_anchor_stop_enabled_;
        config.fiducial_connectivity_enabled =
            pose_graph_fiducial_connectivity_enabled_;
        config.branch_selection_enabled = pose_graph_branch_selection_enabled_;
        config.subdivision_candidate_radius_m =
            pose_graph_subdivision_candidate_radius_m_;
        config.fiducial_neighborhood_radius_m =
            pose_graph_fiducial_neighborhood_radius_m_;
        config.include_temporal_edges = pose_graph_include_temporal_edges_;
        config.temporal_edge_weight = pose_graph_temporal_edge_weight_;
        config.temporal_edge_weight_sparse = pose_graph_temporal_edge_weight_sparse_;
        config.fiducial_hard_weight = pose_graph_fiducial_hard_weight_;
        config.fiducial_target_translation_weight =
            pose_graph_fiducial_target_translation_weight_;
        config.fiducial_target_rotation_weight =
            pose_graph_fiducial_target_rotation_weight_;
        config.current_pose_soft_weight = pose_graph_current_pose_soft_weight_;
        config.fiducial_neighborhood_radius_kfs = static_cast<uint64_t>(
            pose_graph_fiducial_neighborhood_radius_kfs_);
        config.max_temporal_edge_kf_gap = static_cast<uint64_t>(
            pose_graph_max_temporal_edge_kf_gap_);
        config.max_temporal_edge_length_m = pose_graph_max_temporal_edge_length_m_;
        config.corner_yaw_threshold_rad = pose_graph_corner_yaw_threshold_rad_;
        pose_graph_builder_.Configure(config);

        RCLCPP_WARN(
            get_logger(),
            "[F1I-GRAPH-BUILDER-CONFIG] max_vertices=%llu min_vertices=%llu max_path_length=%llu anchor_stop=%s fid_connectivity=%s branch_selection=%s subdivision_radius=%.3f fiducial_neighborhood_radius_kfs=%llu max_temporal_edge_kf_gap=%llu max_temporal_edge_length_m=%.3f corner_yaw_threshold_rad=%.3f temporal_edges=%s weights_temporal=%.3f sparse=%.3f fid_hard=%.3f fid_target_t=%.3f fid_target_rot=%.3f current_soft=%.3f debug_force=%s",
            static_cast<unsigned long long>(config.max_vertices),
            static_cast<unsigned long long>(config.min_vertices),
            static_cast<unsigned long long>(config.max_path_length),
            config.anchor_stop_enabled ? "true" : "false",
            config.fiducial_connectivity_enabled ? "true" : "false",
            config.branch_selection_enabled ? "true" : "false",
            config.subdivision_candidate_radius_m,
            static_cast<unsigned long long>(config.fiducial_neighborhood_radius_kfs),
            static_cast<unsigned long long>(config.max_temporal_edge_kf_gap),
            config.max_temporal_edge_length_m,
            config.corner_yaw_threshold_rad,
            config.include_temporal_edges ? "true" : "false",
            config.temporal_edge_weight,
            config.temporal_edge_weight_sparse,
            config.fiducial_hard_weight,
            config.fiducial_target_translation_weight,
            config.fiducial_target_rotation_weight,
            config.current_pose_soft_weight,
            f1i_debug_force_task_enabled_ ? "true" : "false");
        // Nombre de marcador legacy: la vecindad fiducial se construye en 1I.
        // Se conserva `F1L-*` en logs para no romper reducciones históricas.
        RCLCPP_WARN(
            get_logger(),
            "[F1L-FIDUCIAL-NEIGHBORHOOD-CONFIG] radius_m=%.3f legacy_radius_kfs=%llu policy=metric_pose_distance",
            config.fiducial_neighborhood_radius_m,
            static_cast<unsigned long long>(config.fiducial_neighborhood_radius_kfs));
    }

    void ConfigureOptimizationManager()
    {
        // F1J: el dry-run calcula propuestas en memoria. La exportacion SVG es
        // opcional y desactivada por defecto; no forma parte del camino normal
        // ni se lee de vuelta desde el servidor.
        OptimizationManagerConfig config;
        config.dryrun_min_improvement_ratio = f1j_dryrun_min_improvement_ratio_;
        config.dryrun_partial_min_improvement_ratio =
            f1j_dryrun_partial_min_improvement_ratio_;
        config.dryrun_max_allowed_delta_t = f1j_dryrun_max_allowed_delta_t_;
        config.dryrun_partial_max_allowed_delta_t =
            f1j_dryrun_partial_max_allowed_delta_t_;
        config.dryrun_max_allowed_delta_yaw = f1j_dryrun_max_allowed_delta_yaw_;
        config.dryrun_max_final_error_t = f1j_dryrun_max_final_error_t_;
        config.dryrun_require_cost_decrease = f1j_dryrun_require_cost_decrease_;
        config.solver_step_fraction = f1j_solver_step_fraction_;
        config.post_apply_internal_broken_edge_t =
            f1l_post_apply_internal_broken_edge_t_;
        config.post_apply_internal_max_growth_t =
            f1l_post_apply_internal_max_growth_t_;
        config.post_apply_fiducial_absurd_error_t =
            f1l_post_apply_fiducial_absurd_error_t_;
        optimization_manager_.Configure(config);

        RCLCPP_WARN(
            get_logger(),
            "[F1J-OPT-CONFIG] enabled=%s min_improvement=%.3f partial_min_improvement=%.3f max_delta_t=%.3f partial_max_delta_t=%.3f max_delta_yaw=%.3f max_final_error_t=%.3f require_cost_decrease=%s step=%.3f export_debug_plot=%s output_dir=%s",
            f1j_dryrun_enabled_ ? "true" : "false",
            config.dryrun_min_improvement_ratio,
            config.dryrun_partial_min_improvement_ratio,
            config.dryrun_max_allowed_delta_t,
            config.dryrun_partial_max_allowed_delta_t,
            config.dryrun_max_allowed_delta_yaw,
            config.dryrun_max_final_error_t,
            config.dryrun_require_cost_decrease ? "true" : "false",
            config.solver_step_fraction,
            f1j_export_debug_plot_ ? "true" : "false",
            f1j_debug_output_dir_.c_str());
        RCLCPP_WARN(
            get_logger(),
            "[F1K-OPT-APPLY-CONFIG] enabled=%s policy=useful_or_f1l_partial partial_candidate_policy=f1l_controlled",
            f1k_apply_enabled_ ? "true" : "false");
        RCLCPP_WARN(
            get_logger(),
            "[F1L-POST-APPLY-CONFIG] validation_enabled=%s partial_apply_enabled=%s max_partial_retries=%d force_reject_once=%s force_reject_task_id=%d internal_broken_edge_t=%.3f internal_max_growth_t=%.3f fiducial_absurd_error_t=%.3f",
            f1l_validation_enabled_ ? "true" : "false",
            f1l_partial_apply_enabled_ ? "true" : "false",
            f1l_max_partial_retries_,
            f1l_debug_force_reject_once_ ? "true" : "false",
            f1l_debug_force_reject_task_id_,
            f1l_post_apply_internal_broken_edge_t_,
            f1l_post_apply_internal_max_growth_t_,
            f1l_post_apply_fiducial_absurd_error_t_);
        RCLCPP_WARN(
            get_logger(),
            "[F1L-GT-DEBUG-CONFIG] enabled=%s max_dt=%.3f source=GAZEBO_GT_DEBUG usage=metrics_only",
            f1l_gt_kf_debug_enabled_ ? "true" : "false",
            f1l_gt_kf_debug_max_dt_sec_);
        // Nombre de parámetro legacy: este HTML visualiza el dry-run de 1J.
        // 1L solo lo compara contra logs/RViz2 si hay diagnóstico post-apply.
        RCLCPP_WARN(
            get_logger(),
            "[F1L-DEBUG-ANIMATION-CONFIG] enabled=%s output_dir=%s format=html_animation usage=diagnostic_only",
            f1l_debug_animation_enabled_ ? "true" : "false",
            f1l_debug_animation_output_dir_.c_str());
    }

    void LogFiducialInit()
    {
        RCLCPP_WARN(
            get_logger(),
            "[F1E-FID-INIT] sim_enabled=%s fiducials=%zu gt_max_dt=%.3f gt_buffer=%d",
            fiducial_sim_enabled_ ? "true" : "false",
            fiducials_.size(),
            fiducial_gt_max_dt_sec_,
            fiducial_gt_buffer_max_samples_);

        for (const auto& fiducial : fiducials_)
        {
            RCLCPP_WARN(
                get_logger(),
                "[F1E-FID-CONFIG] id=%d world_t=(%.3f,%.3f,%.3f) yaw=%.3f radius=%.3f",
                fiducial.id,
                fiducial.world_T_fiducial(0, 3),
                fiducial.world_T_fiducial(1, 3),
                fiducial.world_T_fiducial(2, 3),
                YawFromTransform(fiducial.world_T_fiducial),
                fiducial.radius_m);
        }
    }

    void LogPoseStoreInit()
    {
        const auto stats = pose_store_.GetPoseStoreStats();
        RCLCPP_WARN(
            get_logger(),
            "[F1D-POSESTORE-INIT] anchors=%llu world_poses=%llu optimized_kfs=%llu propagated_kfs=%llu rebased_kfs=%llu corrections=%llu hard_fiducial_kfs=%llu debug_enabled=%s",
            static_cast<unsigned long long>(stats.anchored_submaps),
            static_cast<unsigned long long>(stats.keyframe_world_poses),
            static_cast<unsigned long long>(stats.optimized_keyframes),
            static_cast<unsigned long long>(stats.propagated_keyframes),
            static_cast<unsigned long long>(stats.rebased_keyframes),
            static_cast<unsigned long long>(stats.submap_corrections),
            static_cast<unsigned long long>(stats.hard_fiducial_keyframes),
            pose_store_debug_enabled_ ? "true" : "false");
        LogPoseStoreStats("init");
    }

    void LogPoseStoreStats(const std::string& reason)
    {
        const auto stats = pose_store_.GetPoseStoreStats();
        RCLCPP_WARN(
            get_logger(),
            "[F1D-POSESTORE-STATS] reason=%s anchors=%llu world_poses=%llu optimized_kfs=%llu propagated_kfs=%llu rebased_kfs=%llu corrections=%llu hard_fiducial_kfs=%llu",
            reason.c_str(),
            static_cast<unsigned long long>(stats.anchored_submaps),
            static_cast<unsigned long long>(stats.keyframe_world_poses),
            static_cast<unsigned long long>(stats.optimized_keyframes),
            static_cast<unsigned long long>(stats.propagated_keyframes),
            static_cast<unsigned long long>(stats.rebased_keyframes),
            static_cast<unsigned long long>(stats.submap_corrections),
            static_cast<unsigned long long>(stats.hard_fiducial_keyframes));

        RCLCPP_WARN(
            get_logger(),
            "[F1D-SERVER-POSESTORE-STATS] reason=%s debug_enabled=%s anchor_done=%s opt_done=%s anchors=%llu world_poses=%llu optimized_kfs=%llu propagated_kfs=%llu rebased_kfs=%llu corrections=%llu hard_fiducial_kfs=%llu",
            reason.c_str(),
            pose_store_debug_enabled_ ? "true" : "false",
            pose_store_debug_anchor_done_ ? "true" : "false",
            pose_store_debug_opt_done_ ? "true" : "false",
            static_cast<unsigned long long>(stats.anchored_submaps),
            static_cast<unsigned long long>(stats.keyframe_world_poses),
            static_cast<unsigned long long>(stats.optimized_keyframes),
            static_cast<unsigned long long>(stats.propagated_keyframes),
            static_cast<unsigned long long>(stats.rebased_keyframes),
            static_cast<unsigned long long>(stats.submap_corrections),
            static_cast<unsigned long long>(stats.hard_fiducial_keyframes));

        const auto fid_stats = fiducial_anchor_manager_.GetStats();
        RCLCPP_WARN(
            get_logger(),
            "[F1E-POSESTORE-STATS] reason=%s anchors=%llu world_poses=%llu hard_fiducial_kfs=%llu fid_observations=%llu fid_accepted=%llu fid_replay=%llu",
            reason.c_str(),
            static_cast<unsigned long long>(stats.anchored_submaps),
            static_cast<unsigned long long>(stats.keyframe_world_poses),
            static_cast<unsigned long long>(stats.hard_fiducial_keyframes),
            static_cast<unsigned long long>(fid_stats.observations),
            static_cast<unsigned long long>(fid_stats.accepted),
            static_cast<unsigned long long>(fid_stats.replay_observations));
    }

    void LogScoreInit()
    {
        const auto stats = score_manager_.GetStats();
        RCLCPP_WARN(
            get_logger(),
            "[F1F-SCORE-INIT] tracked_points=%llu policy=orbslam_raw_quality observations_weight=0.55 found_ratio_weight=0.35 descriptor_weight=0.10",
            static_cast<unsigned long long>(stats.tracked_points));
    }

    void UpdateScoresFromMap(const OrbMap& map, uint64_t arrival_id)
    {
        // F1F: el servidor informa eventos raw al manager, pero no calcula la
        // formula de score. La politica vive en LandmarkScoreManager.
        uint64_t created = 0;
        uint64_t updated = 0;
        uint64_t marked_bad = 0;
        bool has_first_created = false;
        LandmarkScoreUpdateResult first_created;

        for (const auto& mappoint : map.mappoints)
        {
            const RawMapPointId id{map.drone_id, map.map_epoch, mappoint.id};
            const auto update = score_manager_.ApplyOrbSlamQuality(id, mappoint);
            if (update.created)
            {
                ++created;
                if (!has_first_created)
                {
                    first_created = update;
                    has_first_created = true;
                }
            }
            else
            {
                ++updated;
            }
            if (update.record.is_bad)
            {
                ++marked_bad;
            }
        }

        if (has_first_created)
        {
            const auto& record = first_created.record;
            RCLCPP_WARN(
                get_logger(),
                "[F1F-SCORE-INIT] arrival_id=%llu mp=%llu drone_id=%u epoch=%llu obs=%u found_ratio=%.3f is_bad=%s descriptor_valid=%s score=%.3f reason=%s new_points=%llu",
                static_cast<unsigned long long>(arrival_id),
                static_cast<unsigned long long>(record.id.local_mp_id),
                record.id.drone_id,
                static_cast<unsigned long long>(record.id.map_epoch),
                record.observations,
                record.found_ratio,
                record.is_bad ? "true" : "false",
                record.descriptor_valid ? "true" : "false",
                record.score,
                orbslam3_multi::ToString(record.last_event),
                static_cast<unsigned long long>(created));
        }

        const auto stats = score_manager_.GetStats();
        RCLCPP_WARN(
            get_logger(),
            "[F1F-SCORE-UPDATE-ORBSLAM] arrival_id=%llu drone_id=%u epoch=%llu delta_mps=%zu created=%llu updated=%llu marked_bad=%llu tracked_points=%llu",
            static_cast<unsigned long long>(arrival_id),
            map.drone_id,
            static_cast<unsigned long long>(map.map_epoch),
            map.mappoints.size(),
            static_cast<unsigned long long>(created),
            static_cast<unsigned long long>(updated),
            static_cast<unsigned long long>(marked_bad),
            static_cast<unsigned long long>(stats.tracked_points));
        LogScoreStats("orbslam_raw_update");
    }

    void LogScoreStats(const std::string& reason)
    {
        const auto stats = score_manager_.GetStats();
        RCLCPP_WARN(
            get_logger(),
            "[F1F-SCORE-STATS] reason=%s tracked_points=%llu score_min=%.3f score_mean=%.3f score_max=%.3f bad_points=%llu",
            reason.c_str(),
            static_cast<unsigned long long>(stats.tracked_points),
            stats.score_min,
            stats.score_mean,
            stats.score_max,
            static_cast<unsigned long long>(stats.bad_points));
    }

    sensor_msgs::msg::PointCloud2 BuildPointCloud2(
        const std::vector<orbslam3_multi::GlobalSparsePoint>& points) const
    {
        sensor_msgs::msg::PointCloud2 cloud;
        cloud.header.stamp = now();
        cloud.header.frame_id = world_frame_;
        cloud.height = 1;
        cloud.width = static_cast<uint32_t>(points.size());
        cloud.is_bigendian = false;
        cloud.is_dense = false;

        auto add_field =
            [&cloud](const std::string& name,
                     uint32_t offset,
                     uint8_t datatype)
            {
                sensor_msgs::msg::PointField field;
                field.name = name;
                field.offset = offset;
                field.datatype = datatype;
                field.count = 1;
                cloud.fields.push_back(field);
            };

        add_field("x", 0, sensor_msgs::msg::PointField::FLOAT32);
        add_field("y", 4, sensor_msgs::msg::PointField::FLOAT32);
        add_field("z", 8, sensor_msgs::msg::PointField::FLOAT32);
        add_field("score", 12, sensor_msgs::msg::PointField::FLOAT32);
        add_field("drone_id", 16, sensor_msgs::msg::PointField::UINT32);
        add_field("map_epoch", 20, sensor_msgs::msg::PointField::UINT32);

        cloud.point_step = 24;
        cloud.row_step = cloud.point_step * cloud.width;
        cloud.data.resize(static_cast<size_t>(cloud.row_step));

        for (size_t i = 0; i < points.size(); ++i)
        {
            const auto& point = points[i];
            const size_t base = i * cloud.point_step;
            const float x = static_cast<float>(point.x);
            const float y = static_cast<float>(point.y);
            const float z = static_cast<float>(point.z);
            const float score = point.score;
            const uint32_t drone_id = point.drone_id;
            const uint32_t map_epoch =
                point.map_epoch > std::numeric_limits<uint32_t>::max()
                    ? std::numeric_limits<uint32_t>::max()
                    : static_cast<uint32_t>(point.map_epoch);

            std::memcpy(&cloud.data[base + 0], &x, sizeof(float));
            std::memcpy(&cloud.data[base + 4], &y, sizeof(float));
            std::memcpy(&cloud.data[base + 8], &z, sizeof(float));
            std::memcpy(&cloud.data[base + 12], &score, sizeof(float));
            std::memcpy(&cloud.data[base + 16], &drone_id, sizeof(uint32_t));
            std::memcpy(&cloud.data[base + 20], &map_epoch, sizeof(uint32_t));
        }

        return cloud;
    }

    void PublishGlobalSparseCloud(const std::string& reason)
    {
        if (reason == "timer")
        {
            RepublishLastGlobalSparseCloud(reason);
            return;
        }

        // F1F: GlobalMapBuilder decide que puntos son publicables. El servidor
        // solo convierte el resultado a PointCloud2 y deja evidencia de frame,
        // topic y campos para RViz2.
        const GlobalMapBuildResult build =
            global_map_builder_.Build(
                raw_db_,
                pose_store_,
                score_manager_,
                static_cast<float>(global_map_min_score_to_publish_));

        RCLCPP_WARN(
            get_logger(),
            "[F1F-GLOBALMAP-BUILD] reason=%s anchored_submaps=%llu skipped_unanchored=%llu raw_points=%llu candidate_points=%llu returned_points=%llu",
            reason.c_str(),
            static_cast<unsigned long long>(build.stats.anchored_submaps),
            static_cast<unsigned long long>(build.stats.skipped_unanchored_submaps),
            static_cast<unsigned long long>(build.stats.raw_points),
            static_cast<unsigned long long>(build.stats.candidate_points),
            static_cast<unsigned long long>(build.stats.returned_points));

        if (build.stats.skipped_unanchored_submaps > 0)
        {
            RCLCPP_WARN(
                get_logger(),
                "[F1F-GLOBALMAP-SKIP-UNANCHORED] reason=%s skipped_unanchored_submaps=%llu",
                reason.c_str(),
                static_cast<unsigned long long>(build.stats.skipped_unanchored_submaps));
        }

        RCLCPP_WARN(
            get_logger(),
            "[F1F-GLOBALMAP-POINT-STATS] reason=%s points=%llu server_corrected_points=%llu score_min=%.3f score_mean=%.3f score_max=%.3f bad_skipped=%llu invalid_pose_skipped=%llu below_score_skipped=%llu",
            reason.c_str(),
            static_cast<unsigned long long>(build.stats.returned_points),
            static_cast<unsigned long long>(build.stats.server_corrected_points),
            build.stats.score_min,
            build.stats.score_mean,
            build.stats.score_max,
            static_cast<unsigned long long>(build.stats.bad_skipped),
            static_cast<unsigned long long>(build.stats.invalid_pose_skipped),
            static_cast<unsigned long long>(build.stats.below_score_skipped));

        auto cloud = BuildPointCloud2(build.points);
        last_global_sparse_cloud_ = cloud;
        has_last_global_sparse_cloud_ = true;
        global_sparse_cloud_pub_->publish(cloud);
        RCLCPP_WARN(
            get_logger(),
            "[F1F-GLOBALMAP-PUBLISH] reason=%s topic=%s frame_id=%s points_from_backend=%zu points_published=%u min_score_to_publish=%.3f score_field=true drone_id_field=true map_epoch_field=true",
            reason.c_str(),
            global_sparse_cloud_topic_.c_str(),
            cloud.header.frame_id.c_str(),
            build.points.size(),
            cloud.width,
            global_map_min_score_to_publish_);
    }

    void RepublishLastGlobalSparseCloud(const std::string& reason)
    {
        // F1H-hotfix: el timer no debe reconstruir desde `raw_db_` porque puede
        // intercalarse visualmente con una nube recien publicada por delta o
        // snapshot. Reutiliza el ultimo `PointCloud2` validado y solo refresca
        // el stamp para RViz2.
        if (!has_last_global_sparse_cloud_)
        {
            RCLCPP_WARN(
                get_logger(),
                "[F1F-GLOBALMAP-REPUBLISH-SKIP] reason=%s cause=no_cached_cloud topic=%s",
                reason.c_str(),
                global_sparse_cloud_topic_.c_str());
            return;
        }

        auto cloud = last_global_sparse_cloud_;
        cloud.header.stamp = now();
        global_sparse_cloud_pub_->publish(cloud);
        RCLCPP_WARN(
            get_logger(),
            "[F1F-GLOBALMAP-REPUBLISH] reason=%s topic=%s frame_id=%s cached_points=%u",
            reason.c_str(),
            global_sparse_cloud_topic_.c_str(),
            cloud.header.frame_id.c_str(),
            cloud.width);
    }

    void CreateFullSnapshotClients()
    {
        // F1G: cada wrapper ya ofrece `orbslam/get_full_map`. El servidor solo
        // crea clientes y pide snapshots de forma acotada para resincronizar raw
        // sin bloquear el hilo principal de ROS.
        full_snapshot_clients_.resize(static_cast<size_t>(n_drones_) + 1U);
        full_snapshot_pending_.assign(static_cast<size_t>(n_drones_) + 1U, false);
        for (uint32_t drone_id = 1; drone_id <= static_cast<uint32_t>(n_drones_); ++drone_id)
        {
            const std::string service = DroneNamespace(drone_id) + "/orbslam/get_full_map";
            full_snapshot_clients_[drone_id] = create_client<GetOrbMap>(service);
            RCLCPP_WARN(
                get_logger(),
                "[F1G-FULL-SNAPSHOT-CLIENT-READY] drone_id=%u service=%s startup_delay_sec=%.3f period_sec=%.3f",
                drone_id,
                service.c_str(),
                f1g_full_snapshot_startup_delay_sec_,
                f1g_full_snapshot_period_sec_);
        }
    }

    void RequestFullSnapshots(const std::string& reason)
    {
        for (uint32_t drone_id = 1; drone_id <= static_cast<uint32_t>(n_drones_); ++drone_id)
        {
            RequestFullSnapshot(drone_id, reason);
        }
    }

    void RequestFullSnapshot(uint32_t drone_id, const std::string& reason)
    {
        if (drone_id >= full_snapshot_clients_.size() || !full_snapshot_clients_[drone_id])
        {
            return;
        }

        const std::string service = DroneNamespace(drone_id) + "/orbslam/get_full_map";
        if (full_snapshot_pending_[drone_id])
        {
            RCLCPP_WARN(
                get_logger(),
                "[F1G-FULL-SNAPSHOT-REQUEST] drone_id=%u service=%s reason=%s status=skip_pending",
                drone_id,
                service.c_str(),
                reason.c_str());
            return;
        }
        if (!full_snapshot_clients_[drone_id]->service_is_ready())
        {
            RCLCPP_WARN(
                get_logger(),
                "[F1G-FULL-SNAPSHOT-TIMEOUT] drone_id=%u service=%s reason=%s status=service_not_ready",
                drone_id,
                service.c_str(),
                reason.c_str());
            return;
        }

        full_snapshot_pending_[drone_id] = true;
        auto request = std::make_shared<GetOrbMap::Request>();
        RCLCPP_WARN(
            get_logger(),
            "[F1G-FULL-SNAPSHOT-REQUEST] drone_id=%u service=%s reason=%s status=sent",
            drone_id,
            service.c_str(),
            reason.c_str());
        full_snapshot_clients_[drone_id]->async_send_request(
            request,
            [this, drone_id, service, reason](
                rclcpp::Client<GetOrbMap>::SharedFuture future)
            {
                OnFullSnapshotResponse(drone_id, service, reason, future);
            });
    }

    void OnFullSnapshotResponse(uint32_t requested_drone_id,
                                const std::string& service,
                                const std::string& reason,
                                rclcpp::Client<GetOrbMap>::SharedFuture future)
    {
        if (requested_drone_id < full_snapshot_pending_.size())
        {
            full_snapshot_pending_[requested_drone_id] = false;
        }

        try
        {
            const auto response = future.get();
            if (!response)
            {
                RCLCPP_ERROR(
                    get_logger(),
                    "[F1G-FULL-SNAPSHOT-RX] requested_drone_id=%u service=%s success=false error=null_response",
                    requested_drone_id,
                    service.c_str());
                return;
            }

            OrbMap snapshot = response->map;
            if (snapshot.drone_id == 0 &&
                snapshot.keyframes.empty() &&
                snapshot.mappoints.empty())
            {
                RCLCPP_WARN(
                    get_logger(),
                    "[F1G-FULL-SNAPSHOT-RX] requested_drone_id=%u service=%s success=false error=empty_or_uninitialized_snapshot reason=%s",
                    requested_drone_id,
                    service.c_str(),
                    reason.c_str());
                return;
            }

            const uint64_t arrival_id = rawdb_next_arrival_id_++;
            RCLCPP_WARN(
                get_logger(),
                "[F1G-FULL-SNAPSHOT-RX] requested_drone_id=%u drone_id=%u epoch=%llu seq=%llu kfs=%zu mps=%zu service=%s reason=%s",
                requested_drone_id,
                snapshot.drone_id,
                static_cast<unsigned long long>(snapshot.map_epoch),
                static_cast<unsigned long long>(snapshot.map_sequence),
                snapshot.keyframes.size(),
                snapshot.mappoints.size(),
                service.c_str(),
                reason.c_str());
            RCLCPP_WARN(
                get_logger(),
                "[F1G-FULL-SNAPSHOT-ARRIVAL] arrival_id=%llu drone_id=%u epoch=%llu seq=%llu kfs=%zu mps=%zu",
                static_cast<unsigned long long>(arrival_id),
                snapshot.drone_id,
                static_cast<unsigned long long>(snapshot.map_epoch),
                static_cast<unsigned long long>(snapshot.map_sequence),
                snapshot.keyframes.size(),
                snapshot.mappoints.size());

            MaybeSetF1GDebugOptimizedKeyFrame(snapshot, "live_full_snapshot");
            const auto insert_result = raw_db_.InsertFullSnapshot(arrival_id, snapshot);
            ProcessFullSnapshotAfterInsert(snapshot, arrival_id, insert_result, "live_full_snapshot");
        }
        catch (const std::exception& ex)
        {
            RCLCPP_ERROR(
                get_logger(),
                "[F1G-FULL-SNAPSHOT-RX] requested_drone_id=%u service=%s success=false error=%s",
                requested_drone_id,
                service.c_str(),
                ex.what());
        }
    }

    bool MaybeSetF1GDebugOptimizedKeyFrame(const OrbMap& map, const std::string& reason)
    {
        if (!f1g_debug_mark_optimized_kf_ || f1g_debug_optimized_kf_done_)
        {
            return false;
        }

        for (const auto& keyframe : map.keyframes)
        {
            const RawKeyFrameId keyframe_id{map.drone_id, map.map_epoch, keyframe.id};
            const auto current_world_pose = pose_store_.GetWorldPose(keyframe_id);
            if (!current_world_pose)
            {
                continue;
            }

            const auto result =
                pose_store_.SetOptimizedKeyFramePose(
                    keyframe_id,
                    current_world_pose.value(),
                    raw_db_,
                    "F1G_DEBUG_OPTIMIZED_POSE");
            if (!result.success)
            {
                RCLCPP_WARN(
                    get_logger(),
                    "[F1G-DEBUG-OPTIMIZED-KF-SET] status=failed drone_id=%u epoch=%llu kf=%llu reason=%s trigger=%s",
                    keyframe_id.drone_id,
                    static_cast<unsigned long long>(keyframe_id.map_epoch),
                    static_cast<unsigned long long>(keyframe_id.local_kf_id),
                    result.reason.c_str(),
                    reason.c_str());
                return false;
            }

            f1g_debug_optimized_kf_done_ = true;
            RCLCPP_WARN(
                get_logger(),
                "[F1G-DEBUG-OPTIMIZED-KF-SET] status=applied drone_id=%u epoch=%llu kf=%llu source=F1G_DEBUG_OPTIMIZED_POSE trigger=%s",
                keyframe_id.drone_id,
                static_cast<unsigned long long>(keyframe_id.map_epoch),
                static_cast<unsigned long long>(keyframe_id.local_kf_id),
                reason.c_str());
            return true;
        }

        RCLCPP_WARN(
            get_logger(),
            "[F1G-DEBUG-OPTIMIZED-KF-SET] status=waiting reason=no_world_pose_in_snapshot drone_id=%u epoch=%llu trigger=%s",
            map.drone_id,
            static_cast<unsigned long long>(map.map_epoch),
            reason.c_str());
        return false;
    }

    void ProcessFullSnapshotAfterInsert(const OrbMap& map,
                                        uint64_t arrival_id,
                                        const orbslam3_multi::RawInsertResult& insert_result,
                                        const std::string& source)
    {
        RCLCPP_WARN(
            get_logger(),
            "[F1G-RAWDB-INSERT-FULL] arrival_id=%llu drone_id=%u epoch=%llu new_kfs=%llu updated_kfs=%llu new_mps=%llu updated_mps=%llu bad_kfs=%llu bad_mps=%llu raw_pose_changed=%zu large_raw_pose_changed=%llu",
            static_cast<unsigned long long>(arrival_id),
            map.drone_id,
            static_cast<unsigned long long>(map.map_epoch),
            static_cast<unsigned long long>(insert_result.new_keyframes),
            static_cast<unsigned long long>(insert_result.updated_keyframes),
            static_cast<unsigned long long>(insert_result.new_mappoints),
            static_cast<unsigned long long>(insert_result.updated_mappoints),
            static_cast<unsigned long long>(insert_result.bad_keyframes),
            static_cast<unsigned long long>(insert_result.bad_mappoints),
            insert_result.raw_pose_changed_keyframes.size(),
            static_cast<unsigned long long>(insert_result.large_raw_pose_changed_keyframes));

        RCLCPP_WARN(
            get_logger(),
            "[F1G-SNAPSHOT-RECONCILE] arrival_id=%llu drone_id=%u epoch=%llu policy=insert_update_no_absent_delete new_kfs=%llu updated_kfs=%llu new_mps=%llu updated_mps=%llu",
            static_cast<unsigned long long>(arrival_id),
            map.drone_id,
            static_cast<unsigned long long>(map.map_epoch),
            static_cast<unsigned long long>(insert_result.new_keyframes),
            static_cast<unsigned long long>(insert_result.updated_keyframes),
            static_cast<unsigned long long>(insert_result.new_mappoints),
            static_cast<unsigned long long>(insert_result.updated_mappoints));

        for (const auto& change : insert_result.raw_pose_changed_keyframes)
        {
            RCLCPP_WARN(
                get_logger(),
                "[F1G-RAW-POSE-CHANGED] arrival_id=%llu drone_id=%u epoch=%llu kf=%llu delta_t=%.6f delta_yaw=%.6f large=%s source=%s",
                static_cast<unsigned long long>(arrival_id),
                change.keyframe_id.drone_id,
                static_cast<unsigned long long>(change.keyframe_id.map_epoch),
                static_cast<unsigned long long>(change.keyframe_id.local_kf_id),
                change.delta_translation_m,
                change.delta_yaw_rad,
                change.large_change ? "true" : "false",
                source.c_str());
        }

        const auto reconcile =
            pose_store_.ReconcileAfterRawIngestResult(
                insert_result,
                raw_db_,
                "F1G_FULL_SNAPSHOT_RECONCILE");
        for (const auto& event : reconcile.events)
        {
            if (event.action == "rebase_anchor")
            {
                RCLCPP_WARN(
                    get_logger(),
                    "[F1G-POSESTORE-REBASE-ANCHOR] drone_id=%u epoch=%llu kf=%llu delta_t=%.6f delta_yaw=%.6f reason=%s",
                    event.keyframe_id.drone_id,
                    static_cast<unsigned long long>(event.keyframe_id.map_epoch),
                    static_cast<unsigned long long>(event.keyframe_id.local_kf_id),
                    event.raw_delta_translation_m,
                    event.raw_delta_yaw_rad,
                    event.reason.c_str());
            }
            else if (event.action == "keep_optimized")
            {
                RCLCPP_WARN(
                    get_logger(),
                    "[F1G-POSESTORE-KEEP-OPTIMIZED] drone_id=%u epoch=%llu kf=%llu delta_t=%.6f delta_yaw=%.6f reason=%s",
                    event.keyframe_id.drone_id,
                    static_cast<unsigned long long>(event.keyframe_id.map_epoch),
                    static_cast<unsigned long long>(event.keyframe_id.local_kf_id),
                    event.raw_delta_translation_m,
                    event.raw_delta_yaw_rad,
                    event.reason.c_str());
                RCLCPP_WARN(
                    get_logger(),
                    "[F1K-POSESTORE-KEEP-SERVER-OPTIMIZED] drone_id=%u epoch=%llu kf=%llu raw_pose_changed=true action=recompute_correction_keep_world_pose delta_t=%.6f delta_yaw=%.6f",
                    event.keyframe_id.drone_id,
                    static_cast<unsigned long long>(event.keyframe_id.map_epoch),
                    static_cast<unsigned long long>(event.keyframe_id.local_kf_id),
                    event.raw_delta_translation_m,
                    event.raw_delta_yaw_rad);
                RCLCPP_WARN(
                    get_logger(),
                    "[F1K-POSESTORE-RECOMPUTE-CORRECTION-AFTER-RAW-CHANGE] drone_id=%u epoch=%llu kf=%llu source=full_snapshot",
                    event.keyframe_id.drone_id,
                    static_cast<unsigned long long>(event.keyframe_id.map_epoch),
                    static_cast<unsigned long long>(event.keyframe_id.local_kf_id));
            }
            else if (event.action == "inherit_last_correction")
            {
                RCLCPP_WARN(
                    get_logger(),
                    "[F1K-POSESTORE-KEEP-FUTURE-INHERIT] drone_id=%u epoch=%llu kf=%llu raw_pose_changed=true action=apply_last_correction_after_raw_change delta_t=%.6f delta_yaw=%.6f reason=%s",
                    event.keyframe_id.drone_id,
                    static_cast<unsigned long long>(event.keyframe_id.map_epoch),
                    static_cast<unsigned long long>(event.keyframe_id.local_kf_id),
                    event.raw_delta_translation_m,
                    event.raw_delta_yaw_rad,
                    event.reason.c_str());
            }
        }

        if (!reconcile.optimized_submaps_affected.empty())
        {
            RCLCPP_WARN(
                get_logger(),
                "[F1G-SNAPSHOT-AFFECTS-OPTIMIZED-KFS] arrival_id=%llu affected_submaps=%zu optimized_kept=%llu",
                static_cast<unsigned long long>(arrival_id),
                reconcile.optimized_submaps_affected.size(),
                static_cast<unsigned long long>(reconcile.optimized_kept));
        }

        RCLCPP_WARN(
            get_logger(),
            "[F1G-POSESTORE-RECONCILE-SUMMARY] arrival_id=%llu raw_pose_changed=%llu anchor_rebased=%llu optimized_kept=%llu no_world_pose=%llu failed=%llu affected_optimized_submaps=%zu",
            static_cast<unsigned long long>(arrival_id),
            static_cast<unsigned long long>(reconcile.raw_pose_changed),
            static_cast<unsigned long long>(reconcile.anchor_rebased),
            static_cast<unsigned long long>(reconcile.optimized_kept),
            static_cast<unsigned long long>(reconcile.no_world_pose),
            static_cast<unsigned long long>(reconcile.failed),
            reconcile.optimized_submaps_affected.size());

        LogRawInsert("F1C-RAWDB-INSERT-FULL", arrival_id, insert_result.stats);
        UpdateScoresFromMap(map, arrival_id);
        RegisterF1EKeyFramesFromMap(map);
        PublishGlobalSparseCloud("full_snapshot");
        RCLCPP_WARN(
            get_logger(),
            "[F1G-GLOBALMAP-REBUILD-AFTER-SNAPSHOT] arrival_id=%llu drone_id=%u epoch=%llu source=%s",
            static_cast<unsigned long long>(arrival_id),
            map.drone_id,
            static_cast<unsigned long long>(map.map_epoch),
            source.c_str());
    }

    // F1B: crea un subscriber por dron a `orbslam/orb_map_delta`.
    // Entrada: `n_drones_` y `namespace_base_`.
    // Efecto: deja activa la recepcion multi-dron y loggea cada topic usado.
    void CreateOrbMapSubscriptions()
    {
        // F1B: recorremos IDs esperados 1..n para mantener estable la relacion
        // entre topic suscrito y `subscribed_drone_id` usado luego en los logs.
        for (uint32_t drone_id = 1; drone_id <= static_cast<uint32_t>(n_drones_); ++drone_id)
        {
            const std::string topic = DroneNamespace(drone_id) + "/orbslam/orb_map_delta";

            // Callback ROS creado en F1B y ampliado en F1C: mantiene los logs
            // de recepción y además inserta cada delta en RawMapDatabase.
            auto sub = create_subscription<OrbMap>(
                topic,
                rclcpp::QoS(rclcpp::KeepLast(20)),
                [this, drone_id](const OrbMap::SharedPtr msg)
                {
                    OnOrbMapDelta(msg, drone_id);
                });

            orb_map_delta_subs_.push_back(sub);

            RCLCPP_WARN(
                get_logger(),
                "[F1B-SERVER-SUBSCRIBED] drone_id=%u topic=%s",
                drone_id,
                topic.c_str());
        }
    }

    void CreateGroundTruthSubscriptions()
    {
        // F1E: GT solo entra por esta ruta para simular detecciones fiduciales.
        // No se usa para mapa, loops, fused landmarks ni pose final.
        for (uint32_t drone_id = 1; drone_id <= static_cast<uint32_t>(n_drones_); ++drone_id)
        {
            const std::string topic = DroneNamespace(drone_id) + "/sensor/GT/pose";
            auto sub = create_subscription<geometry_msgs::msg::PoseStamped>(
                topic,
                rclcpp::QoS(rclcpp::KeepLast(50)),
                [this, drone_id](const geometry_msgs::msg::PoseStamped::SharedPtr msg)
                {
                    OnGroundTruthPose(msg, drone_id);
                });
            gt_pose_subs_.push_back(sub);

            RCLCPP_WARN(
                get_logger(),
                "[F1E-FID-GT-SUBSCRIBED] drone_id=%u topic=%s",
                drone_id,
                topic.c_str());
        }
    }

    void OnGroundTruthPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg,
                           uint32_t drone_id)
    {
        if (!msg)
        {
            return;
        }

        Eigen::Matrix4d world_T_body = Eigen::Matrix4d::Identity();
        if (!PoseMsgToMatrix(msg->pose, world_T_body))
        {
            RCLCPP_WARN(
                get_logger(),
                "[F1E-FID-CANDIDATE-GT] drone_id=%u status=rejected reason=invalid_gt_pose",
                drone_id);
            return;
        }

        GroundTruthSample sample;
        sample.stamp = rclcpp::Time(msg->header.stamp);
        sample.world_T_body = world_T_body;

        auto& buffer = gt_buffers_[drone_id];
        buffer.push_back(sample);
        while (buffer.size() > static_cast<size_t>(fiducial_gt_buffer_max_samples_))
        {
            buffer.erase(buffer.begin());
        }
    }

    bool FindBestGroundTruth(uint32_t drone_id,
                             const rclcpp::Time& keyframe_stamp,
                             GroundTruthSample& best_sample,
                             double& best_dt_sec,
                             std::string& source,
                             double max_dt_sec = -1.0) const
    {
        const auto buffer_it = gt_buffers_.find(drone_id);
        if (buffer_it == gt_buffers_.end() || buffer_it->second.empty())
        {
            return false;
        }

        const auto& buffer = buffer_it->second;
        if (keyframe_stamp.nanoseconds() <= 0)
        {
            best_sample = buffer.back();
            best_dt_sec = 0.0;
            source = "SIM_GT_NEAREST_POSE";
            return true;
        }

        bool found = false;
        int64_t best_abs_dt_ns = 0;
        for (const auto& sample : buffer)
        {
            const int64_t dt_ns =
                std::llabs((sample.stamp - keyframe_stamp).nanoseconds());
            if (!found || dt_ns < best_abs_dt_ns)
            {
                found = true;
                best_abs_dt_ns = dt_ns;
                best_sample = sample;
            }
        }

        best_dt_sec = static_cast<double>(best_abs_dt_ns) * 1e-9;
        source = "SIM_GT_TEMPORAL_ASSOCIATION";
        const double allowed_dt =
            max_dt_sec >= 0.0 ? max_dt_sec : fiducial_gt_max_dt_sec_;
        return found && best_dt_sec <= allowed_dt;
    }

    void AssociateGtDebugForDelta(const OrbMap& map, uint64_t arrival_id)
    {
        if (!f1l_gt_kf_debug_enabled_ || rawdb_replay_enabled_)
        {
            return;
        }

        for (const auto& keyframe : map.keyframes)
        {
            const RawKeyFrameId keyframe_id{
                map.drone_id,
                map.map_epoch,
                keyframe.id};
            const rclcpp::Time keyframe_stamp(keyframe.stamp);
            GroundTruthSample gt_sample;
            double dt_sec = 0.0;
            std::string source;
            if (!FindBestGroundTruth(
                    map.drone_id,
                    keyframe_stamp,
                    gt_sample,
                    dt_sec,
                    source,
                    f1l_gt_kf_debug_max_dt_sec_))
            {
                if (f1l_gt_debug_missing_logged_.insert(keyframe_id).second)
                {
                    RCLCPP_WARN(
                        get_logger(),
                        "[F1L-GT-KF-ASSOC] arrival_id=%llu drone_id=%u epoch=%llu kf=%llu kf_stamp=%.6f gt_stamp=0.000000 dt=-1.000000 valid=false quality=INVALID source=GAZEBO_GT_DEBUG reason=no_gt_within_threshold",
                        static_cast<unsigned long long>(arrival_id),
                        keyframe_id.drone_id,
                        static_cast<unsigned long long>(keyframe_id.map_epoch),
                        static_cast<unsigned long long>(keyframe_id.local_kf_id),
                        StampToSeconds(keyframe.stamp));
                }
                continue;
            }

            DebugGtKeyFramePose debug_pose;
            debug_pose.world_T_kf_gt =
                pose_store_.TransformBodyPoseToCameraPose(gt_sample.world_T_body);
            debug_pose.kf_stamp_sec = StampToSeconds(keyframe.stamp);
            debug_pose.gt_stamp_sec = gt_sample.stamp.seconds();
            debug_pose.association_dt_sec = dt_sec;
            debug_pose.association_quality =
                dt_sec <= 0.5 * f1l_gt_kf_debug_max_dt_sec_ ? "OK" : "LOW";
            f1l_gt_keyframe_store_[keyframe_id] = debug_pose;

            // 1L: este GT se registra como metrica externa por KF. No se pasa
            // al solver ni cambia pesos/vertices; solo permite diagnosticar si
            // RViz2 muestra deriva o dano colateral tras el apply.
            RCLCPP_WARN(
                get_logger(),
                "[F1L-GT-KF-ASSOC] arrival_id=%llu drone_id=%u epoch=%llu kf=%llu kf_stamp=%.6f gt_stamp=%.6f dt=%.6f valid=true quality=%s source=GAZEBO_GT_DEBUG gt_camera_t=(%.3f,%.3f,%.3f)",
                static_cast<unsigned long long>(arrival_id),
                keyframe_id.drone_id,
                static_cast<unsigned long long>(keyframe_id.map_epoch),
                static_cast<unsigned long long>(keyframe_id.local_kf_id),
                debug_pose.kf_stamp_sec,
                debug_pose.gt_stamp_sec,
                debug_pose.association_dt_sec,
                debug_pose.association_quality.c_str(),
                debug_pose.world_T_kf_gt(0, 3),
                debug_pose.world_T_kf_gt(1, 3),
                debug_pose.world_T_kf_gt(2, 3));
        }
    }

    DebugGtWindowStats LogGtWindowErrors(uint64_t task_id,
                                         const std::vector<RawKeyFrameId>& keyframes,
                                         bool after_apply) const
    {
        DebugGtWindowStats stats;
        if (!f1l_gt_kf_debug_enabled_)
        {
            return stats;
        }

        std::set<RawKeyFrameId> unique_keyframes(keyframes.begin(), keyframes.end());
        double sum_error = 0.0;
        for (const auto& keyframe_id : unique_keyframes)
        {
            const auto gt_it = f1l_gt_keyframe_store_.find(keyframe_id);
            if (gt_it == f1l_gt_keyframe_store_.end())
            {
                continue;
            }
            const auto world_pose = pose_store_.GetWorldPose(keyframe_id);
            if (!world_pose.has_value())
            {
                continue;
            }

            const Eigen::Vector3d delta =
                world_pose.value().block<3, 1>(0, 3) -
                gt_it->second.world_T_kf_gt.block<3, 1>(0, 3);
            const double error_t = delta.norm();
            const double error_yaw = std::abs(NormalizeAngle(
                YawFromTransform(world_pose.value()) -
                YawFromTransform(gt_it->second.world_T_kf_gt)));
            stats.error_t_by_keyframe[keyframe_id] = error_t;
            ++stats.valid_kfs;
            sum_error += error_t;
            stats.max_error_t = std::max(stats.max_error_t, error_t);

            if (after_apply)
            {
                RCLCPP_WARN(
                    get_logger(),
                    "[F1L-GT-KF-ERROR-AFTER] task_id=%llu drone_id=%u epoch=%llu kf=%llu error_t=%.6f error_yaw=%.6f map_t=(%.3f,%.3f,%.3f) gt_t=(%.3f,%.3f,%.3f) association_dt=%.6f quality=%s",
                    static_cast<unsigned long long>(task_id),
                    keyframe_id.drone_id,
                    static_cast<unsigned long long>(keyframe_id.map_epoch),
                    static_cast<unsigned long long>(keyframe_id.local_kf_id),
                    error_t,
                    error_yaw,
                    world_pose.value()(0, 3),
                    world_pose.value()(1, 3),
                    world_pose.value()(2, 3),
                    gt_it->second.world_T_kf_gt(0, 3),
                    gt_it->second.world_T_kf_gt(1, 3),
                    gt_it->second.world_T_kf_gt(2, 3),
                    gt_it->second.association_dt_sec,
                    gt_it->second.association_quality.c_str());
            }
            else
            {
                RCLCPP_WARN(
                    get_logger(),
                    "[F1L-GT-KF-ERROR-BEFORE] task_id=%llu drone_id=%u epoch=%llu kf=%llu error_t=%.6f error_yaw=%.6f map_t=(%.3f,%.3f,%.3f) gt_t=(%.3f,%.3f,%.3f) association_dt=%.6f quality=%s",
                    static_cast<unsigned long long>(task_id),
                    keyframe_id.drone_id,
                    static_cast<unsigned long long>(keyframe_id.map_epoch),
                    static_cast<unsigned long long>(keyframe_id.local_kf_id),
                    error_t,
                    error_yaw,
                    world_pose.value()(0, 3),
                    world_pose.value()(1, 3),
                    world_pose.value()(2, 3),
                    gt_it->second.world_T_kf_gt(0, 3),
                    gt_it->second.world_T_kf_gt(1, 3),
                    gt_it->second.world_T_kf_gt(2, 3),
                    gt_it->second.association_dt_sec,
                    gt_it->second.association_quality.c_str());
            }
        }

        if (stats.valid_kfs > 0)
        {
            stats.mean_error_t = sum_error / static_cast<double>(stats.valid_kfs);
        }
        return stats;
    }

    void LogGtWindowComparison(uint64_t task_id,
                               const DebugGtWindowStats& before,
                               const DebugGtWindowStats& after) const
    {
        if (!f1l_gt_kf_debug_enabled_)
        {
            return;
        }

        uint64_t worsened_kfs = 0;
        for (const auto& [keyframe_id, after_error] : after.error_t_by_keyframe)
        {
            const auto before_it = before.error_t_by_keyframe.find(keyframe_id);
            if (before_it != before.error_t_by_keyframe.end() &&
                after_error > before_it->second + 0.05)
            {
                ++worsened_kfs;
            }
        }

        RCLCPP_WARN(
            get_logger(),
            "[F1L-GT-WINDOW-STATS] task_id=%llu valid_before=%llu valid_after=%llu mean_before=%.6f max_before=%.6f mean_after=%.6f max_after=%.6f worsened_kfs=%llu source=GAZEBO_GT_DEBUG",
            static_cast<unsigned long long>(task_id),
            static_cast<unsigned long long>(before.valid_kfs),
            static_cast<unsigned long long>(after.valid_kfs),
            before.mean_error_t,
            before.max_error_t,
            after.mean_error_t,
            after.max_error_t,
            static_cast<unsigned long long>(worsened_kfs));
    }

    std::vector<RawKeyFrameId> CollectGtCollateralKeyFrames(
        const PoseGraphProblem& problem) const
    {
        std::vector<RawKeyFrameId> keyframes;
        for (const auto& vertex : problem.vertices)
        {
            if (vertex.keyframe_id == problem.target_keyframe_id)
            {
                continue;
            }
            if (vertex.is_fixed || vertex.is_hard_fiducial ||
                vertex.is_anchor_neighborhood)
            {
                keyframes.push_back(vertex.keyframe_id);
            }
        }
        if (keyframes.empty())
        {
            for (const auto& keyframe_id : problem.fixed_keyframes)
            {
                if (!(keyframe_id == problem.target_keyframe_id))
                {
                    keyframes.push_back(keyframe_id);
                }
            }
        }
        return keyframes;
    }

    void LogGtCollateralCheck(uint64_t task_id,
                              const DebugGtWindowStats& before,
                              const DebugGtWindowStats& after,
                              const std::vector<RawKeyFrameId>& collateral) const
    {
        if (!f1l_gt_kf_debug_enabled_)
        {
            return;
        }

        double sum_before = 0.0;
        double sum_after = 0.0;
        uint64_t valid = 0;
        for (const auto& keyframe_id : collateral)
        {
            const auto before_it = before.error_t_by_keyframe.find(keyframe_id);
            const auto after_it = after.error_t_by_keyframe.find(keyframe_id);
            if (before_it == before.error_t_by_keyframe.end() ||
                after_it == after.error_t_by_keyframe.end())
            {
                continue;
            }
            sum_before += before_it->second;
            sum_after += after_it->second;
            ++valid;
        }

        const double mean_before =
            valid == 0 ? 0.0 : sum_before / static_cast<double>(valid);
        const double mean_after =
            valid == 0 ? 0.0 : sum_after / static_cast<double>(valid);
        RCLCPP_WARN(
            get_logger(),
            "[F1L-GT-COLLATERAL-CHECK] task_id=%llu valid_kfs=%llu previous_fid_neighborhood_mean_before=%.6f previous_fid_neighborhood_mean_after=%.6f source=GAZEBO_GT_DEBUG",
            static_cast<unsigned long long>(task_id),
            static_cast<unsigned long long>(valid),
            mean_before,
            mean_after);
    }

    void ExportF1LDebugAnimation(const PoseGraphProblem& problem,
                                 const OptimizationDryRunResult& dry_run) const
    {
        if (!f1l_debug_animation_enabled_)
        {
            return;
        }

        struct AnimationPoint
        {
            RawKeyFrameId id;
            Eigen::Matrix4d before = Eigen::Matrix4d::Identity();
            std::optional<Eigen::Matrix4d> after;
            std::optional<Eigen::Matrix4d> gt;
            bool is_vertex = false;
            bool is_fixed = false;
            bool is_target = false;
            bool is_anchor_neighborhood = false;
        };

        std::map<RawKeyFrameId, AnimationPoint> points;
        auto ensure_point = [&](const RawKeyFrameId& id) -> AnimationPoint&
        {
            auto& point = points[id];
            point.id = id;
            return point;
        };

        for (const auto& vertex : problem.vertices)
        {
            auto& point = ensure_point(vertex.keyframe_id);
            point.before = vertex.initial_world_T_kf;
            point.is_vertex = true;
            point.is_fixed = vertex.is_fixed || vertex.is_hard_fiducial;
            point.is_target = vertex.keyframe_id == problem.target_keyframe_id;
            point.is_anchor_neighborhood = vertex.is_anchor_neighborhood;
        }
        for (const auto& keyframe_id : problem.affected_non_variable_keyframes)
        {
            auto& point = ensure_point(keyframe_id);
            const auto pose = pose_store_.GetWorldPose(keyframe_id);
            if (pose.has_value())
            {
                point.before = pose.value();
            }
        }
        for (const auto& proposal : dry_run.proposed_vertex_poses)
        {
            auto& point = ensure_point(proposal.keyframe_id);
            point.before = proposal.before_world_T_kf;
            point.after = proposal.proposed_world_T_kf;
            point.is_vertex = true;
            point.is_fixed = point.is_fixed || proposal.fixed_vertex;
            point.is_target = point.is_target ||
                proposal.keyframe_id == problem.target_keyframe_id;
        }
        for (const auto& proposal : dry_run.proposed_propagated_poses)
        {
            auto& point = ensure_point(proposal.keyframe_id);
            point.before = proposal.before_world_T_kf;
            point.after = proposal.proposed_world_T_kf;
        }
        for (auto& [keyframe_id, point] : points)
        {
            const auto gt_it = f1l_gt_keyframe_store_.find(keyframe_id);
            if (gt_it != f1l_gt_keyframe_store_.end())
            {
                point.gt = gt_it->second.world_T_kf_gt;
            }
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
        uint64_t vertex_count = 0;
        uint64_t gt_count = 0;
        uint64_t after_count = 0;
        for (const auto& [keyframe_id, point] : points)
        {
            (void)keyframe_id;
            observe(point.before);
            if (point.gt.has_value())
            {
                observe(point.gt.value());
                ++gt_count;
            }
            if (point.after.has_value())
            {
                observe(point.after.value());
                ++after_count;
            }
            if (point.is_vertex)
            {
                ++vertex_count;
            }
        }
        if (points.empty() || !std::isfinite(min_x) || !std::isfinite(min_y))
        {
            RCLCPP_WARN(
                get_logger(),
                "[F1L-DEBUG-ANIMATION-EXPORT] task_id=%llu enabled=true success=false path=none reason=empty_window",
                static_cast<unsigned long long>(problem.task_id));
            return;
        }

        const double margin = 1.0;
        min_x -= margin;
        max_x += margin;
        min_y -= margin;
        max_y += margin;
        const double width = std::max(1.0, max_x - min_x);
        const double height = std::max(1.0, max_y - min_y);
        const double canvas = 1000.0;
        auto sx = [&](double x) { return 50.0 + (x - min_x) / width * (canvas - 100.0); };
        auto sy = [&](double y) { return canvas - 80.0 - (y - min_y) / height * (canvas - 140.0); };

        const std::string slash =
            (!f1l_debug_animation_output_dir_.empty() &&
             f1l_debug_animation_output_dir_.back() == '/') ? "" : "/";
        const std::string path =
            f1l_debug_animation_output_dir_ + slash +
            "f1l_debug_animation_task_" + std::to_string(problem.task_id) + ".html";
        std::error_code mkdir_error;
        std::filesystem::create_directories(
            f1l_debug_animation_output_dir_,
            mkdir_error);
        if (mkdir_error)
        {
            RCLCPP_WARN(
                get_logger(),
                "[F1L-DEBUG-ANIMATION-EXPORT] task_id=%llu enabled=true success=false path=%s reason=mkdir_failed_%s",
                static_cast<unsigned long long>(problem.task_id),
                path.c_str(),
                mkdir_error.message().c_str());
            return;
        }
        std::ofstream out(path);
        if (!out)
        {
            RCLCPP_WARN(
                get_logger(),
                "[F1L-DEBUG-ANIMATION-EXPORT] task_id=%llu enabled=true success=false path=%s reason=open_failed",
                static_cast<unsigned long long>(problem.task_id),
                path.c_str());
            return;
        }

        auto write_circle = [&](const AnimationPoint& point,
                                const Eigen::Matrix4d& pose,
                                const std::string& fill,
                                double radius,
                                double opacity)
        {
            out << "<circle cx=\"" << sx(pose(0, 3))
                << "\" cy=\"" << sy(pose(1, 3))
                << "\" r=\"" << radius
                << "\" fill=\"" << fill
                << "\" opacity=\"" << opacity
                << "\"><title>kf=" << point.id.local_kf_id
                << " drone=" << point.id.drone_id
                << " epoch=" << point.id.map_epoch
                << " x=" << pose(0, 3)
                << " y=" << pose(1, 3)
                << "</title></circle>\n";
        };

        out << "<!doctype html><html><head><meta charset=\"utf-8\"/>\n";
        out << "<title>F1L optimization task " << problem.task_id << "</title>\n";
        out << "<style>body{font-family:sans-serif;margin:0;background:#f7f7f4;color:#1f2933;}"
               "header{padding:12px 18px;background:#111827;color:white;}"
               "button{margin:8px 4px;padding:8px 12px;border:1px solid #999;background:white;cursor:pointer;}"
               "button.active{background:#1d4ed8;color:white;border-color:#1d4ed8;}"
               ".wrap{padding:12px 18px;}.frame{display:none}.frame.active{display:block}"
               "svg{background:white;border:1px solid #d1d5db;max-width:100%;height:auto}"
               ".legend span{display:inline-block;margin-right:16px;font-size:13px}</style>\n";
        out << "</head><body><header><strong>F1L task " << problem.task_id
            << "</strong> &nbsp; before=" << dry_run.before_error_t
            << " after=" << dry_run.after_error_t
            << " useful=" << (dry_run.useful ? "true" : "false")
            << " partial=" << (dry_run.partial_candidate ? "true" : "false")
            << "</header><div class=\"wrap\">\n";
        out << "<div><button id=\"b0\" class=\"active\" onclick=\"showFrame(0)\">Inicial</button>"
               "<button id=\"b1\" onclick=\"showFrame(1)\">Grafo</button>"
               "<button id=\"b2\" onclick=\"showFrame(2)\">Optimizado</button>"
               "<button onclick=\"togglePlay()\">Play/Pausa</button></div>\n";
        out << "<p class=\"legend\"><span style=\"color:#dc2626\">rojo: mapa antes</span>"
               "<span style=\"color:#111\">negro: GT</span>"
               "<span style=\"color:#7c3aed\">morado: vertices antes</span>"
               "<span style=\"color:#6b7280\">gris: GT de vertices</span>"
               "<span style=\"color:#2563eb\">azul: mapa optimizado/propagado</span>"
               "<span style=\"color:#16a34a\">verde: vertices optimizados</span></p>\n";

        auto write_svg_open = [&]()
        {
            out << "<svg width=\"1000\" height=\"1000\" viewBox=\"0 0 1000 1000\">\n";
            out << "<rect width=\"1000\" height=\"1000\" fill=\"white\"/>\n";
            out << "<text x=\"30\" y=\"970\" font-size=\"14\">XY projection. GT is diagnostic only and never feeds the solver.</text>\n";
        };
        auto write_edges = [&](const std::string& color, bool optimized)
        {
            for (const auto& edge : problem.edges)
            {
                const auto from_it = points.find(edge.from_keyframe_id);
                const auto to_it = points.find(edge.to_keyframe_id);
                if (from_it == points.end() || to_it == points.end())
                {
                    continue;
                }
                const auto& from_pose =
                    optimized && from_it->second.after.has_value()
                        ? from_it->second.after.value()
                        : from_it->second.before;
                const auto& to_pose =
                    optimized && to_it->second.after.has_value()
                        ? to_it->second.after.value()
                        : to_it->second.before;
                out << "<line x1=\"" << sx(from_pose(0, 3))
                    << "\" y1=\"" << sy(from_pose(1, 3))
                    << "\" x2=\"" << sx(to_pose(0, 3))
                    << "\" y2=\"" << sy(to_pose(1, 3))
                    << "\" stroke=\"" << color
                    << "\" stroke-width=\"1.5\" opacity=\"0.55\"/>\n";
            }
        };

        out << "<div id=\"f0\" class=\"frame active\">";
        write_svg_open();
        for (const auto& [keyframe_id, point] : points)
        {
            (void)keyframe_id;
            write_circle(point, point.before, "#dc2626", 3.5, 0.80);
            if (point.gt.has_value())
            {
                write_circle(point, point.gt.value(), "#111111", 3.0, 0.75);
            }
        }
        out << "</svg></div>\n";

        out << "<div id=\"f1\" class=\"frame\">";
        write_svg_open();
        write_edges("#7c3aed", false);
        for (const auto& [keyframe_id, point] : points)
        {
            (void)keyframe_id;
            write_circle(point, point.before, "#dc2626", 2.5, 0.30);
            if (point.gt.has_value())
            {
                write_circle(point, point.gt.value(), "#111111", 2.2, 0.25);
            }
            if (point.is_vertex)
            {
                write_circle(point, point.before,
                             point.is_fixed ? "#f97316" : "#7c3aed",
                             point.is_target ? 10.0 : 7.0,
                             0.95);
                if (point.gt.has_value())
                {
                    write_circle(point, point.gt.value(), "#6b7280",
                                 point.is_target ? 9.0 : 6.0,
                                 0.90);
                }
            }
        }
        out << "</svg></div>\n";

        out << "<div id=\"f2\" class=\"frame\">";
        write_svg_open();
        write_edges("#16a34a", true);
        for (const auto& [keyframe_id, point] : points)
        {
            (void)keyframe_id;
            if (point.gt.has_value())
            {
                write_circle(point, point.gt.value(), "#111111", 2.5, 0.35);
            }
            if (point.after.has_value())
            {
                write_circle(point, point.after.value(),
                             point.is_vertex ? "#16a34a" : "#2563eb",
                             point.is_vertex ? (point.is_target ? 10.0 : 7.0) : 3.5,
                             0.90);
            }
        }
        out << "</svg></div>\n";

        out << "<p>window_kfs=" << points.size()
            << " vertices=" << vertex_count
            << " gt_valid=" << gt_count
            << " optimized_or_propagated=" << after_count
            << "</p>\n";
        out << "<script>let frame=0,playing=true;function showFrame(i){frame=i;"
               "for(let n=0;n<3;n++){document.getElementById('f'+n).classList.toggle('active',n===i);"
               "document.getElementById('b'+n).classList.toggle('active',n===i);}}"
               "function togglePlay(){playing=!playing;}setInterval(()=>{if(playing)showFrame((frame+1)%3);},1400);</script>\n";
        out << "</div></body></html>\n";

        RCLCPP_WARN(
            get_logger(),
            "[F1L-DEBUG-ANIMATION-EXPORT] task_id=%llu enabled=true success=true path=%s window_kfs=%llu vertices=%llu gt_valid=%llu optimized_or_propagated=%llu format=html_animation",
            static_cast<unsigned long long>(problem.task_id),
            path.c_str(),
            static_cast<unsigned long long>(points.size()),
            static_cast<unsigned long long>(vertex_count),
            static_cast<unsigned long long>(gt_count),
            static_cast<unsigned long long>(after_count));
    }

    const FiducialConfig* FindContainingFiducial(const Eigen::Matrix4d& world_T_body,
                                                 double& distance_m) const
    {
        const FiducialConfig* best = nullptr;
        double best_distance = 0.0;
        for (const auto& fiducial : fiducials_)
        {
            const Eigen::Vector3d delta =
                world_T_body.block<3, 1>(0, 3) -
                fiducial.world_T_fiducial.block<3, 1>(0, 3);
            const double distance = delta.norm();
            if (!best || distance < best_distance)
            {
                best = &fiducial;
                best_distance = distance;
            }
        }

        distance_m = best_distance;
        if (!best || best_distance > best->radius_m)
        {
            return nullptr;
        }
        return best;
    }

    uint64_t MakeGlobalKeyFrameId(const RawKeyFrameId& keyframe_id) const
    {
        // F1E: ID compacto solo para logs/journal. La identidad canonica sigue
        // siendo `(drone_id, map_epoch, local_kf_id)`.
        return static_cast<uint64_t>(keyframe_id.drone_id) * 1000000000000ULL +
               keyframe_id.map_epoch * 1000000ULL +
               keyframe_id.local_kf_id;
    }

    void ProcessFiducialsForDelta(const OrbMap& map, uint64_t arrival_id)
    {
        if (!fiducial_sim_enabled_ || rawdb_replay_enabled_ || fiducials_.empty())
        {
            return;
        }

        for (const auto& keyframe : map.keyframes)
        {
            const RawKeyFrameId keyframe_id{
                map.drone_id,
                map.map_epoch,
                keyframe.id};
            if (live_fiducial_observed_keyframes_.find(keyframe_id) !=
                live_fiducial_observed_keyframes_.end())
            {
                continue;
            }

            GroundTruthSample gt_sample;
            double dt_sec = 0.0;
            std::string source;
            const rclcpp::Time keyframe_stamp(keyframe.stamp);
            if (!FindBestGroundTruth(map.drone_id, keyframe_stamp, gt_sample, dt_sec, source))
            {
                continue;
            }

            double distance_m = 0.0;
            const FiducialConfig* fiducial =
                FindContainingFiducial(gt_sample.world_T_body, distance_m);
            RCLCPP_WARN(
                get_logger(),
                "[F1E-FID-CANDIDATE-GT] arrival_id=%llu drone_id=%u epoch=%llu kf=%llu kf_stamp=%.6f gt_stamp=%.6f dt=%.6f dist_to_best_fid=%.3f inside=%s source=%s",
                static_cast<unsigned long long>(arrival_id),
                map.drone_id,
                static_cast<unsigned long long>(map.map_epoch),
                static_cast<unsigned long long>(keyframe.id),
                StampToSeconds(keyframe.stamp),
                gt_sample.stamp.seconds(),
                dt_sec,
                distance_m,
                fiducial ? "true" : "false",
                source.c_str());

            if (!fiducial)
            {
                continue;
            }

            FiducialObservation observation;
            observation.arrival_id = arrival_id;
            observation.drone_id = map.drone_id;
            observation.map_epoch = map.map_epoch;
            observation.local_keyframe_id = keyframe.id;
            observation.global_keyframe_id = MakeGlobalKeyFrameId(keyframe_id);
            observation.fiducial_id = fiducial->id;
            observation.world_T_body_fiducial = gt_sample.world_T_body;
            observation.keyframe_stamp_sec = StampToSeconds(keyframe.stamp);
            observation.gt_stamp_sec = gt_sample.stamp.seconds();
            observation.association_dt_sec = dt_sec;
            observation.distance_to_fiducial_m = distance_m;
            observation.source = source;

            RCLCPP_WARN(
                get_logger(),
                "[F1E-FID-KF-ASSOC] arrival_id=%llu drone_id=%u epoch=%llu kf=%llu fid=%d kf_stamp=%.6f gt_stamp=%.6f dt=%.6f dist_to_fid=%.3f source=%s",
                static_cast<unsigned long long>(arrival_id),
                observation.drone_id,
                static_cast<unsigned long long>(observation.map_epoch),
                static_cast<unsigned long long>(observation.local_keyframe_id),
                observation.fiducial_id,
                observation.keyframe_stamp_sec,
                observation.gt_stamp_sec,
                observation.association_dt_sec,
                observation.distance_to_fiducial_m,
                observation.source.c_str());

            HandleFiducialObservation(observation, true);
            live_fiducial_observed_keyframes_.insert(keyframe_id);
        }
    }

    void HandleFiducialObservation(const FiducialObservation& observation,
                                   bool persist_observation)
    {
        RCLCPP_WARN(
            get_logger(),
            "[F1E-FID-OBS] arrival_id=%llu drone_id=%u epoch=%llu kf=%llu fid=%d source=%s",
            static_cast<unsigned long long>(observation.arrival_id),
            observation.drone_id,
            static_cast<unsigned long long>(observation.map_epoch),
            static_cast<unsigned long long>(observation.local_keyframe_id),
            observation.fiducial_id,
            observation.source.c_str());

        const auto result =
            fiducial_anchor_manager_.RegisterFiducialObservation(
                observation,
                raw_db_,
                pose_store_);

        if (!result.observation_accepted)
        {
            RCLCPP_WARN(
                get_logger(),
                "[F1E-FID-OBS] arrival_id=%llu status=rejected reason=%s",
                static_cast<unsigned long long>(observation.arrival_id),
                result.reason.c_str());
            return;
        }

        RCLCPP_WARN(
            get_logger(),
            "[F1E-FID-WORLD-T-LOCAL] arrival_id=%llu fid=%d drone_id=%u epoch=%llu kf=%llu world_T_local_t=(%.3f,%.3f,%.3f) yaw=%.3f source=%s",
            static_cast<unsigned long long>(observation.arrival_id),
            observation.fiducial_id,
            observation.drone_id,
            static_cast<unsigned long long>(observation.map_epoch),
            static_cast<unsigned long long>(observation.local_keyframe_id),
            result.world_T_local(0, 3),
            result.world_T_local(1, 3),
            result.world_T_local(2, 3),
            YawFromTransform(result.world_T_local),
            observation.source.c_str());

        RCLCPP_WARN(
            get_logger(),
            "[F1F-BODY-CAMERA-APPLY] arrival_id=%llu fid=%d drone_id=%u epoch=%llu kf=%llu body_t=(%.3f,%.3f,%.3f) camera_t=(%.3f,%.3f,%.3f) source=%s",
            static_cast<unsigned long long>(observation.arrival_id),
            observation.fiducial_id,
            observation.drone_id,
            static_cast<unsigned long long>(observation.map_epoch),
            static_cast<unsigned long long>(observation.local_keyframe_id),
            observation.world_T_body_fiducial(0, 3),
            observation.world_T_body_fiducial(1, 3),
            observation.world_T_body_fiducial(2, 3),
            result.world_T_camera_fiducial(0, 3),
            result.world_T_camera_fiducial(1, 3),
            result.world_T_camera_fiducial(2, 3),
            observation.source.c_str());

        if (result.first_anchor_created)
        {
            RCLCPP_WARN(
                get_logger(),
                "[F1E-FID-FIRST-ANCHOR] fid=%d drone_id=%u epoch=%llu kf=%llu body_t=(%.3f,%.3f,%.3f) camera_t=(%.3f,%.3f,%.3f) world_T_local_t=(%.3f,%.3f,%.3f) yaw=%.3f",
                observation.fiducial_id,
                observation.drone_id,
                static_cast<unsigned long long>(observation.map_epoch),
                static_cast<unsigned long long>(observation.local_keyframe_id),
                observation.world_T_body_fiducial(0, 3),
                observation.world_T_body_fiducial(1, 3),
                observation.world_T_body_fiducial(2, 3),
                result.world_T_camera_fiducial(0, 3),
                result.world_T_camera_fiducial(1, 3),
                result.world_T_camera_fiducial(2, 3),
                result.world_T_local(0, 3),
                result.world_T_local(1, 3),
                result.world_T_local(2, 3),
                YawFromTransform(result.world_T_local));

            RCLCPP_WARN(
                get_logger(),
                "[F1D-POSESTORE-ANCHOR-SET] drone_id=%u epoch=%llu source=%s replaced=false",
                observation.drone_id,
                static_cast<unsigned long long>(observation.map_epoch),
                observation.source.c_str());
        }

        if (result.keyframe_marked_hard_fiducial)
        {
            RCLCPP_WARN(
                get_logger(),
                "[F1E-FID-KF-HARD] drone_id=%u epoch=%llu kf=%llu fid=%d source=%s",
                observation.drone_id,
                static_cast<unsigned long long>(observation.map_epoch),
                static_cast<unsigned long long>(observation.local_keyframe_id),
                observation.fiducial_id,
                observation.source.c_str());
        }

        if (result.revisit)
        {
            const char* decision = "UNKNOWN";
            if (result.revisit_ok)
            {
                decision = "OK";
            }
            else if (result.task_created)
            {
                decision = "TASK_CREATED";
            }
            else if (result.duplicate_task)
            {
                decision = "TASK_DUPLICATE";
            }

            RCLCPP_WARN(
                get_logger(),
                "[F1H-FID-REVISIT] arrival_id=%llu fid=%d drone_id=%u epoch=%llu kf=%llu source=%s decision=%s",
                static_cast<unsigned long long>(observation.arrival_id),
                observation.fiducial_id,
                observation.drone_id,
                static_cast<unsigned long long>(observation.map_epoch),
                static_cast<unsigned long long>(observation.local_keyframe_id),
                observation.source.c_str(),
                decision);

            if (result.pose_error_valid)
            {
                RCLCPP_WARN(
                    get_logger(),
                    "[F1H-FID-POSE-ERROR] fid=%d kf=%llu drone_id=%u epoch=%llu error_t=%.6f error_rot=%.6f error_yaw=%.6f threshold_t=%.6f threshold_rot=%.6f threshold_yaw=%.6f decision=%s source=%s",
                    observation.fiducial_id,
                    static_cast<unsigned long long>(observation.local_keyframe_id),
                    observation.drone_id,
                    static_cast<unsigned long long>(observation.map_epoch),
                    result.error_t_m,
                    result.error_rot_rad,
                    result.error_yaw_rad,
                    result.threshold_t_m,
                    result.threshold_rot_rad,
                    result.threshold_yaw_rad,
                    decision,
                    observation.source.c_str());
            }

            if (result.revisit_ok)
            {
                RCLCPP_WARN(
                    get_logger(),
                    "[F1H-FID-OK] arrival_id=%llu fid=%d drone_id=%u epoch=%llu kf=%llu error_t=%.6f error_yaw=%.6f",
                    static_cast<unsigned long long>(observation.arrival_id),
                    observation.fiducial_id,
                    observation.drone_id,
                    static_cast<unsigned long long>(observation.map_epoch),
                    static_cast<unsigned long long>(observation.local_keyframe_id),
                    result.error_t_m,
                    result.error_yaw_rad);
            }
            else if (result.task_created)
            {
                RCLCPP_WARN(
                    get_logger(),
                    "[F1H-FID-TASK-CREATED] task_id=%llu arrival_id=%llu fid=%d drone_id=%u epoch=%llu kf=%llu error_t=%.6f error_rot=%.6f error_yaw=%.6f",
                    static_cast<unsigned long long>(result.task_id),
                    static_cast<unsigned long long>(observation.arrival_id),
                    observation.fiducial_id,
                    observation.drone_id,
                    static_cast<unsigned long long>(observation.map_epoch),
                    static_cast<unsigned long long>(observation.local_keyframe_id),
                    result.error_t_m,
                    result.error_rot_rad,
                    result.error_yaw_rad);
            }
            else if (result.duplicate_task)
            {
                RCLCPP_WARN(
                    get_logger(),
                    "[F1H-FID-TASK-SKIP-DUPLICATE] task_id=%llu arrival_id=%llu fid=%d drone_id=%u epoch=%llu kf=%llu",
                    static_cast<unsigned long long>(result.task_id),
                    static_cast<unsigned long long>(observation.arrival_id),
                    observation.fiducial_id,
                    observation.drone_id,
                    static_cast<unsigned long long>(observation.map_epoch),
                    static_cast<unsigned long long>(observation.local_keyframe_id));
            }
        }

        if (persist_observation)
        {
            raw_db_.AddFiducialObservation(ToRecordedObservation(observation));
            const auto fid_count =
                raw_db_.GetDatabaseStats().fiducial_observations;
            RCLCPP_WARN(
                get_logger(),
                "[F1E-FID-JOURNAL-SAVE] arrival_id=%llu fid=%d drone_id=%u epoch=%llu kf=%llu observations=%llu",
                static_cast<unsigned long long>(observation.arrival_id),
                observation.fiducial_id,
                observation.drone_id,
                static_cast<unsigned long long>(observation.map_epoch),
                static_cast<unsigned long long>(observation.local_keyframe_id),
                static_cast<unsigned long long>(fid_count));
        }

        LogFiducialStats(observation.source);
        LogPoseStoreStats("fiducial_observation");
        BuildPoseGraphsForPendingTasks(observation.source);
    }

    void LogFiducialStats(const std::string& reason)
    {
        const auto stats = fiducial_anchor_manager_.GetStats();
        RCLCPP_WARN(
            get_logger(),
            "[F1E-FID-STATS] reason=%s observations=%llu accepted=%llu rejected=%llu anchors_created=%llu replay_observations=%llu hard_fiducial_kfs=%llu",
            reason.c_str(),
            static_cast<unsigned long long>(stats.observations),
            static_cast<unsigned long long>(stats.accepted),
            static_cast<unsigned long long>(stats.rejected),
            static_cast<unsigned long long>(stats.anchors_created),
            static_cast<unsigned long long>(stats.replay_observations),
            static_cast<unsigned long long>(stats.hard_fiducial_keyframes));

        RCLCPP_WARN(
            get_logger(),
            "[F1H-FID-TASK-STATS] reason=%s total=%llu pending=%llu confirmed_ok=%llu high_error=%llu duplicates=%llu no_pose=%llu revisits=%llu",
            reason.c_str(),
            static_cast<unsigned long long>(stats.tasks_created),
            static_cast<unsigned long long>(
                fiducial_anchor_manager_.PendingFiducialOptimizationTaskCount()),
            static_cast<unsigned long long>(stats.revisit_ok),
            static_cast<unsigned long long>(stats.revisit_high_error),
            static_cast<unsigned long long>(stats.task_duplicates),
            static_cast<unsigned long long>(stats.revisit_no_pose),
                static_cast<unsigned long long>(stats.revisit_observations));
    }

    void BuildPoseGraphsForPendingTasks(const std::string& reason)
    {
        // F1I: consume de forma no destructiva las tareas pendientes expuestas
        // por F1H. Se guarda solo el ID ya loggeado para no reconstruir el mismo
        // problema continuamente; no se elimina la tarea del backend.
        const auto tasks =
            fiducial_anchor_manager_.GetPendingFiducialOptimizationTasks();
        for (const auto& task : tasks)
        {
            if (f1i_pose_graph_built_task_ids_.find(task.task_id) !=
                f1i_pose_graph_built_task_ids_.end())
            {
                continue;
            }

            const auto build_result =
                BuildAndLogPoseGraphForTask(task, reason, false);
            if (build_result.success)
            {
                f1i_pose_graph_built_task_ids_.insert(task.task_id);
            }
        }
    }

    PoseGraphBuildResult BuildAndLogPoseGraphForTask(
        const FiducialOptimizationTask& task,
        const std::string& trigger,
        bool debug_task)
    {
        RCLCPP_WARN(
            get_logger(),
            "[F1I-TASK-RX] task_id=%llu type=%s source=%s trigger=%s debug=%s drone_id=%u epoch=%llu kf=%llu fid=%d error_t=%.6f error_rot=%.6f error_yaw=%.6f",
            static_cast<unsigned long long>(task.task_id),
            task.task_type.c_str(),
            task.source.c_str(),
            trigger.c_str(),
            debug_task ? "true" : "false",
            task.drone_id,
            static_cast<unsigned long long>(task.map_epoch),
            static_cast<unsigned long long>(task.keyframe_id.local_kf_id),
            task.fiducial_id,
            task.error_t_m,
            task.error_rot_rad,
            task.error_yaw_rad);

        RCLCPP_WARN(
            get_logger(),
            "[F1I-GRAPH-BUILD-START] task_id=%llu drone_id=%u epoch=%llu target_kf=%llu source=%s",
            static_cast<unsigned long long>(task.task_id),
            task.submap_id.drone_id,
            static_cast<unsigned long long>(task.submap_id.map_epoch),
            static_cast<unsigned long long>(task.keyframe_id.local_kf_id),
            task.source.c_str());

        const auto partial_it =
            f1l_partial_checkpoint_task_by_submap_.find(task.submap_id);
        if (partial_it != f1l_partial_checkpoint_task_by_submap_.end())
        {
            const uint64_t retry_count =
                f1l_partial_retry_count_by_submap_[task.submap_id];
            RCLCPP_WARN(
                get_logger(),
                "[F1I-GRAPH-REBUILD-FROM-PARTIAL] previous_task_id=%llu new_task_id=%llu drone_id=%u epoch=%llu retry_count=%llu max_retries=%d source=GlobalPoseStore_partial_checkpoint",
                static_cast<unsigned long long>(partial_it->second),
                static_cast<unsigned long long>(task.task_id),
                task.submap_id.drone_id,
                static_cast<unsigned long long>(task.submap_id.map_epoch),
                static_cast<unsigned long long>(retry_count),
                f1l_max_partial_retries_);
        }

        auto build_result =
            pose_graph_builder_.BuildForFiducialTask(task, raw_db_, pose_store_);
        if (!build_result.success)
        {
            const auto& coverage = build_result.problem.coverage;
            if (!coverage.reason.empty())
            {
                RCLCPP_ERROR(
                    get_logger(),
                    "[F1I-GRAPH-REJECT] task_id=%llu reason=%s window_keyframes=%llu control_vertices=%llu max_control_span_kfs=%llu max_allowed_edge_kf_gap=%d max_control_span_m=%.3f max_allowed_edge_length_m=%.3f uncovered_long_segments=%llu mandatory_vertices_missing=%llu max_vertices_exhausted=%s",
                    static_cast<unsigned long long>(task.task_id),
                    coverage.reason.c_str(),
                    static_cast<unsigned long long>(coverage.window_keyframes),
                    static_cast<unsigned long long>(coverage.control_vertices),
                    static_cast<unsigned long long>(coverage.max_control_span_kfs),
                    pose_graph_max_temporal_edge_kf_gap_,
                    coverage.max_control_span_m,
                    pose_graph_max_temporal_edge_length_m_,
                    static_cast<unsigned long long>(coverage.uncovered_long_segments),
                    static_cast<unsigned long long>(coverage.mandatory_vertices_missing),
                    coverage.max_vertices_exhausted ? "true" : "false");
            }
            RCLCPP_ERROR(
                get_logger(),
                "[F1I-GRAPH-BUILD-SUMMARY] task_id=%llu success=false reason=%s vertices=0 edges=0 priors=0 propagation=0",
                static_cast<unsigned long long>(task.task_id),
                build_result.reason.c_str());
            return build_result;
        }

        if (partial_it != f1l_partial_checkpoint_task_by_submap_.end())
        {
            build_result.problem.rebuilt_from_partial_checkpoint = true;
            build_result.problem.partial_parent_task_id = partial_it->second;
            build_result.problem.partial_retry_count =
                f1l_partial_retry_count_by_submap_[task.submap_id];
        }

        LogPoseGraphProblem(build_result.problem, build_result.reason);
        RunAndLogOptimizationDryRun(task, build_result.problem, trigger, debug_task);
        return build_result;
    }

    bool SameRawStats(const RawDatabaseStats& a, const RawDatabaseStats& b) const
    {
        return a.journal_entries == b.journal_entries &&
               a.delta_entries == b.delta_entries &&
               a.full_snapshot_entries == b.full_snapshot_entries &&
               a.fiducial_observations == b.fiducial_observations &&
               a.submaps == b.submaps &&
               a.keyframes == b.keyframes &&
               a.mappoints == b.mappoints &&
               a.last_arrival_id == b.last_arrival_id;
    }

    bool SamePoseStoreStats(const orbslam3_multi::GlobalPoseStoreStats& a,
                            const orbslam3_multi::GlobalPoseStoreStats& b) const
    {
        return a.anchored_submaps == b.anchored_submaps &&
               a.keyframe_world_poses == b.keyframe_world_poses &&
               a.optimized_keyframes == b.optimized_keyframes &&
               a.propagated_keyframes == b.propagated_keyframes &&
               a.rebased_keyframes == b.rebased_keyframes &&
               a.submap_corrections == b.submap_corrections &&
               a.hard_fiducial_keyframes == b.hard_fiducial_keyframes;
    }

    void RunAndLogOptimizationDryRun(const FiducialOptimizationTask& task,
                                     const PoseGraphProblem& problem,
                                     const std::string& trigger,
                                     bool debug_task)
    {
        if (!f1j_dryrun_enabled_)
        {
            RCLCPP_WARN(
                get_logger(),
                "[F1J-SERVER-DRYRUN-DONE] task_id=%llu enabled=false reason=disabled",
                static_cast<unsigned long long>(problem.task_id));
            return;
        }

        const auto raw_stats_before = raw_db_.GetDatabaseStats();
        const auto pose_stats_before = pose_store_.GetPoseStoreStats();

        RCLCPP_WARN(
            get_logger(),
            "[F1J-SERVER-DRYRUN-REQUEST] task_id=%llu trigger=%s debug=%s",
            static_cast<unsigned long long>(problem.task_id),
            trigger.c_str(),
            debug_task ? "true" : "false");
        RCLCPP_WARN(
            get_logger(),
            "[F1J-OPT-TASK-RX] task_id=%llu type=%s source=%s drone_id=%u epoch=%llu kf=%llu fid=%d before_task_error_t=%.6f before_task_error_yaw=%.6f",
            static_cast<unsigned long long>(task.task_id),
            task.task_type.c_str(),
            task.source.c_str(),
            task.drone_id,
            static_cast<unsigned long long>(task.map_epoch),
            static_cast<unsigned long long>(task.keyframe_id.local_kf_id),
            task.fiducial_id,
            task.error_t_m,
            task.error_yaw_rad);
        RCLCPP_WARN(
            get_logger(),
            "[F1J-OPT-GRAPH-RX] task_id=%llu vertices=%llu variable=%llu fixed=%llu hard_fiducial=%llu edges=%llu priors=%llu propagation=%llu",
            static_cast<unsigned long long>(problem.task_id),
            static_cast<unsigned long long>(problem.summary.vertices),
            static_cast<unsigned long long>(problem.summary.variable_vertices),
            static_cast<unsigned long long>(problem.summary.fixed_vertices),
            static_cast<unsigned long long>(problem.summary.hard_fiducial_vertices),
            static_cast<unsigned long long>(problem.summary.edges),
            static_cast<unsigned long long>(problem.summary.priors),
            static_cast<unsigned long long>(problem.summary.propagation_entries));

        DumpPoseGraphProblemForOffline(problem);

        const OptimizationDryRunResult result =
            optimization_manager_.RunDryRun(problem, raw_db_, pose_store_);

        if (!result.precheck_ok)
        {
            RCLCPP_ERROR(
                get_logger(),
                "[F1J-OPT-PRECHECK] task_id=%llu ok=false reason=%s vertices=%llu variable=%llu fixed=%llu edges=%llu priors=%llu propagation=%llu",
                static_cast<unsigned long long>(problem.task_id),
                result.reason.c_str(),
                static_cast<unsigned long long>(problem.summary.vertices),
                static_cast<unsigned long long>(problem.summary.variable_vertices),
                static_cast<unsigned long long>(problem.summary.fixed_vertices),
                static_cast<unsigned long long>(problem.summary.edges),
                static_cast<unsigned long long>(problem.summary.priors),
                static_cast<unsigned long long>(problem.summary.propagation_entries));
            RCLCPP_ERROR(
                get_logger(),
                "[F1J-OPT-PRECHECK-FAIL] task_id=%llu reason=%s",
                static_cast<unsigned long long>(problem.task_id),
                result.reason.c_str());
            return;
        }

        RCLCPP_WARN(
            get_logger(),
            "[F1J-OPT-PRECHECK] task_id=%llu ok=true vertices=%llu variable=%llu fixed=%llu edges=%llu priors=%llu propagation=%llu",
            static_cast<unsigned long long>(problem.task_id),
            static_cast<unsigned long long>(problem.summary.vertices),
            static_cast<unsigned long long>(problem.summary.variable_vertices),
            static_cast<unsigned long long>(problem.summary.fixed_vertices),
            static_cast<unsigned long long>(problem.summary.edges),
            static_cast<unsigned long long>(problem.summary.priors),
            static_cast<unsigned long long>(problem.summary.propagation_entries));
        RCLCPP_WARN(
            get_logger(),
            "[F1J-OPT-POSE-COPY] task_id=%llu copied=%llu",
            static_cast<unsigned long long>(problem.task_id),
            static_cast<unsigned long long>(result.copied_poses));
        RCLCPP_WARN(
            get_logger(),
            "[F1J-OPT-CONSTRAINTS] task_id=%llu edges=%llu priors=%llu task_type=%s",
            static_cast<unsigned long long>(problem.task_id),
            static_cast<unsigned long long>(problem.summary.edges),
            static_cast<unsigned long long>(problem.summary.priors),
            problem.task_type.c_str());
        RCLCPP_WARN(
            get_logger(),
            "[F1J-OPT-DRYRUN-START] task_id=%llu solver=fiducial_window_progressive_deformation",
            static_cast<unsigned long long>(problem.task_id));
        RCLCPP_WARN(
            get_logger(),
            "[F1J-OPT-SOLVER-SUMMARY] task_id=%llu success=%s iterations=%llu initial_cost=%.6f final_cost=%.6f moved_kfs=%llu fixed_kfs=%llu max_delta_t=%.6f mean_delta_t=%.6f max_delta_yaw=%.6f hard_fixed_moved=%s",
            static_cast<unsigned long long>(problem.task_id),
            result.success ? "true" : "false",
            static_cast<unsigned long long>(result.solver_iterations),
            result.initial_cost,
            result.final_cost,
            static_cast<unsigned long long>(result.moved_kfs),
            static_cast<unsigned long long>(result.fixed_kfs),
            result.max_delta_t,
            result.mean_delta_t,
            result.max_delta_yaw,
            result.hard_fixed_moved ? "true" : "false");
        RCLCPP_WARN(
            get_logger(),
            "[F1J-OPT-ANCHOR-PRESERVATION] task_id=%llu ok=%s required=%s graph_satisfied=%s previous_fiducial_fixed_count=%llu previous_fiducial_neighborhood_fixed_count=%llu fixed_kfs=%llu hard_fixed_moved=%s max_previous_fiducial_delta_t=%.9f max_previous_fiducial_delta_yaw=%.9f max_previous_fiducial_neighborhood_delta_t=%.9f max_previous_fiducial_neighborhood_delta_yaw=%.9f",
            static_cast<unsigned long long>(problem.task_id),
            result.anchor_preservation_ok ? "true" : "false",
            result.anchor_preservation_required ? "true" : "false",
            result.anchor_preservation_graph_satisfied ? "true" : "false",
            static_cast<unsigned long long>(result.previous_fiducial_fixed_count),
            static_cast<unsigned long long>(
                result.previous_fiducial_neighborhood_fixed_count),
            static_cast<unsigned long long>(result.fixed_kfs),
            result.hard_fixed_moved ? "true" : "false",
            result.max_previous_fiducial_delta_t,
            result.max_previous_fiducial_delta_yaw,
            result.max_previous_fiducial_neighborhood_delta_t,
            result.max_previous_fiducial_neighborhood_delta_yaw);
        RCLCPP_WARN(
            get_logger(),
            "[F1J-OPT-PROPAGATION-DRYRUN] task_id=%llu affected_non_variable=%llu proposed=%llu max_propagated_delta_t=%.6f mean_propagated_delta_t=%.6f mode=path_segment_interpolated",
            static_cast<unsigned long long>(problem.task_id),
            static_cast<unsigned long long>(problem.summary.affected_non_variable_keyframes),
            static_cast<unsigned long long>(result.propagated_kfs),
            result.max_propagated_delta_t,
            result.mean_propagated_delta_t);
        RCLCPP_WARN(
            get_logger(),
            "[F1J-OPT-DRYRUN-RESULT] task_id=%llu task=%s success=%s before_t=%.6f after_t=%.6f before_yaw=%.6f after_yaw=%.6f improvement_ratio=%.6f initial_cost=%.6f final_cost=%.6f",
            static_cast<unsigned long long>(problem.task_id),
            result.task_type.c_str(),
            result.success ? "true" : "false",
            result.before_error_t,
            result.after_error_t,
            result.before_error_yaw,
            result.after_error_yaw,
            result.improvement_ratio,
            result.initial_cost,
            result.final_cost);
        RCLCPP_WARN(
            get_logger(),
            "[F1J-OPT-DRYRUN-DECISION] task_id=%llu useful=%s partial_candidate=%s reason=%s partial_reason=%s before_t=%.6f after_t=%.6f improvement_ratio=%.6f max_delta_t=%.6f max_delta_yaw=%.6f",
            static_cast<unsigned long long>(problem.task_id),
            result.useful ? "true" : "false",
            result.partial_candidate ? "true" : "false",
            result.decision_reason.c_str(),
            result.partial_reason.empty() ? "none" : result.partial_reason.c_str(),
            result.before_error_t,
            result.after_error_t,
            result.improvement_ratio,
            result.max_delta_t,
            result.max_delta_yaw);

        const auto raw_stats_after = raw_db_.GetDatabaseStats();
        const auto pose_stats_after = pose_store_.GetPoseStoreStats();
        const bool raw_unchanged = SameRawStats(raw_stats_before, raw_stats_after);
        const bool pose_unchanged =
            SamePoseStoreStats(pose_stats_before, pose_stats_after);
        RCLCPP_WARN(
            get_logger(),
            "[F1J-OPT-NO-STATE-MUTATION] task_id=%llu ok=%s raw_unchanged=%s pose_store_unchanged=%s",
            static_cast<unsigned long long>(problem.task_id),
            (raw_unchanged && pose_unchanged) ? "true" : "false",
            raw_unchanged ? "true" : "false",
            pose_unchanged ? "true" : "false");

        if (result.success && f1j_export_debug_plot_)
        {
            const std::string slash =
                (!f1j_debug_output_dir_.empty() &&
                 f1j_debug_output_dir_.back() == '/') ? "" : "/";
            const std::string path =
                f1j_debug_output_dir_ + slash +
                "f1j_dryrun_task_" + std::to_string(problem.task_id) + ".svg";
            const auto export_result =
                optimization_debug_exporter_.ExportDryRun2DPlot(problem, result, path);
            RCLCPP_WARN(
                get_logger(),
                "[F1J-OPT-DEBUG-EXPORT] task_id=%llu enabled=true success=%s path=%s reason=%s",
                static_cast<unsigned long long>(problem.task_id),
                export_result.success ? "true" : "false",
                export_result.path.c_str(),
                export_result.reason.c_str());
        }
        else
        {
            RCLCPP_WARN(
                get_logger(),
                "[F1J-OPT-DEBUG-EXPORT] task_id=%llu enabled=false reason=disabled_by_default",
                static_cast<unsigned long long>(problem.task_id));
        }

        // 1J con nombre legacy `F1L`: exporta una animacion HTML del dry-run
        // con mapa antes, GT externo y propuesta optimizada. No abre ventanas
        // desde ROS ni realimenta el servidor; 1L puede usarla despues como
        // evidencia cruzada.
        ExportF1LDebugAnimation(problem, result);

        const auto partial_retry_it =
            f1l_partial_retry_count_by_submap_.find(problem.submap_id);
        const uint64_t partial_retry_count =
            partial_retry_it == f1l_partial_retry_count_by_submap_.end()
                ? 0U
                : partial_retry_it->second;
        const bool partial_retry_limit_ok =
            partial_retry_it == f1l_partial_retry_count_by_submap_.end() ||
            partial_retry_count < static_cast<uint64_t>(f1l_max_partial_retries_);
        const bool can_apply_partial =
            f1k_apply_enabled_ &&
            f1l_validation_enabled_ &&
            f1l_partial_apply_enabled_ &&
            partial_retry_limit_ok &&
            result.partial_candidate &&
            !result.useful;
        if (result.partial_candidate && !result.useful &&
            !partial_retry_limit_ok)
        {
            RCLCPP_ERROR(
                get_logger(),
                "[F1L-PARTIAL-RETRY-LIMIT] task_id=%llu drone_id=%u epoch=%llu retry_count=%llu max_retries=%d applied=false",
                static_cast<unsigned long long>(problem.task_id),
                problem.submap_id.drone_id,
                static_cast<unsigned long long>(problem.submap_id.map_epoch),
                static_cast<unsigned long long>(partial_retry_count),
                f1l_max_partial_retries_);
        }
        if (result.partial_candidate && !result.useful && !can_apply_partial)
        {
            RCLCPP_WARN(
                get_logger(),
                "[F1K-OPT-PARTIAL-PENDING] task_id=%llu applied=false useful=false partial_candidate=true reason=%s required_next=F1L_DECISION",
                static_cast<unsigned long long>(problem.task_id),
                result.partial_reason.empty()
                    ? result.decision_reason.c_str()
                    : result.partial_reason.c_str());
        }
        if (f1k_apply_enabled_ && (result.useful || can_apply_partial))
        {
            ApplyAndLogOptimizationResult(task, result, problem, trigger);
        }
        else if (!f1k_apply_enabled_ && result.useful)
        {
            RCLCPP_WARN(
                get_logger(),
                "[F1K-OPT-APPLY-SKIP] task_id=%llu reason=disabled useful=true",
                static_cast<unsigned long long>(problem.task_id));
        }

        RCLCPP_WARN(
            get_logger(),
            "[F1J-SERVER-DRYRUN-DONE] task_id=%llu success=%s useful=%s partial_candidate=%s no_apply=%s",
            static_cast<unsigned long long>(problem.task_id),
            result.success ? "true" : "false",
            result.useful ? "true" : "false",
            result.partial_candidate ? "true" : "false",
            (f1k_apply_enabled_ && (result.useful || can_apply_partial)) ? "false" : "true");
    }

    std::vector<RawKeyFrameId> CollectApplyAffectedKeyFrames(
        const OptimizationDryRunResult& dry_run) const
    {
        // 1K: el backup cubre variables y propagados antes de escribir en
        // GlobalPoseStore. 1L solo observara el resultado; si hay rechazo, la
        // restauracion sigue siendo una proteccion de apply, no logica nueva.
        std::vector<RawKeyFrameId> affected;
        affected.reserve(
            dry_run.proposed_vertex_poses.size() +
            dry_run.proposed_propagated_poses.size());
        for (const auto& proposal : dry_run.proposed_vertex_poses)
        {
            affected.push_back(proposal.keyframe_id);
        }
        for (const auto& proposal : dry_run.proposed_propagated_poses)
        {
            affected.push_back(proposal.keyframe_id);
        }
        return affected;
    }

    bool ShouldForceF1LReject(uint64_t task_id)
    {
        if (!f1l_debug_force_reject_once_ || f1l_debug_force_reject_consumed_)
        {
            return false;
        }
        if (f1l_debug_force_reject_task_id_ >= 0 &&
            task_id != static_cast<uint64_t>(f1l_debug_force_reject_task_id_))
        {
            return false;
        }
        f1l_debug_force_reject_consumed_ = true;
        return true;
    }

    void ApplyAndLogOptimizationResult(const FiducialOptimizationTask& task,
                                       const OptimizationDryRunResult& dry_run,
                                       const PoseGraphProblem& problem,
                                       const std::string& trigger)
    {
        const bool anchor_preservation_required =
            dry_run.anchor_preservation_required ||
            problem.anchor_preservation.required;
        const bool anchor_preservation_graph_satisfied =
            dry_run.anchor_preservation_graph_satisfied &&
            (!problem.anchor_preservation.required ||
             problem.anchor_preservation.satisfied);
        const uint64_t previous_fiducial_fixed_count =
            std::max(
                dry_run.previous_fiducial_fixed_count,
                problem.anchor_preservation.previous_fiducial_fixed_count);
        std::string anchor_precheck_reason = "not_required";
        bool anchor_precheck_ok = true;
        if (anchor_preservation_required)
        {
            anchor_precheck_reason = "ok";
            anchor_precheck_ok =
                anchor_preservation_graph_satisfied &&
                dry_run.anchor_preservation_ok &&
                previous_fiducial_fixed_count > 0 &&
                dry_run.fixed_kfs > 0 &&
                !dry_run.hard_fixed_moved;
            if (!anchor_preservation_graph_satisfied ||
                previous_fiducial_fixed_count == 0 ||
                dry_run.fixed_kfs == 0)
            {
                anchor_precheck_reason = "missing_previous_fiducial_anchor";
            }
            else if (!dry_run.anchor_preservation_ok || dry_run.hard_fixed_moved)
            {
                anchor_precheck_reason = "anchor_preservation_failed";
            }
        }
        // 1K: antes de crear backup o escribir en GlobalPoseStore,
        // exigimos que la evidencia producida por 1I/1J demuestre que el
        // fiducial previo del submapa esta fijo y no se movera con el apply.
        RCLCPP_WARN(
            get_logger(),
            "[F1K-OPT-APPLY-ANCHOR-PRESERVATION-PRECHECK] task_id=%llu ok=%s required=%s graph_satisfied=%s dryrun_ok=%s previous_fiducial_fixed_count=%llu fixed_kfs=%llu hard_fixed_moved=%s max_previous_fiducial_delta_t=%.9f max_previous_fiducial_delta_yaw=%.9f reason=%s",
            static_cast<unsigned long long>(dry_run.task_id),
            anchor_precheck_ok ? "true" : "false",
            anchor_preservation_required ? "true" : "false",
            anchor_preservation_graph_satisfied ? "true" : "false",
            dry_run.anchor_preservation_ok ? "true" : "false",
            static_cast<unsigned long long>(previous_fiducial_fixed_count),
            static_cast<unsigned long long>(dry_run.fixed_kfs),
            dry_run.hard_fixed_moved ? "true" : "false",
            dry_run.max_previous_fiducial_delta_t,
            dry_run.max_previous_fiducial_delta_yaw,
            anchor_precheck_reason.c_str());
        if (!anchor_precheck_ok)
        {
            RCLCPP_ERROR(
                get_logger(),
                "[F1K-OPT-APPLY-PRECHECK-FAIL] task_id=%llu reason=%s",
                static_cast<unsigned long long>(dry_run.task_id),
                anchor_precheck_reason.c_str());
            return;
        }

        // 1K: antes de cruzar la frontera persistente, guardamos el estado
        // afectado. Si la validacion post-apply rechaza, GlobalPoseStore puede
        // volver exactamente al punto anterior sin tocar RawMapDatabase.
        const auto raw_stats_before = raw_db_.GetDatabaseStats();
        const auto pose_stats_before = pose_store_.GetPoseStoreStats();
        const GlobalMapBuildResult map_before =
            global_map_builder_.Build(
                raw_db_,
                pose_store_,
                score_manager_,
                static_cast<float>(global_map_min_score_to_publish_));
        const auto affected_keyframes = CollectApplyAffectedKeyFrames(dry_run);
        const DebugGtWindowStats gt_before =
            LogGtWindowErrors(dry_run.task_id, affected_keyframes, false);
        const std::vector<RawKeyFrameId> gt_collateral_keyframes =
            CollectGtCollateralKeyFrames(problem);
        const auto backup =
            pose_store_.CreateApplyBackup(dry_run.task_id, affected_keyframes);
        RCLCPP_WARN(
            get_logger(),
            "[F1L-POSESTORE-BACKUP-CREATED] task_id=%llu ok=%s affected_kfs=%llu affected_submaps=%llu reason=%s",
            static_cast<unsigned long long>(dry_run.task_id),
            backup.success ? "true" : "false",
            static_cast<unsigned long long>(backup.affected_keyframes),
            static_cast<unsigned long long>(backup.affected_submaps),
            backup.reason.c_str());
        if (!backup.success)
        {
            RCLCPP_ERROR(
                get_logger(),
                "[F1K-OPT-APPLY-PRECHECK-FAIL] task_id=%llu reason=f1l_backup_failed_%s",
                static_cast<unsigned long long>(dry_run.task_id),
                backup.reason.c_str());
            return;
        }
        const bool allow_partial_candidate =
            f1l_validation_enabled_ && f1l_partial_apply_enabled_;
        const OptimizationApplyResult apply =
            optimization_manager_.ApplyCandidateResult(
                dry_run,
                problem,
                raw_db_,
                pose_store_,
                allow_partial_candidate);
        const auto raw_stats_after = raw_db_.GetDatabaseStats();
        const auto pose_stats_after = pose_store_.GetPoseStoreStats();
        const bool raw_unchanged = SameRawStats(raw_stats_before, raw_stats_after);
        const DebugGtWindowStats gt_after =
            LogGtWindowErrors(dry_run.task_id, affected_keyframes, true);
        LogGtWindowComparison(dry_run.task_id, gt_before, gt_after);
        LogGtCollateralCheck(
            dry_run.task_id,
            gt_before,
            gt_after,
            gt_collateral_keyframes);

        RCLCPP_WARN(
            get_logger(),
            "[F1K-OPT-APPLY-PRECHECK] task_id=%llu ok=%s useful=%s partial_candidate=%s moved_kfs=%llu propagated=%llu fixed=%llu reason=%s trigger=%s",
            static_cast<unsigned long long>(dry_run.task_id),
            apply.precheck_ok ? "true" : "false",
            dry_run.useful ? "true" : "false",
            dry_run.partial_candidate ? "true" : "false",
            static_cast<unsigned long long>(dry_run.moved_kfs),
            static_cast<unsigned long long>(dry_run.propagated_kfs),
            static_cast<unsigned long long>(dry_run.fixed_kfs),
            apply.reason.c_str(),
            trigger.c_str());
        if (!apply.precheck_ok)
        {
            RCLCPP_ERROR(
                get_logger(),
                "[F1K-OPT-APPLY-PRECHECK-FAIL] task_id=%llu reason=%s",
                static_cast<unsigned long long>(dry_run.task_id),
                apply.reason.c_str());
        }

        for (const auto& record : apply.optimized_records)
        {
            RCLCPP_WARN(
                get_logger(),
                "[F1K-OPT-APPLY-KF] task_id=%llu drone_id=%u epoch=%llu kf=%llu source=%s applied=%s delta_t=%.6f delta_yaw=%.6f reason=%s",
                static_cast<unsigned long long>(dry_run.task_id),
                record.keyframe_id.drone_id,
                static_cast<unsigned long long>(record.keyframe_id.map_epoch),
                static_cast<unsigned long long>(record.keyframe_id.local_kf_id),
                record.source.c_str(),
                record.applied ? "true" : "false",
                record.delta_t_m,
                record.delta_yaw_rad,
                record.reason.c_str());
            if (record.applied)
            {
                RCLCPP_WARN(
                    get_logger(),
                    "[F1K-POSESTORE-OPTIMIZED-POSE-SET] task_id=%llu drone_id=%u epoch=%llu kf=%llu source=%s delta_t=%.6f delta_yaw=%.6f",
                    static_cast<unsigned long long>(dry_run.task_id),
                    record.keyframe_id.drone_id,
                    static_cast<unsigned long long>(record.keyframe_id.map_epoch),
                    static_cast<unsigned long long>(record.keyframe_id.local_kf_id),
                    record.source.c_str(),
                    record.delta_t_m,
                    record.delta_yaw_rad);
                RCLCPP_WARN(
                    get_logger(),
                    "[F1K-POSESTORE-CORRECTION-SET] task_id=%llu drone_id=%u epoch=%llu kf=%llu correction_t=%.6f correction_yaw=%.6f source=%s",
                    static_cast<unsigned long long>(dry_run.task_id),
                    record.keyframe_id.drone_id,
                    static_cast<unsigned long long>(record.keyframe_id.map_epoch),
                    static_cast<unsigned long long>(record.keyframe_id.local_kf_id),
                    record.correction_t_m,
                    record.correction_yaw_rad,
                    record.source.c_str());
            }
        }

        for (const auto& record : apply.propagated_records)
        {
            RCLCPP_WARN(
                get_logger(),
                "[F1K-OPT-APPLY-PROPAGATED-KF] task_id=%llu drone_id=%u epoch=%llu kf=%llu source=%s applied=%s delta_t=%.6f delta_yaw=%.6f reason=%s",
                static_cast<unsigned long long>(dry_run.task_id),
                record.keyframe_id.drone_id,
                static_cast<unsigned long long>(record.keyframe_id.map_epoch),
                static_cast<unsigned long long>(record.keyframe_id.local_kf_id),
                record.source.c_str(),
                record.applied ? "true" : "false",
                record.delta_t_m,
                record.delta_yaw_rad,
                record.reason.c_str());
            if (record.applied)
            {
                RCLCPP_WARN(
                    get_logger(),
                    "[F1K-POSESTORE-PROPAGATED-POSE-SET] task_id=%llu drone_id=%u epoch=%llu kf=%llu source=%s delta_t=%.6f delta_yaw=%.6f",
                    static_cast<unsigned long long>(dry_run.task_id),
                    record.keyframe_id.drone_id,
                    static_cast<unsigned long long>(record.keyframe_id.map_epoch),
                    static_cast<unsigned long long>(record.keyframe_id.local_kf_id),
                    record.source.c_str(),
                    record.delta_t_m,
                    record.delta_yaw_rad);
                RCLCPP_WARN(
                    get_logger(),
                    "[F1K-POSESTORE-CORRECTION-SET] task_id=%llu drone_id=%u epoch=%llu kf=%llu correction_t=%.6f correction_yaw=%.6f source=%s",
                    static_cast<unsigned long long>(dry_run.task_id),
                    record.keyframe_id.drone_id,
                    static_cast<unsigned long long>(record.keyframe_id.map_epoch),
                    static_cast<unsigned long long>(record.keyframe_id.local_kf_id),
                    record.correction_t_m,
                    record.correction_yaw_rad,
                    record.source.c_str());
            }
        }

        if (apply.skipped_propagated_kfs > 0)
        {
            RCLCPP_WARN(
                get_logger(),
                "[F1L-PARTIAL-PROPAGATION-SKIP] task_id=%llu skipped_propagated_kfs=%llu reason=partial_checkpoint_requires_graph_rebuild_before_broad_propagation",
                static_cast<unsigned long long>(dry_run.task_id),
                static_cast<unsigned long long>(apply.skipped_propagated_kfs));
        }

        if (apply.last_correction_set)
        {
            RCLCPP_WARN(
                get_logger(),
                "[F1K-POSESTORE-LAST-CORRECTION-SET] task_id=%llu submap=(%u,%llu) from_kf=%llu last_correction_set=true",
                static_cast<unsigned long long>(dry_run.task_id),
                apply.last_correction_submap.drone_id,
                static_cast<unsigned long long>(apply.last_correction_submap.map_epoch),
                static_cast<unsigned long long>(apply.last_correction_from_keyframe.local_kf_id));
        }

        RCLCPP_WARN(
            get_logger(),
            "[F1K-RAWDB-NOT-MODIFIED] task_id=%llu ok=%s before_journal=%llu after_journal=%llu before_kfs=%llu after_kfs=%llu before_mps=%llu after_mps=%llu",
            static_cast<unsigned long long>(dry_run.task_id),
            raw_unchanged ? "true" : "false",
            static_cast<unsigned long long>(raw_stats_before.journal_entries),
            static_cast<unsigned long long>(raw_stats_after.journal_entries),
            static_cast<unsigned long long>(raw_stats_before.keyframes),
            static_cast<unsigned long long>(raw_stats_after.keyframes),
            static_cast<unsigned long long>(raw_stats_before.mappoints),
            static_cast<unsigned long long>(raw_stats_after.mappoints));
        RCLCPP_WARN(
            get_logger(),
            "[F1K-OPT-APPLY-SUMMARY] task_id=%llu applied=%s reason=%s optimized_kfs=%llu propagated_kfs=%llu skipped_propagated_kfs=%llu pending_rebased=%llu fixed_kfs=%llu hard_fixed_moved=%s max_delta_t=%.6f mean_delta_t=%.6f max_delta_yaw=%.6f last_correction_set=%s raw_db_modified=%s pose_store_before_optimized=%llu pose_store_after_optimized=%llu pose_store_after_propagated=%llu",
            static_cast<unsigned long long>(dry_run.task_id),
            apply.applied ? "true" : "false",
            apply.reason.c_str(),
            static_cast<unsigned long long>(apply.optimized_kfs),
            static_cast<unsigned long long>(apply.propagated_kfs),
            static_cast<unsigned long long>(apply.skipped_propagated_kfs),
            static_cast<unsigned long long>(apply.pending_rebased_kfs),
            static_cast<unsigned long long>(apply.fixed_kfs),
            apply.hard_fixed_moved ? "true" : "false",
            apply.max_delta_t,
            apply.mean_delta_t,
            apply.max_delta_yaw,
            apply.last_correction_set ? "true" : "false",
            raw_unchanged ? "false" : "true",
            static_cast<unsigned long long>(pose_stats_before.optimized_keyframes),
            static_cast<unsigned long long>(pose_stats_after.optimized_keyframes),
            static_cast<unsigned long long>(pose_stats_after.propagated_keyframes));
        RCLCPP_WARN(
            get_logger(),
            "[F1K-OPT-APPLY-RESULT] task_id=%llu applied=%s reason=%s raw_db_modified=%s global_pose_store_modified=%s",
            static_cast<unsigned long long>(dry_run.task_id),
            apply.applied ? "true" : "false",
            apply.reason.c_str(),
            raw_unchanged ? "false" : "true",
            apply.global_pose_store_modified ? "true" : "false");

        if (!apply.applied)
        {
            pose_store_.ConfirmApply(dry_run.task_id);
            return;
        }

        RCLCPP_WARN(
            get_logger(),
            "[F1K-GLOBALMAP-REBUILD-AFTER-APPLY] task_id=%llu reason=apply_success_pending_f1l_validation",
            static_cast<unsigned long long>(dry_run.task_id));

        if (!f1l_validation_enabled_)
        {
            RCLCPP_WARN(
                get_logger(),
                "[F1L-POST-APPLY-START] task_id=%llu enabled=false reason=validation_disabled",
                static_cast<unsigned long long>(dry_run.task_id));
            pose_store_.ConfirmApply(dry_run.task_id);
            PublishGlobalSparseCloud("f1k_apply");
            RCLCPP_WARN(
                get_logger(),
                "[F1K-GLOBALMAP-PUBLISH-AFTER-APPLY] task_id=%llu topic=%s frame_id=%s",
                static_cast<unsigned long long>(dry_run.task_id),
                global_sparse_cloud_topic_.c_str(),
                world_frame_.c_str());
            return;
        }

        const bool force_reject = ShouldForceF1LReject(dry_run.task_id);
        RCLCPP_WARN(
            get_logger(),
            "[F1L-POST-APPLY-START] task_id=%llu type=%s useful=%s partial_candidate=%s forced_reject=%s",
            static_cast<unsigned long long>(dry_run.task_id),
            dry_run.task_type.c_str(),
            dry_run.useful ? "true" : "false",
            dry_run.partial_candidate ? "true" : "false",
            force_reject ? "true" : "false");

        PostApplyValidationResult validation =
            optimization_manager_.ValidatePostApply(
                dry_run,
                problem,
                apply,
                pose_store_,
                force_reject);

        const GlobalMapBuildResult map_after =
            global_map_builder_.Build(
                raw_db_,
                pose_store_,
                score_manager_,
                static_cast<float>(global_map_min_score_to_publish_));
        const bool global_map_ok =
            (map_before.stats.returned_points == 0 ||
             map_after.stats.returned_points > 0) &&
            map_after.stats.invalid_pose_skipped <=
                map_before.stats.invalid_pose_skipped + 100;

        RCLCPP_WARN(
            get_logger(),
            "[F1L-POST-APPLY-ERROR] task_id=%llu type=%s before_t=%.6f predicted_after_t=%.6f real_after_t=%.6f before_yaw=%.6f predicted_after_yaw=%.6f real_after_yaw=%.6f improvement_ratio=%.6f",
            static_cast<unsigned long long>(dry_run.task_id),
            dry_run.task_type.c_str(),
            validation.error.before_error_t,
            validation.error.predicted_after_error_t,
            validation.error.real_after_error_t,
            validation.error.before_error_yaw,
            validation.error.predicted_after_error_yaw,
            validation.error.real_after_error_yaw,
            validation.error.improvement_ratio);
        RCLCPP_WARN(
            get_logger(),
            "[F1L-PARTIAL-ABSURD-ERROR-POLICY] task_id=%llu enabled=true absurd_threshold_t=%.6f before_t=%.6f real_after_t=%.6f improvement_ratio=%.6f partial_candidate=%s",
            static_cast<unsigned long long>(dry_run.task_id),
            f1l_post_apply_fiducial_absurd_error_t_,
            validation.error.before_error_t,
            validation.error.real_after_error_t,
            validation.error.improvement_ratio,
            dry_run.partial_candidate ? "true" : "false");
        RCLCPP_WARN(
            get_logger(),
            "[F1L-INTERNAL-EDGE-CLASSIFY] task_id=%llu strong_edges_checked=%llu deformable_edges_checked=%llu strong_edges_broken=%llu deformable_edges_broken=%llu max_strong_internal_after=%.6f max_deformable_internal_after=%.6f",
            static_cast<unsigned long long>(dry_run.task_id),
            static_cast<unsigned long long>(
                validation.internal_error.strong_edges_checked),
            static_cast<unsigned long long>(
                validation.internal_error.deformable_edges_checked),
            static_cast<unsigned long long>(
                validation.internal_error.strong_edges_broken),
            static_cast<unsigned long long>(
                validation.internal_error.deformable_edges_broken),
            validation.internal_error.max_strong_internal_after,
            validation.internal_error.max_deformable_internal_after);
        RCLCPP_WARN(
            get_logger(),
            "[F1L-POST-APPLY-INTERNAL-ERROR] task_id=%llu ok=%s internal_mean_before=%.6f internal_mean_after=%.6f internal_max_before=%.6f internal_max_after=%.6f num_edges_worse=%llu num_edges_broken=%llu strong_edges_broken=%llu deformable_edges_broken=%llu",
            static_cast<unsigned long long>(dry_run.task_id),
            validation.internal_edges_ok ? "true" : "false",
            validation.internal_error.internal_mean_before,
            validation.internal_error.internal_mean_after,
            validation.internal_error.internal_max_before,
            validation.internal_error.internal_max_after,
            static_cast<unsigned long long>(validation.internal_error.num_edges_worse),
            static_cast<unsigned long long>(validation.internal_error.num_edges_broken),
            static_cast<unsigned long long>(
                validation.internal_error.strong_edges_broken),
            static_cast<unsigned long long>(
                validation.internal_error.deformable_edges_broken));
        RCLCPP_WARN(
            get_logger(),
            "[F1L-POST-APPLY-FIXED-CHECK] task_id=%llu ok=%s hard_fixed_moved=%s fixed_moved_count=%llu max_fixed_delta_t=%.9f max_fixed_delta_yaw=%.9f",
            static_cast<unsigned long long>(dry_run.task_id),
            validation.fixed_ok ? "true" : "false",
            validation.fixed_check.hard_fixed_moved ? "true" : "false",
            static_cast<unsigned long long>(validation.fixed_check.fixed_moved_count),
            validation.fixed_check.max_fixed_delta_t,
            validation.fixed_check.max_fixed_delta_yaw);
        RCLCPP_WARN(
            get_logger(),
            "[F1L-ANCHOR-PRESERVATION-CHECK] task_id=%llu ok=%s required=%s graph_satisfied=%s previous_fiducial_fixed_count=%llu previous_fiducial_neighborhood_fixed_count=%llu independent_branches=%llu checked_branch_anchors=%llu subdivision_candidates=%llu max_previous_fiducial_delta_t=%.9f max_previous_fiducial_delta_yaw=%.9f max_previous_fiducial_neighborhood_delta_t=%.9f max_previous_fiducial_neighborhood_delta_yaw=%.9f reason=%s",
            static_cast<unsigned long long>(dry_run.task_id),
            validation.anchor_preservation_ok ? "true" : "false",
            validation.anchor_preservation_check.required ? "true" : "false",
            validation.anchor_preservation_check.graph_satisfied ? "true" : "false",
            static_cast<unsigned long long>(
                validation.anchor_preservation_check.previous_fiducial_fixed_count),
            static_cast<unsigned long long>(
                validation.anchor_preservation_check
                    .previous_fiducial_neighborhood_fixed_count),
            static_cast<unsigned long long>(
                validation.anchor_preservation_check.independent_branches),
            static_cast<unsigned long long>(
                validation.anchor_preservation_check.checked_branch_anchors),
            static_cast<unsigned long long>(
                validation.anchor_preservation_check.subdivision_candidates),
            validation.anchor_preservation_check.max_previous_fiducial_delta_t,
            validation.anchor_preservation_check.max_previous_fiducial_delta_yaw,
            validation.anchor_preservation_check
                .max_previous_fiducial_neighborhood_delta_t,
            validation.anchor_preservation_check
                .max_previous_fiducial_neighborhood_delta_yaw,
            validation.anchor_preservation_check.reason.c_str());
        if (!validation.anchor_preservation_ok)
        {
            RCLCPP_ERROR(
                get_logger(),
                "[F1L-ANCHOR-PRESERVATION-FAIL] task_id=%llu reason=%s",
                static_cast<unsigned long long>(dry_run.task_id),
                validation.anchor_preservation_check.reason.c_str());
        }
        RCLCPP_WARN(
            get_logger(),
            "[F1L-POST-APPLY-PROPAGATION-CHECK] task_id=%llu ok=%s propagated_count=%llu skipped_propagated_count=%llu rebased_count=%llu propagated_max_delta_t=%.6f propagated_mean_delta_t=%.6f propagation_discontinuity_max_t=%.9f propagation_discontinuity_max_yaw=%.9f",
            static_cast<unsigned long long>(dry_run.task_id),
            validation.propagation_ok ? "true" : "false",
            static_cast<unsigned long long>(
                validation.propagation_check.propagated_count),
            static_cast<unsigned long long>(
                validation.propagation_check.skipped_propagated_count),
            static_cast<unsigned long long>(
                validation.propagation_check.rebased_count),
            validation.propagation_check.propagated_max_delta_t,
            validation.propagation_check.propagated_mean_delta_t,
            validation.propagation_check.propagation_discontinuity_max_t,
            validation.propagation_check.propagation_discontinuity_max_yaw);
        RCLCPP_WARN(
            get_logger(),
            "[F1L-POST-APPLY-GLOBALMAP-CHECK] task_id=%llu ok=%s published_points_before=%llu published_points_after=%llu server_corrected_before=%llu server_corrected_after=%llu score_min=%.3f score_mean=%.3f score_max=%.3f nan_points=0 invalid_pose_skipped_before=%llu invalid_pose_skipped_after=%llu anchored_submaps=%llu",
            static_cast<unsigned long long>(dry_run.task_id),
            global_map_ok ? "true" : "false",
            static_cast<unsigned long long>(map_before.stats.returned_points),
            static_cast<unsigned long long>(map_after.stats.returned_points),
            static_cast<unsigned long long>(map_before.stats.server_corrected_points),
            static_cast<unsigned long long>(map_after.stats.server_corrected_points),
            map_after.stats.score_min,
            map_after.stats.score_mean,
            map_after.stats.score_max,
            static_cast<unsigned long long>(map_before.stats.invalid_pose_skipped),
            static_cast<unsigned long long>(map_after.stats.invalid_pose_skipped),
            static_cast<unsigned long long>(map_after.stats.anchored_submaps));
        RCLCPP_WARN(
            get_logger(),
            "[F1L-GLOBALMAP-KF-PROJECTION] task_id=%llu keyframe_projected_before=%llu keyframe_projected_after=%llu fallback_submap_before=%llu fallback_submap_after=%llu",
            static_cast<unsigned long long>(dry_run.task_id),
            static_cast<unsigned long long>(map_before.stats.keyframe_projected_points),
            static_cast<unsigned long long>(map_after.stats.keyframe_projected_points),
            static_cast<unsigned long long>(map_before.stats.fallback_submap_points),
            static_cast<unsigned long long>(map_after.stats.fallback_submap_points));

        PostApplyDecision final_decision = validation.decision;
        std::string final_reason = validation.reason;
        if (!global_map_ok && final_decision != PostApplyDecision::RejectRollback)
        {
            final_decision = PostApplyDecision::RejectRollback;
            final_reason = "global_map_check_failed";
        }

        if (final_decision == PostApplyDecision::Accept)
        {
            f1l_partial_checkpoint_task_by_submap_.erase(problem.submap_id);
            f1l_partial_retry_count_by_submap_.erase(problem.submap_id);
            RCLCPP_WARN(
                get_logger(),
                "[F1L-POST-APPLY-ACCEPT] task_id=%llu reason=%s real_after_t=%.6f threshold_t=%.6f decision=%s",
                static_cast<unsigned long long>(dry_run.task_id),
                final_reason.c_str(),
                validation.error.real_after_error_t,
                f1j_dryrun_max_final_error_t_,
                orbslam3_multi::ToString(final_decision));
            const bool confirmed = pose_store_.ConfirmApply(dry_run.task_id);
            RCLCPP_WARN(
                get_logger(),
                "[F1L-POSESTORE-COMMIT-CONFIRMED] task_id=%llu ok=%s decision=ACCEPT",
                static_cast<unsigned long long>(dry_run.task_id),
                confirmed ? "true" : "false");
            PublishGlobalSparseCloud("f1l_accept");
            RCLCPP_WARN(
                get_logger(),
                "[F1K-GLOBALMAP-PUBLISH-AFTER-APPLY] task_id=%llu topic=%s frame_id=%s decision=ACCEPT",
                static_cast<unsigned long long>(dry_run.task_id),
                global_sparse_cloud_topic_.c_str(),
                world_frame_.c_str());
            return;
        }

        if (final_decision == PostApplyDecision::PartialKeepForNextPass)
        {
            auto retry_it =
                f1l_partial_retry_count_by_submap_.find(problem.submap_id);
            const uint64_t retry_count =
                retry_it == f1l_partial_retry_count_by_submap_.end()
                    ? 0U
                    : retry_it->second + 1U;
            f1l_partial_checkpoint_task_by_submap_[problem.submap_id] =
                dry_run.task_id;
            f1l_partial_retry_count_by_submap_[problem.submap_id] = retry_count;
            RCLCPP_WARN(
                get_logger(),
                "[F1L-POST-APPLY-PARTIAL] task_id=%llu reason=%s partial_reason=%s real_after_t=%.6f threshold_t=%.6f needs_second_pass=true partial_pending=true retry_required=true retry_strategy=%s decision=%s",
                static_cast<unsigned long long>(dry_run.task_id),
                final_reason.c_str(),
                dry_run.partial_reason.empty()
                    ? "partial_real_improvement_safe"
                    : dry_run.partial_reason.c_str(),
                validation.error.real_after_error_t,
                f1j_dryrun_max_final_error_t_,
                validation.retry.strategy.c_str(),
                orbslam3_multi::ToString(final_decision));
            RCLCPP_WARN(
                get_logger(),
                "[F1L-PARTIAL-PENDING] task_id=%llu retry_required=true graph_rebuild_required=true retry_count=%llu max_retries=%d retry_strategy=%s reason=%s",
                static_cast<unsigned long long>(dry_run.task_id),
                static_cast<unsigned long long>(retry_count),
                f1l_max_partial_retries_,
                validation.retry.strategy.c_str(),
                final_reason.c_str());
            RCLCPP_WARN(
                get_logger(),
                "[F1L-PARTIAL-CHECKPOINT] task_id=%llu checkpoint_saved=true rollback_target=partial_checkpoint safe=true inherit_submap_last_correction=false real_after_t=%.6f before_t=%.6f strong_edges_broken=%llu deformable_edges_broken=%llu",
                static_cast<unsigned long long>(dry_run.task_id),
                validation.error.real_after_error_t,
                validation.error.before_error_t,
                static_cast<unsigned long long>(
                    validation.internal_error.strong_edges_broken),
                static_cast<unsigned long long>(
                    validation.internal_error.deformable_edges_broken));
            RCLCPP_WARN(
                get_logger(),
                "[F1L-RETRY-SUGGESTION] task_id=%llu strategy=%s retry_allowed=%s reason=%s",
                static_cast<unsigned long long>(dry_run.task_id),
                validation.retry.strategy.c_str(),
                validation.retry.retry_allowed ? "true" : "false",
                validation.retry.reason.c_str());
            const bool confirmed = pose_store_.ConfirmApply(dry_run.task_id);
            RCLCPP_WARN(
                get_logger(),
                "[F1L-POSESTORE-COMMIT-CONFIRMED] task_id=%llu ok=%s decision=PARTIAL_KEEP_FOR_NEXT_PASS",
                static_cast<unsigned long long>(dry_run.task_id),
                confirmed ? "true" : "false");
            PublishGlobalSparseCloud("f1l_partial");
            RCLCPP_WARN(
                get_logger(),
                "[F1K-GLOBALMAP-PUBLISH-AFTER-APPLY] task_id=%llu topic=%s frame_id=%s decision=PARTIAL_KEEP_FOR_NEXT_PASS",
                static_cast<unsigned long long>(dry_run.task_id),
                global_sparse_cloud_topic_.c_str(),
                world_frame_.c_str());
            if (confirmed &&
                retry_count < static_cast<uint64_t>(f1l_max_partial_retries_))
            {
                // 1L: la segunda pasada debe usar el mismo fiducial objetivo y
                // las poses del checkpoint recien escrito. Esperar a otro KF
                // crearia una tarea nueva desde una pose raw sin herencia y
                // volveria artificialmente al error absurdo original.
                RCLCPP_WARN(
                    get_logger(),
                    "[F1L-PARTIAL-RETRY-START] task_id=%llu retry_count=%llu max_retries=%d rebuild_same_task=true source=GlobalPoseStore_partial_checkpoint",
                    static_cast<unsigned long long>(dry_run.task_id),
                    static_cast<unsigned long long>(retry_count + 1U),
                    f1l_max_partial_retries_);
                BuildAndLogPoseGraphForTask(
                    task,
                    "f1l_partial_retry_from_checkpoint",
                    false);
            }
            return;
        }

        RCLCPP_ERROR(
            get_logger(),
            "[F1L-POST-APPLY-REJECT] task_id=%llu reason=%s real_after_t=%.6f before_t=%.6f decision=%s",
            static_cast<unsigned long long>(dry_run.task_id),
            final_reason.c_str(),
            validation.error.real_after_error_t,
            validation.error.before_error_t,
            orbslam3_multi::ToString(final_decision));
        const auto rollback = pose_store_.RestoreApplyBackup(dry_run.task_id);
        RCLCPP_WARN(
            get_logger(),
            "[F1L-POSESTORE-ROLLBACK] task_id=%llu ok=%s restored_world=%llu removed_world=%llu restored_optimized=%llu removed_optimized=%llu restored_submap_corrections=%llu removed_submap_corrections=%llu reason=%s",
            static_cast<unsigned long long>(dry_run.task_id),
            rollback.success ? "true" : "false",
            static_cast<unsigned long long>(rollback.restored_world_poses),
            static_cast<unsigned long long>(rollback.removed_world_poses),
            static_cast<unsigned long long>(rollback.restored_optimized_poses),
            static_cast<unsigned long long>(rollback.removed_optimized_poses),
            static_cast<unsigned long long>(rollback.restored_submap_corrections),
            static_cast<unsigned long long>(rollback.removed_submap_corrections),
            rollback.reason.c_str());
        RCLCPP_WARN(
            get_logger(),
            "[F1L-GLOBALMAP-REBUILD-AFTER-ROLLBACK] task_id=%llu reason=%s",
            static_cast<unsigned long long>(dry_run.task_id),
            final_reason.c_str());
        PublishGlobalSparseCloud("f1l_rollback");
        RCLCPP_WARN(
            get_logger(),
            "[F1L-GLOBALMAP-PUBLISH-AFTER-ROLLBACK] task_id=%llu topic=%s frame_id=%s",
            static_cast<unsigned long long>(dry_run.task_id),
            global_sparse_cloud_topic_.c_str(),
            world_frame_.c_str());
        RCLCPP_WARN(
            get_logger(),
            "[F1L-ROLLBACK-DIAGNOSTIC] task_id=%llu type=%s reason=%s before_t=%.6f predicted_after_t=%.6f real_after_t=%.6f internal_max_before=%.6f internal_max_after=%.6f hard_fixed_moved=%s propagation_ok=%s globalmap_ok=%s",
            static_cast<unsigned long long>(dry_run.task_id),
            dry_run.task_type.c_str(),
            final_reason.c_str(),
            validation.error.before_error_t,
            validation.error.predicted_after_error_t,
            validation.error.real_after_error_t,
            validation.internal_error.internal_max_before,
            validation.internal_error.internal_max_after,
            validation.fixed_check.hard_fixed_moved ? "true" : "false",
            validation.propagation_ok ? "true" : "false",
            global_map_ok ? "true" : "false");
        RCLCPP_WARN(
            get_logger(),
            "[F1L-RETRY-SUGGESTION] task_id=%llu strategy=%s retry_allowed=%s reason=%s",
            static_cast<unsigned long long>(dry_run.task_id),
            validation.retry.strategy.c_str(),
            validation.retry.retry_allowed ? "true" : "false",
            validation.retry.reason.c_str());
    }

    void LogPoseGraphProblem(const PoseGraphProblem& problem,
                             const std::string& reason)
    {
        uint64_t min_kf = problem.target_keyframe_id.local_kf_id;
        uint64_t max_kf = problem.target_keyframe_id.local_kf_id;
        for (const auto& vertex : problem.vertices)
        {
            min_kf = std::min<uint64_t>(min_kf, vertex.keyframe_id.local_kf_id);
            max_kf = std::max<uint64_t>(max_kf, vertex.keyframe_id.local_kf_id);
        }
        for (const auto& keyframe_id : problem.affected_non_variable_keyframes)
        {
            min_kf = std::min<uint64_t>(min_kf, keyframe_id.local_kf_id);
            max_kf = std::max<uint64_t>(max_kf, keyframe_id.local_kf_id);
        }

        RCLCPP_WARN(
            get_logger(),
            "[F1I-GRAPH-WINDOW] task_id=%llu drone_id=%u epoch=%llu target_kf=%llu local_kf_range=[%llu,%llu] vertices=%llu affected_non_vertices=%llu anchor_stop=%s rebuilt_from_partial=%s partial_parent_task_id=%llu partial_retry_count=%llu",
            static_cast<unsigned long long>(problem.task_id),
            problem.submap_id.drone_id,
            static_cast<unsigned long long>(problem.submap_id.map_epoch),
            static_cast<unsigned long long>(problem.target_keyframe_id.local_kf_id),
            static_cast<unsigned long long>(min_kf),
            static_cast<unsigned long long>(max_kf),
            static_cast<unsigned long long>(problem.vertices.size()),
            static_cast<unsigned long long>(problem.affected_non_variable_keyframes.size()),
            pose_graph_anchor_stop_enabled_ ? "true" : "false",
            problem.rebuilt_from_partial_checkpoint ? "true" : "false",
            static_cast<unsigned long long>(problem.partial_parent_task_id),
            static_cast<unsigned long long>(problem.partial_retry_count));

        const auto config = pose_graph_builder_.GetConfig();
        uint64_t corner_vertices = 0;
        uint64_t split_vertices = 0;
        uint64_t target_neighborhood_vertices = 0;
        for (const auto& vertex : problem.vertices)
        {
            if (vertex.selection_reason == "corner_yaw_vertex")
            {
                ++corner_vertices;
            }
            if (vertex.selection_reason == "long_edge_split_vertex")
            {
                ++split_vertices;
            }
            if (vertex.selection_reason == "target_fiducial_neighborhood")
            {
                ++target_neighborhood_vertices;
            }
        }
        RCLCPP_WARN(
            get_logger(),
            "[F1I-GRAPH-VERTEX-COVERAGE] task_id=%llu vertices=%llu max_vertices=%llu corner_vertices=%llu long_edge_split_vertices=%llu target_neighborhood_vertices=%llu previous_fiducial_neighborhood_vertices=%llu max_edge_kf_gap=%llu max_allowed_edge_kf_gap=%llu max_edge_length_m=%.3f max_allowed_edge_length_m=%.3f uncovered_long_segments=%llu coverage_complete=%s",
            static_cast<unsigned long long>(problem.task_id),
            static_cast<unsigned long long>(problem.vertices.size()),
            static_cast<unsigned long long>(config.max_vertices),
            static_cast<unsigned long long>(corner_vertices),
            static_cast<unsigned long long>(split_vertices),
            static_cast<unsigned long long>(target_neighborhood_vertices),
            static_cast<unsigned long long>(
                problem.anchor_preservation
                    .previous_fiducial_neighborhood_fixed_count),
            static_cast<unsigned long long>(problem.coverage.max_control_span_kfs),
            static_cast<unsigned long long>(config.max_temporal_edge_kf_gap),
            problem.coverage.max_control_span_m,
            config.max_temporal_edge_length_m,
            static_cast<unsigned long long>(
                problem.coverage.uncovered_long_segments),
            problem.coverage.coverage_complete ? "true" : "false");

        RCLCPP_WARN(
            get_logger(),
            "[F1I-FID-CONNECTIVITY-BRANCHES] task_id=%llu required=%s satisfied=%s independent_branches=%llu branch_anchor_count=%llu previous_fiducial_fixed_count=%llu previous_fiducial_neighborhood_fixed_count=%llu subdivision_candidates=%llu subdivided_confirmed=%llu reason=%s",
            static_cast<unsigned long long>(problem.task_id),
            problem.anchor_preservation.required ? "true" : "false",
            problem.anchor_preservation.satisfied ? "true" : "false",
            static_cast<unsigned long long>(
                problem.anchor_preservation.independent_branches),
            static_cast<unsigned long long>(
                problem.anchor_preservation.branch_anchor_count),
            static_cast<unsigned long long>(
                problem.anchor_preservation.previous_fiducial_fixed_count),
            static_cast<unsigned long long>(
                problem.anchor_preservation
                    .previous_fiducial_neighborhood_fixed_count),
            static_cast<unsigned long long>(
                problem.anchor_preservation.subdivision_candidates),
            static_cast<unsigned long long>(
                problem.anchor_preservation.subdivided_confirmed),
            problem.anchor_preservation.reason.c_str());

        for (const auto& edge : problem.fiducial_connectivity_edges)
        {
            RCLCPP_WARN(
                get_logger(),
                "[F1I-FID-CONNECTIVITY-EDGE] task_id=%llu from_fid=%d to_fid=%d from_kf=%llu to_kf=%llu status=%s selected_anchor=%s independent_branch=%s kf_gap=%llu reason=%s",
                static_cast<unsigned long long>(problem.task_id),
                edge.from_fiducial_id,
                edge.to_fiducial_id,
                static_cast<unsigned long long>(edge.from_keyframe_id.local_kf_id),
                static_cast<unsigned long long>(edge.to_keyframe_id.local_kf_id),
                orbslam3_multi::ToString(edge.status),
                edge.selected_as_branch_anchor ? "true" : "false",
                edge.independent_branch ? "true" : "false",
                static_cast<unsigned long long>(edge.kf_gap),
                edge.reason.c_str());
            RCLCPP_WARN(
                get_logger(),
                "[F1I-FID-CONNECTIVITY-SUBDIVISION] task_id=%llu from_fid=%d to_fid=%d edge_status=%s reason=%s",
                static_cast<unsigned long long>(problem.task_id),
                edge.from_fiducial_id,
                edge.to_fiducial_id,
                orbslam3_multi::ToString(edge.status),
                edge.reason.c_str());
        }

        RCLCPP_WARN(
            get_logger(),
            "[F1I-GRAPH-ANCHOR-PRESERVATION] task_id=%llu required=%s satisfied=%s previous_fiducial_fixed_count=%llu previous_fiducial_neighborhood_fixed_count=%llu branch_anchor_count=%llu independent_branches=%llu",
            static_cast<unsigned long long>(problem.task_id),
            problem.anchor_preservation.required ? "true" : "false",
            problem.anchor_preservation.satisfied ? "true" : "false",
            static_cast<unsigned long long>(
                problem.anchor_preservation.previous_fiducial_fixed_count),
            static_cast<unsigned long long>(
                problem.anchor_preservation
                    .previous_fiducial_neighborhood_fixed_count),
            static_cast<unsigned long long>(
                problem.anchor_preservation.branch_anchor_count),
            static_cast<unsigned long long>(
                problem.anchor_preservation.independent_branches));

        for (const auto& vertex : problem.vertices)
        {
            RCLCPP_WARN(
                get_logger(),
                "[F1I-GRAPH-VERTEX-SELECT] task_id=%llu drone_id=%u epoch=%llu kf=%llu variable=%s fixed=%s hard_fiducial=%s anchor_neighborhood=%s support=%llu weight=%.3f reason=%s world_t=(%.3f,%.3f,%.3f) yaw=%.3f",
                static_cast<unsigned long long>(problem.task_id),
                vertex.keyframe_id.drone_id,
                static_cast<unsigned long long>(vertex.keyframe_id.map_epoch),
                static_cast<unsigned long long>(vertex.keyframe_id.local_kf_id),
                vertex.is_variable ? "true" : "false",
                vertex.is_fixed ? "true" : "false",
                vertex.is_hard_fiducial ? "true" : "false",
                vertex.is_anchor_neighborhood ? "true" : "false",
                static_cast<unsigned long long>(vertex.support_count),
                vertex.weight,
                vertex.selection_reason.c_str(),
                vertex.initial_world_T_kf(0, 3),
                vertex.initial_world_T_kf(1, 3),
                vertex.initial_world_T_kf(2, 3),
                YawFromTransform(vertex.initial_world_T_kf));
            RCLCPP_WARN(
                get_logger(),
                "[F1I-GRAPH-%s-KF] task_id=%llu drone_id=%u epoch=%llu kf=%llu reason=%s",
                vertex.is_fixed ? "FIXED" : "VARIABLE",
                static_cast<unsigned long long>(problem.task_id),
                vertex.keyframe_id.drone_id,
                static_cast<unsigned long long>(vertex.keyframe_id.map_epoch),
                static_cast<unsigned long long>(vertex.keyframe_id.local_kf_id),
                vertex.selection_reason.c_str());
            if (vertex.selection_reason == "previous_fiducial_anchor")
            {
                RCLCPP_WARN(
                    get_logger(),
                    "[F1I-GRAPH-PREVIOUS-FIDUCIAL-ANCHOR] task_id=%llu drone_id=%u epoch=%llu kf=%llu fixed=%s hard_fiducial=%s",
                    static_cast<unsigned long long>(problem.task_id),
                    vertex.keyframe_id.drone_id,
                    static_cast<unsigned long long>(vertex.keyframe_id.map_epoch),
                    static_cast<unsigned long long>(vertex.keyframe_id.local_kf_id),
                    vertex.is_fixed ? "true" : "false",
                    vertex.is_hard_fiducial ? "true" : "false");
            }
            if (vertex.selection_reason == "corner_yaw_vertex")
            {
                RCLCPP_WARN(
                    get_logger(),
                    "[F1I-GRAPH-CORNER-VERTEX] task_id=%llu drone_id=%u epoch=%llu kf=%llu threshold_rad=%.3f",
                    static_cast<unsigned long long>(problem.task_id),
                    vertex.keyframe_id.drone_id,
                    static_cast<unsigned long long>(vertex.keyframe_id.map_epoch),
                    static_cast<unsigned long long>(vertex.keyframe_id.local_kf_id),
                    config.corner_yaw_threshold_rad);
            }
            if (vertex.selection_reason == "long_edge_split_vertex")
            {
                RCLCPP_WARN(
                    get_logger(),
                    "[F1I-GRAPH-LONG-EDGE-SPLIT] task_id=%llu drone_id=%u epoch=%llu kf=%llu max_gap=%llu max_length_m=%.3f",
                    static_cast<unsigned long long>(problem.task_id),
                    vertex.keyframe_id.drone_id,
                    static_cast<unsigned long long>(vertex.keyframe_id.map_epoch),
                    static_cast<unsigned long long>(vertex.keyframe_id.local_kf_id),
                    static_cast<unsigned long long>(config.max_temporal_edge_kf_gap),
                    config.max_temporal_edge_length_m);
            }
        }

        RCLCPP_WARN(
            get_logger(),
            "[F1I-GRAPH-EDGES] task_id=%llu count=%llu temporal_edges=%llu source=F1I_TEMPORAL_WINDOW",
            static_cast<unsigned long long>(problem.task_id),
            static_cast<unsigned long long>(problem.edges.size()),
            static_cast<unsigned long long>(problem.edges.size()));
        for (const auto& edge : problem.edges)
        {
            RCLCPP_WARN(
                get_logger(),
                "[F1L-GRAPH-EDGE-SUPPORT] task_id=%llu edge_id=%llu from_kf=%llu to_kf=%llu intermediate_kfs=%llu support_kfs=%llu support_length_m=%.6f support_density_kfs_per_m=%.6f support_rigidity_multiplier=%.6f weight=%.6f",
                static_cast<unsigned long long>(problem.task_id),
                static_cast<unsigned long long>(edge.edge_id),
                static_cast<unsigned long long>(edge.from_keyframe_id.local_kf_id),
                static_cast<unsigned long long>(edge.to_keyframe_id.local_kf_id),
                static_cast<unsigned long long>(edge.intermediate_keyframe_count),
                static_cast<unsigned long long>(edge.support_keyframe_count),
                edge.support_length_m,
                edge.support_density_kfs_per_m,
                edge.support_rigidity_multiplier,
                edge.weight);
        }

        uint64_t hard_priors = 0;
        uint64_t target_priors = 0;
        uint64_t soft_priors = 0;
        for (const auto& prior : problem.priors)
        {
            if (prior.prior_type == orbslam3_multi::PoseGraphPriorType::FiducialHard)
            {
                ++hard_priors;
            }
            else if (prior.prior_type == orbslam3_multi::PoseGraphPriorType::FiducialTarget)
            {
                ++target_priors;
            }
            else
            {
                ++soft_priors;
            }
        }
        RCLCPP_WARN(
            get_logger(),
            "[F1I-GRAPH-PRIORS] task_id=%llu count=%llu hard_fiducial=%llu fiducial_target=%llu current_soft=%llu",
            static_cast<unsigned long long>(problem.task_id),
            static_cast<unsigned long long>(problem.priors.size()),
            static_cast<unsigned long long>(hard_priors),
            static_cast<unsigned long long>(target_priors),
            static_cast<unsigned long long>(soft_priors));

        RCLCPP_WARN(
            get_logger(),
            "[F1I-GRAPH-WEIGHTS] task_id=%llu temporal=%.3f temporal_sparse=%.3f fiducial_hard=%.3f fiducial_target_t=%.3f fiducial_target_rot=%.3f current_pose_soft=%.3f",
            static_cast<unsigned long long>(problem.task_id),
            config.temporal_edge_weight,
            config.temporal_edge_weight_sparse,
            config.fiducial_hard_weight,
            config.fiducial_target_translation_weight,
            config.fiducial_target_rotation_weight,
            config.current_pose_soft_weight);

        RCLCPP_WARN(
            get_logger(),
            "[F1I-GRAPH-PROPAGATION-PLAN] task_id=%llu affected_non_variable=%llu entries=%llu mode_policy=path_segment_or_nearest",
            static_cast<unsigned long long>(problem.task_id),
            static_cast<unsigned long long>(problem.affected_non_variable_keyframes.size()),
            static_cast<unsigned long long>(problem.propagation_plan.size()));

        for (const auto& entry : problem.propagation_plan)
        {
            const std::string control_b_text = entry.control_vertex_b
                ? std::to_string(entry.control_vertex_b->local_kf_id)
                : "none";
            RCLCPP_WARN(
                get_logger(),
                "[F1I-GRAPH-PROPAGATION-SEGMENT] task_id=%llu affected_kf=%llu control_a=%llu control_b=%s alpha=%.6f distance_from_a_m=%.3f segment_length_m=%.3f control_span_kf_gap=%llu mode=%s",
                static_cast<unsigned long long>(problem.task_id),
                static_cast<unsigned long long>(entry.affected_keyframe_id.local_kf_id),
                static_cast<unsigned long long>(entry.control_vertex_a.local_kf_id),
                control_b_text.c_str(),
                entry.segment_alpha,
                entry.distance_from_a_m,
                entry.segment_length_m,
                static_cast<unsigned long long>(entry.control_span_kf_gap),
                orbslam3_multi::ToString(entry.mode));
        }

        RCLCPP_WARN(
            get_logger(),
            "[F1I-GRAPH-PROBLEM-CREATED] task_id=%llu type=%s source=%s vertices=%llu edges=%llu priors=%llu variables=%llu fixed=%llu affected_non_variable=%llu propagation=%llu",
            static_cast<unsigned long long>(problem.task_id),
            problem.task_type.c_str(),
            problem.source.c_str(),
            static_cast<unsigned long long>(problem.summary.vertices),
            static_cast<unsigned long long>(problem.summary.edges),
            static_cast<unsigned long long>(problem.summary.priors),
            static_cast<unsigned long long>(problem.summary.variable_vertices),
            static_cast<unsigned long long>(problem.summary.fixed_vertices),
            static_cast<unsigned long long>(
                problem.summary.affected_non_variable_keyframes),
            static_cast<unsigned long long>(problem.summary.propagation_entries));

        RCLCPP_WARN(
            get_logger(),
            "[F1I-GRAPH-BUILD-SUMMARY] task_id=%llu success=true reason=%s vertices=%llu variable=%llu fixed=%llu hard_fiducial=%llu edges=%llu priors=%llu affected_non_variable=%llu propagation=%llu",
            static_cast<unsigned long long>(problem.task_id),
            reason.c_str(),
            static_cast<unsigned long long>(problem.summary.vertices),
            static_cast<unsigned long long>(problem.summary.variable_vertices),
            static_cast<unsigned long long>(problem.summary.fixed_vertices),
            static_cast<unsigned long long>(problem.summary.hard_fiducial_vertices),
            static_cast<unsigned long long>(problem.summary.edges),
            static_cast<unsigned long long>(problem.summary.priors),
            static_cast<unsigned long long>(
                problem.summary.affected_non_variable_keyframes),
            static_cast<unsigned long long>(problem.summary.propagation_entries));
    }

    void PopulatePoseGraphDebugKeyFrames(PoseGraphProblem& problem) const
    {
        std::set<RawKeyFrameId> keyframes;
        for (const auto& vertex : problem.vertices)
        {
            keyframes.insert(vertex.keyframe_id);
        }
        for (const auto& keyframe_id : problem.affected_non_variable_keyframes)
        {
            keyframes.insert(keyframe_id);
        }

        problem.debug_keyframes.clear();
        problem.debug_keyframes.reserve(keyframes.size());
        for (const auto& keyframe_id : keyframes)
        {
            const auto map_pose = pose_store_.GetWorldPose(keyframe_id);
            if (!map_pose.has_value())
            {
                continue;
            }

            orbslam3_multi::PoseGraphDebugKeyFramePose debug;
            debug.keyframe_id = keyframe_id;
            debug.map_world_T_kf = map_pose.value();
            const auto gt_it = f1l_gt_keyframe_store_.find(keyframe_id);
            if (gt_it != f1l_gt_keyframe_store_.end())
            {
                debug.has_gt = true;
                debug.gt_world_T_kf = gt_it->second.world_T_kf_gt;
                debug.association_dt_sec = gt_it->second.association_dt_sec;
                debug.association_quality = gt_it->second.association_quality;
            }
            else
            {
                debug.association_quality = "MISSING";
            }
            problem.debug_keyframes.push_back(debug);
        }
    }

    void DumpPoseGraphProblemForOffline(const PoseGraphProblem& problem) const
    {
        if (!f1l_graph_dump_enabled_)
        {
            RCLCPP_WARN(
                get_logger(),
                "[F1L-GRAPH-DUMP] task_id=%llu enabled=false reason=disabled",
                static_cast<unsigned long long>(problem.task_id));
            return;
        }

        PoseGraphProblem dump_problem = problem;
        PopulatePoseGraphDebugKeyFrames(dump_problem);
        uint64_t gt_debug_count = 0;
        for (const auto& debug : dump_problem.debug_keyframes)
        {
            if (debug.has_gt)
            {
                ++gt_debug_count;
            }
        }

        const std::string slash =
            (!f1l_graph_dump_output_dir_.empty() &&
             f1l_graph_dump_output_dir_.back() == '/') ? "" : "/";
        const std::string path =
            f1l_graph_dump_output_dir_ + slash +
            "f1l_graph_task_" + std::to_string(problem.task_id) + ".tsv";
        std::error_code mkdir_error;
        std::filesystem::create_directories(
            f1l_graph_dump_output_dir_,
            mkdir_error);
        if (mkdir_error)
        {
            RCLCPP_WARN(
                get_logger(),
                "[F1L-GRAPH-DUMP] task_id=%llu enabled=true success=false path=%s reason=mkdir_failed_%s vertices=%llu edges=%llu priors=%llu propagation=%llu debug_kfs=%llu gt_debug_kfs=%llu",
                static_cast<unsigned long long>(problem.task_id),
                path.c_str(),
                mkdir_error.message().c_str(),
                static_cast<unsigned long long>(dump_problem.vertices.size()),
                static_cast<unsigned long long>(dump_problem.edges.size()),
                static_cast<unsigned long long>(dump_problem.priors.size()),
                static_cast<unsigned long long>(dump_problem.propagation_plan.size()),
                static_cast<unsigned long long>(dump_problem.debug_keyframes.size()),
                static_cast<unsigned long long>(gt_debug_count));
            return;
        }
        const auto save = orbslam3_multi::SavePoseGraphProblem(dump_problem, path);
        RCLCPP_WARN(
            get_logger(),
            "[F1L-GRAPH-DUMP] task_id=%llu enabled=true success=%s path=%s reason=%s vertices=%llu edges=%llu priors=%llu propagation=%llu debug_kfs=%llu gt_debug_kfs=%llu",
            static_cast<unsigned long long>(problem.task_id),
            save.success ? "true" : "false",
            path.c_str(),
            save.reason.c_str(),
            static_cast<unsigned long long>(dump_problem.vertices.size()),
            static_cast<unsigned long long>(dump_problem.edges.size()),
            static_cast<unsigned long long>(dump_problem.priors.size()),
            static_cast<unsigned long long>(dump_problem.propagation_plan.size()),
            static_cast<unsigned long long>(dump_problem.debug_keyframes.size()),
            static_cast<unsigned long long>(gt_debug_count));
    }

    std::optional<FiducialOptimizationTask> CreateF1IDebugTaskFromCurrentState()
    {
        // F1I: la prueba replay puede usar un dataset cuyo residual fiducial
        // queda bajo umbral. Esta tarea sintetica valida solo el builder sobre
        // KFs reales/replay ya anclados; no se inserta en FiducialAnchorManager.
        const auto submap_ids = raw_db_.GetSubmapIds();
        for (const auto& submap_id : submap_ids)
        {
            if (!pose_store_.HasSubmapAnchor(submap_id))
            {
                continue;
            }

            RawKeyFrameId selected;
            bool found = false;
            const auto keyframe_ids = raw_db_.GetKeyFrameIdsForSubmap(submap_id);
            for (auto it = keyframe_ids.rbegin(); it != keyframe_ids.rend(); ++it)
            {
                if (!pose_store_.GetWorldPose(*it))
                {
                    continue;
                }
                if (!found || !pose_store_.IsHardFiducialKeyFrame(*it))
                {
                    selected = *it;
                    found = true;
                    if (!pose_store_.IsHardFiducialKeyFrame(*it))
                    {
                        break;
                    }
                }
            }
            if (!found)
            {
                continue;
            }

            const auto current_pose = pose_store_.GetWorldPose(selected);
            if (!current_pose)
            {
                continue;
            }

            FiducialOptimizationTask task;
            task.task_id = 9000000000ULL + (++f1i_debug_task_counter_);
            task.task_type = "FIDUCIAL_REVISIT_ERROR_DEBUG";
            task.keyframe_id = selected;
            task.submap_id = submap_id;
            task.drone_id = selected.drone_id;
            task.map_epoch = selected.map_epoch;
            task.fiducial_id = -1;
            task.estimated_world_T_kf = current_pose.value();
            task.target_world_T_kf =
                PlanarTransform(
                    f1i_debug_task_dx_,
                    f1i_debug_task_dy_,
                    f1i_debug_task_dz_,
                    f1i_debug_task_dyaw_) * current_pose.value();
            task.error_t_m =
                std::sqrt(f1i_debug_task_dx_ * f1i_debug_task_dx_ +
                          f1i_debug_task_dy_ * f1i_debug_task_dy_ +
                          f1i_debug_task_dz_ * f1i_debug_task_dz_);
            task.error_yaw_rad = std::abs(f1i_debug_task_dyaw_);
            task.error_rot_rad = std::abs(f1i_debug_task_dyaw_);
            task.threshold_t_m = fiducial_revisit_error_threshold_m_;
            task.threshold_rot_rad = fiducial_revisit_rot_threshold_rad_;
            task.threshold_yaw_rad = fiducial_revisit_yaw_threshold_rad_;
            task.arrival_id = raw_db_.GetDatabaseStats().last_arrival_id;
            task.source = "F1I_DEBUG_FORCE_BUILD";

            RCLCPP_WARN(
                get_logger(),
                "[F1I-DEBUG-TASK-CREATED] task_id=%llu drone_id=%u epoch=%llu kf=%llu dx=%.3f dy=%.3f dz=%.3f dyaw=%.3f",
                static_cast<unsigned long long>(task.task_id),
                task.drone_id,
                static_cast<unsigned long long>(task.map_epoch),
                static_cast<unsigned long long>(task.keyframe_id.local_kf_id),
                f1i_debug_task_dx_,
                f1i_debug_task_dy_,
                f1i_debug_task_dz_,
                f1i_debug_task_dyaw_);
            return task;
        }

        return std::nullopt;
    }

    void TryF1IDebugBuildAfterReplay()
    {
        if (!f1i_debug_force_task_enabled_ || f1i_debug_task_done_)
        {
            return;
        }

        const auto task = CreateF1IDebugTaskFromCurrentState();
        if (!task)
        {
            RCLCPP_WARN(
                get_logger(),
                "[F1I-GRAPH-DEBUG-BUILD-ONLY] status=waiting reason=no_anchored_keyframe");
            return;
        }

        RCLCPP_WARN(
            get_logger(),
            "[F1I-GRAPH-DEBUG-BUILD-ONLY] status=building task_id=%llu",
            static_cast<unsigned long long>(task->task_id));
        const auto result =
            BuildAndLogPoseGraphForTask(task.value(), "replay_done_debug", true);
        f1i_debug_task_done_ = result.success;
    }

    // F1B: calcula una huella compacta de IDs locales para KFs o MPs.
    // Entrada: contenedor de mensajes con campo `id`.
    // Salida: `false` si no hay elementos; si hay, rango [min_id, max_id].
    template <typename Container>
    std::pair<bool, std::pair<uint64_t, uint64_t>> ComputeIdRange(const Container& items) const
    {
        if (items.empty())
        {
            return {false, {0, 0}};
        }

        uint64_t min_id = items.front().id;
        uint64_t max_id = items.front().id;

        // F1B: los deltas no tienen por que llegar ordenados; el rango sirve
        // como huella rapida sin asumir orden ni continuidad de IDs locales.
        for (const auto& item : items)
        {
            min_id = std::min<uint64_t>(min_id, item.id);
            max_id = std::max<uint64_t>(max_id, item.id);
        }

        return {true, {min_id, max_id}};
    }

    // F1B: formatea acumulados por dron bajo `stats_mutex_`.
    // Precondicion: el llamador ya mantiene el lock para evitar mezclar datos
    // mientras llega un `OrbMap` nuevo.
    std::string FormatPerDroneStatsLocked() const
    {
        std::ostringstream oss;
        bool first = true;

        // F1B: el `std::map` mantiene salida determinista por `drone_id`, util
        // para comparar logs reducidos entre ejecuciones.
        for (const auto& [drone_id, stats] : per_drone_stats_)
        {
            if (!first)
            {
                oss << ",";
            }
            first = false;

            oss << drone_id << ":maps=" << stats.maps
                << ":kfs=" << stats.keyframes
                << ":mps=" << stats.mappoints;

            if (stats.has_last_message)
            {
                oss << ":last_epoch=" << stats.last_epoch
                    << ":last_seq=" << stats.last_sequence;
            }
        }

        return oss.str();
    }

    // F1B: callback ROS para `/dron_X/orbslam/orb_map_delta`.
    // Entrada: mensaje `OrbMap` publicado por cada wrapper y el dron inferido
    // desde el topic suscrito.
    // Efecto: actualiza contadores de observabilidad, emite logs y, desde F1C,
    // persiste el delta raw en RawMapDatabase con `arrival_id`.
    void OnOrbMapDelta(const OrbMap::SharedPtr msg, uint32_t subscribed_drone_id)
    {
        // F1B: un mensaje nulo no deberia ocurrir, pero dejarlo como error
        // explicito ayuda a distinguir fallo ROS de ausencia real de deltas.
        if (!msg)
        {
            RCLCPP_ERROR(
                get_logger(),
                "[F1B-ORBMAP-RX] subscribed_drone_id=%u error=null_message",
                subscribed_drone_id);
            return;
        }

        const uint32_t drone_id = msg->drone_id;
        const uint64_t map_epoch = msg->map_epoch;
        const uint64_t map_sequence = msg->map_sequence;
        const size_t keyframe_count = msg->keyframes.size();
        const size_t mappoint_count = msg->mappoints.size();
        const uint64_t arrival_id = rawdb_next_arrival_id_++;
        const auto insert_result = raw_db_.InsertDelta(arrival_id, *msg);

        {
            // F1B: todos los acumulados se actualizan juntos para que
            // `PublishStatsLog` nunca vea contadores parciales de un mismo
            // delta.
            std::lock_guard<std::mutex> lock(stats_mutex_);

            ++total_maps_;
            total_keyframes_ += keyframe_count;
            total_mappoints_ += mappoint_count;
            drones_seen_.insert(drone_id);
            epochs_seen_.insert({drone_id, map_epoch});

            auto& stats = per_drone_stats_[drone_id];
            ++stats.maps;
            stats.keyframes += keyframe_count;
            stats.mappoints += mappoint_count;
            stats.last_epoch = map_epoch;
            stats.last_sequence = map_sequence;
            stats.has_last_message = true;
        }

        RCLCPP_WARN(
            get_logger(),
            "[F1C-RAWDB-DELTA-RX] arrival_id=%llu drone_id=%u epoch=%llu seq=%llu kfs=%zu mps=%zu",
            static_cast<unsigned long long>(arrival_id),
            drone_id,
            static_cast<unsigned long long>(map_epoch),
            static_cast<unsigned long long>(map_sequence),
            keyframe_count,
            mappoint_count);

        if (insert_result.new_submap)
        {
            RCLCPP_WARN(
                get_logger(),
                "[F1C-RAWDB-NEW-SUBMAP] drone_id=%u epoch=%llu arrival_id=%llu",
                drone_id,
                static_cast<unsigned long long>(map_epoch),
                static_cast<unsigned long long>(arrival_id));
        }

        LogRawInsert("F1C-RAWDB-INSERT-DELTA", arrival_id, insert_result.stats);
        AssociateGtDebugForDelta(*msg, arrival_id);
        UpdateScoresFromMap(*msg, arrival_id);
        ProcessFiducialsForDelta(*msg, arrival_id);
        RegisterF1EKeyFramesFromMap(*msg);
        HandlePoseStoreDebugAfterInsert(*msg, insert_result.stats.journal_entries);
        PublishGlobalSparseCloud("delta");

        RCLCPP_WARN(
            get_logger(),
            "[F1B-ORBMAP-RX] subscribed_drone_id=%u drone_id=%u drone_name=%s epoch=%llu seq=%llu frame_id=%s map_frame=%s kfs=%zu mps=%zu",
            subscribed_drone_id,
            drone_id,
            msg->drone_name.c_str(),
            static_cast<unsigned long long>(map_epoch),
            static_cast<unsigned long long>(map_sequence),
            msg->header.frame_id.c_str(),
            msg->map_frame.c_str(),
            keyframe_count,
            mappoint_count);

        if (drone_id != subscribed_drone_id)
        {
            // F1B: esta advertencia detecta incoherencias entre namespace ROS y
            // contenido del mensaje sin descartar el delta; en migraciones
            // posteriores puede convertirse en validacion mas estricta.
            RCLCPP_WARN(
                get_logger(),
                "[F1B-ORBMAP-RX] subscribed_drone_id=%u drone_id=%u warning=drone_id_topic_mismatch",
                subscribed_drone_id,
                drone_id);
        }

        LogKeyFrameRange(*msg);
        LogMapPointRange(*msg);
    }

    // F1B: loggea conteo y rango de IDs de KeyFrames recibidos en un delta.
    // Sirve para validar que el wrapper manda contenido sin inspeccionar todo
    // el mensaje en logs gigantes.
    void LogKeyFrameRange(const OrbMap& msg)
    {
        const auto range = ComputeIdRange(msg.keyframes);
        if (!range.first)
        {
            RCLCPP_WARN(
                get_logger(),
                "[F1B-ORBMAP-RX-KFS] drone_id=%u epoch=%llu seq=%llu count=0 first_kf=none last_kf=none",
                msg.drone_id,
                static_cast<unsigned long long>(msg.map_epoch),
                static_cast<unsigned long long>(msg.map_sequence));
            return;
        }

        RCLCPP_WARN(
            get_logger(),
            "[F1B-ORBMAP-RX-KFS] drone_id=%u epoch=%llu seq=%llu count=%zu first_kf=%llu last_kf=%llu",
            msg.drone_id,
            static_cast<unsigned long long>(msg.map_epoch),
            static_cast<unsigned long long>(msg.map_sequence),
            msg.keyframes.size(),
            static_cast<unsigned long long>(range.second.first),
            static_cast<unsigned long long>(range.second.second));
    }

    // F1B: loggea conteo y rango de IDs de MapPoints recibidos en un delta.
    // Mantiene la misma estructura que KFs para que los patrones de reduccion
    // puedan comparar ambos tipos de datos.
    void LogMapPointRange(const OrbMap& msg)
    {
        const auto range = ComputeIdRange(msg.mappoints);
        if (!range.first)
        {
            RCLCPP_WARN(
                get_logger(),
                "[F1B-ORBMAP-RX-MPS] drone_id=%u epoch=%llu seq=%llu count=0 first_mp=none last_mp=none",
                msg.drone_id,
                static_cast<unsigned long long>(msg.map_epoch),
                static_cast<unsigned long long>(msg.map_sequence));
            return;
        }

        RCLCPP_WARN(
            get_logger(),
            "[F1B-ORBMAP-RX-MPS] drone_id=%u epoch=%llu seq=%llu count=%zu first_mp=%llu last_mp=%llu",
            msg.drone_id,
            static_cast<unsigned long long>(msg.map_epoch),
            static_cast<unsigned long long>(msg.map_sequence),
            msg.mappoints.size(),
            static_cast<unsigned long long>(range.second.first),
            static_cast<unsigned long long>(range.second.second));
    }

    // F1B: emite una foto periodica de recepcion acumulada.
    // Efecto: no publica topics ROS; solo deja evidencia textual para decidir
    // si 1B recibe mapas de todos los drones esperados.
    void PublishStatsLog()
    {
        std::lock_guard<std::mutex> lock(stats_mutex_);

        const std::string per_drone = FormatPerDroneStatsLocked();
        const auto raw_stats = raw_db_.GetDatabaseStats();

        RCLCPP_WARN(
            get_logger(),
            "[F1B-SERVER-STATS] rx_maps=%llu rx_kfs=%llu rx_mps=%llu drones_seen=%zu epochs_seen=%zu expected_drones=%d per_drone=\"%s\"",
            static_cast<unsigned long long>(total_maps_),
            static_cast<unsigned long long>(total_keyframes_),
            static_cast<unsigned long long>(total_mappoints_),
            drones_seen_.size(),
            epochs_seen_.size(),
            n_drones_,
            per_drone.c_str());

        RCLCPP_WARN(
            get_logger(),
            "[F1C-RAWDB-STATS] journal=%llu deltas=%llu full=%llu fiducial_observations=%llu submaps=%llu kfs=%llu mps=%llu last_arrival_id=%llu",
            static_cast<unsigned long long>(raw_stats.journal_entries),
            static_cast<unsigned long long>(raw_stats.delta_entries),
            static_cast<unsigned long long>(raw_stats.full_snapshot_entries),
            static_cast<unsigned long long>(raw_stats.fiducial_observations),
            static_cast<unsigned long long>(raw_stats.submaps),
            static_cast<unsigned long long>(raw_stats.keyframes),
            static_cast<unsigned long long>(raw_stats.mappoints),
            static_cast<unsigned long long>(raw_stats.last_arrival_id));
    }

    void LogRawInsert(const char* marker, uint64_t arrival_id, const RawDatabaseStats& stats)
    {
        RCLCPP_WARN(
            get_logger(),
            "[%s] arrival_id=%llu journal=%llu deltas=%llu full=%llu fiducial_observations=%llu submaps=%llu kfs=%llu mps=%llu",
            marker,
            static_cast<unsigned long long>(arrival_id),
            static_cast<unsigned long long>(stats.journal_entries),
            static_cast<unsigned long long>(stats.delta_entries),
            static_cast<unsigned long long>(stats.full_snapshot_entries),
            static_cast<unsigned long long>(stats.fiducial_observations),
            static_cast<unsigned long long>(stats.submaps),
            static_cast<unsigned long long>(stats.keyframes),
            static_cast<unsigned long long>(stats.mappoints));
    }

    void SaveRawDatabase(const std::string& reason)
    {
        if (!rawdb_record_enabled_ || rawdb_record_path_.empty())
        {
            return;
        }

        std::string error;
        const auto stats = raw_db_.GetDatabaseStats();
        const bool ok = raw_db_.SaveToPath(rawdb_record_path_, &error);
        if (ok)
        {
            RCLCPP_WARN(
                get_logger(),
                "[F1C-RAWDB-SAVE] reason=%s path=%s journal=%llu deltas=%llu full=%llu fiducial_observations=%llu submaps=%llu kfs=%llu mps=%llu",
                reason.c_str(),
                rawdb_record_path_.c_str(),
                static_cast<unsigned long long>(stats.journal_entries),
                static_cast<unsigned long long>(stats.delta_entries),
                static_cast<unsigned long long>(stats.full_snapshot_entries),
                static_cast<unsigned long long>(stats.fiducial_observations),
                static_cast<unsigned long long>(stats.submaps),
                static_cast<unsigned long long>(stats.keyframes),
                static_cast<unsigned long long>(stats.mappoints));
        }
        else
        {
            RCLCPP_ERROR(
                get_logger(),
                "[F1C-RAWDB-SAVE] reason=%s path=%s error=%s",
                reason.c_str(),
                rawdb_record_path_.c_str(),
                error.c_str());
        }
    }

    void LoadReplayDataset()
    {
        std::string error;
        RawMapDatabase loaded_db;
        if (!loaded_db.LoadFromPath(rawdb_replay_path_, &error))
        {
            RCLCPP_ERROR(
                get_logger(),
                "[F1C-REPLAY-LOAD] path=%s success=false error=%s",
                rawdb_replay_path_.c_str(),
                error.c_str());
            return;
        }

        replay_entries_ = loaded_db.GetJournalCopy();
        replay_fiducials_by_arrival_.clear();
        const auto fiducial_journal = loaded_db.GetFiducialObservationJournalCopy();
        for (const auto& observation : fiducial_journal)
        {
            replay_fiducials_by_arrival_[observation.arrival_id].push_back(observation);
        }
        const auto loaded_stats = loaded_db.GetDatabaseStats();
        raw_db_.Clear();
        replay_index_ = 0;
        replay_done_logged_ = false;

        RCLCPP_WARN(
            get_logger(),
            "[F1C-REPLAY-LOAD] path=%s success=true entries=%zu deltas=%llu full=%llu submaps=%llu kfs=%llu mps=%llu fiducial_observations=%llu",
            rawdb_replay_path_.c_str(),
            replay_entries_.size(),
            static_cast<unsigned long long>(loaded_stats.delta_entries),
            static_cast<unsigned long long>(loaded_stats.full_snapshot_entries),
            static_cast<unsigned long long>(loaded_stats.submaps),
            static_cast<unsigned long long>(loaded_stats.keyframes),
            static_cast<unsigned long long>(loaded_stats.mappoints),
            static_cast<unsigned long long>(loaded_stats.fiducial_observations));

        replay_timer_ = create_wall_timer(
            std::chrono::duration<double>(rawdb_replay_period_sec_),
            std::bind(&GlobalMapServer::ReplayNextEntry, this));
    }

    void ReplayNextEntry()
    {
        if (replay_index_ >= replay_entries_.size())
        {
            if (!replay_done_logged_)
            {
                replay_done_logged_ = true;
                const auto stats = raw_db_.GetDatabaseStats();
                const uint64_t replay_fiducials = ReplayFiducialObservationCount();
                RCLCPP_WARN(
                    get_logger(),
                    "[F1C-REPLAY-DONE] entries=%zu journal=%llu deltas=%llu full=%llu fiducial_observations=%llu submaps=%llu kfs=%llu mps=%llu",
                    replay_entries_.size(),
                    static_cast<unsigned long long>(stats.journal_entries),
                    static_cast<unsigned long long>(stats.delta_entries),
                    static_cast<unsigned long long>(stats.full_snapshot_entries),
                    static_cast<unsigned long long>(replay_fiducials),
                    static_cast<unsigned long long>(stats.submaps),
                    static_cast<unsigned long long>(stats.keyframes),
                    static_cast<unsigned long long>(stats.mappoints));
                TryF1IDebugBuildAfterReplay();
            }
            return;
        }

        const RawJournalEntry& entry = replay_entries_[replay_index_++];
        orbslam3_multi::RawInsertResult result;
        if (entry.kind == RawJournalEntryKind::FullSnapshot)
        {
            result = raw_db_.InsertFullSnapshot(entry.arrival_id, entry.map);
            RCLCPP_WARN(
                get_logger(),
                "[F1G-REPLAY-FULL-SNAPSHOT] index=%zu arrival_id=%llu drone_id=%u epoch=%llu seq=%llu kfs=%zu mps=%zu",
                replay_index_,
                static_cast<unsigned long long>(entry.arrival_id),
                entry.map.drone_id,
                static_cast<unsigned long long>(entry.map.map_epoch),
                static_cast<unsigned long long>(entry.map.map_sequence),
                entry.map.keyframes.size(),
                entry.map.mappoints.size());
        }
        else
        {
            result = raw_db_.InsertDelta(entry.arrival_id, entry.map);
        }

        RCLCPP_WARN(
            get_logger(),
            "[F1C-REPLAY-DELTA] index=%zu arrival_id=%llu kind=%s drone_id=%u epoch=%llu seq=%llu kfs=%zu mps=%zu",
            replay_index_,
            static_cast<unsigned long long>(entry.arrival_id),
            orbslam3_multi::ToString(entry.kind),
            entry.map.drone_id,
            static_cast<unsigned long long>(entry.map.map_epoch),
            static_cast<unsigned long long>(entry.map.map_sequence),
            entry.map.keyframes.size(),
            entry.map.mappoints.size());

        if (entry.kind == RawJournalEntryKind::FullSnapshot)
        {
            MaybeSetF1GDebugOptimizedKeyFrame(entry.map, "replay_full_snapshot");
            ProcessFullSnapshotAfterInsert(
                entry.map,
                entry.arrival_id,
                result,
                "replay_full_snapshot");
        }
        else
        {
            LogRawInsert("F1C-RAWDB-INSERT-DELTA", entry.arrival_id, result.stats);
            UpdateScoresFromMap(entry.map, entry.arrival_id);
            ProcessReplayFiducials(entry.arrival_id);
            RegisterF1EKeyFramesFromMap(entry.map);
            HandlePoseStoreDebugAfterInsert(entry.map, result.stats.journal_entries);
            PublishGlobalSparseCloud("replay_delta");
        }
    }

    uint64_t ReplayFiducialObservationCount() const
    {
        uint64_t count = 0;
        for (const auto& [_, observations] : replay_fiducials_by_arrival_)
        {
            count += observations.size();
        }
        return count;
    }

    void ProcessReplayFiducials(uint64_t arrival_id)
    {
        const auto it = replay_fiducials_by_arrival_.find(arrival_id);
        if (it == replay_fiducials_by_arrival_.end())
        {
            return;
        }

        for (const auto& recorded : it->second)
        {
            const auto observation =
                FromRecordedObservation(recorded, "REPLAY_RECORDED_FIDUCIAL");
            RCLCPP_WARN(
                get_logger(),
                "[F1E-FID-REPLAY-OBS] arrival_id=%llu drone_id=%u epoch=%llu kf=%llu fid=%d source=REPLAY_RECORDED_FIDUCIAL",
                static_cast<unsigned long long>(observation.arrival_id),
                observation.drone_id,
                static_cast<unsigned long long>(observation.map_epoch),
                static_cast<unsigned long long>(observation.local_keyframe_id),
                observation.fiducial_id);
            HandleFiducialObservation(observation, false);
        }
    }

    void RegisterF1EKeyFramesFromMap(const OrbMap& map)
    {
        // F1E: despues de que un submapa quede anclado por fiducial, los KFs
        // nuevos del mismo submapa pueden recibir pose world derivada sin tocar
        // RawMapDatabase ni convertirlos en hard fiducial.
        for (const auto& keyframe : map.keyframes)
        {
            const RawKeyFrameId keyframe_id{
                map.drone_id,
                map.map_epoch,
                keyframe.id};
            const auto result =
                pose_store_.RegisterNewKeyFrameIfAnchored(
                    keyframe_id,
                    raw_db_,
                    "F1E_FIDUCIAL_ANCHOR");
            if (result.status == GlobalPoseNewKeyFrameStatus::PoseSet)
            {
                RCLCPP_WARN(
                    get_logger(),
                    "[F1E-POSESTORE-NEW-KF-POSE-SET] drone_id=%u epoch=%llu kf=%llu source=%s",
                    keyframe_id.drone_id,
                    static_cast<unsigned long long>(keyframe_id.map_epoch),
                    static_cast<unsigned long long>(keyframe_id.local_kf_id),
                    result.source.c_str());
            }
            else if (result.status == GlobalPoseNewKeyFrameStatus::CorrectionInherited)
            {
                RCLCPP_WARN(
                    get_logger(),
                    "[F1K-POSESTORE-NEW-KF-INHERIT-CORRECTION] drone_id=%u epoch=%llu kf=%llu inherited_correction=true source=%s reason=%s",
                    keyframe_id.drone_id,
                    static_cast<unsigned long long>(keyframe_id.map_epoch),
                    static_cast<unsigned long long>(keyframe_id.local_kf_id),
                    result.source.c_str(),
                    result.reason.c_str());
            }
        }
    }

    void HandlePoseStoreDebugAfterInsert(const OrbMap& map, uint64_t journal_entries)
    {
        if (!pose_store_debug_enabled_)
        {
            return;
        }

        // F1D: el modo debug valida la infraestructura de pose store con datos
        // raw reales/replay. No representa un anchor real; 1E introducira el
        // anclaje por fiducial simulado.
        TryPoseStoreDebugAnchor(journal_entries);
        TryPoseStoreDebugOptimization(journal_entries);
        RegisterPoseStoreKeyFramesFromMap(map);

        if (journal_entries % 25U == 0U)
        {
            LogPoseStoreStats("periodic_debug");
        }
    }

    void TryPoseStoreDebugAnchor(uint64_t journal_entries)
    {
        if (pose_store_debug_anchor_done_ ||
            journal_entries < static_cast<uint64_t>(pose_store_debug_anchor_after_deltas_))
        {
            return;
        }

        const RawSubmapId submap_id = DebugPoseStoreSubmapId();
        if (!raw_db_.HasSubmap(submap_id))
        {
            RCLCPP_WARN(
                get_logger(),
                "[F1D-SERVER-DEBUG-ANCHOR] status=waiting reason=submap_missing drone_id=%u epoch=%llu journal=%llu",
                submap_id.drone_id,
                static_cast<unsigned long long>(submap_id.map_epoch),
                static_cast<unsigned long long>(journal_entries));
            return;
        }

        const Eigen::Matrix4d world_T_local =
            PlanarTransform(pose_store_debug_anchor_world_x_,
                            pose_store_debug_anchor_world_y_,
                            pose_store_debug_anchor_world_z_,
                            pose_store_debug_anchor_yaw_);

        RCLCPP_WARN(
            get_logger(),
            "[F1D-POSESTORE-ANCHOR-REQUEST] drone_id=%u epoch=%llu source=DEBUG_TEST journal=%llu world_t=(%.3f,%.3f,%.3f) yaw=%.3f",
            submap_id.drone_id,
            static_cast<unsigned long long>(submap_id.map_epoch),
            static_cast<unsigned long long>(journal_entries),
            pose_store_debug_anchor_world_x_,
            pose_store_debug_anchor_world_y_,
            pose_store_debug_anchor_world_z_,
            pose_store_debug_anchor_yaw_);

        const auto result =
            pose_store_.AnchorSubmap(submap_id, world_T_local, raw_db_, "DEBUG_TEST");
        if (!result.success)
        {
            RCLCPP_ERROR(
                get_logger(),
                "[F1D-SERVER-DEBUG-ANCHOR] status=failed drone_id=%u epoch=%llu reason=%s",
                submap_id.drone_id,
                static_cast<unsigned long long>(submap_id.map_epoch),
                result.reason.c_str());
            return;
        }

        pose_store_debug_anchor_done_ = true;
        RCLCPP_WARN(
            get_logger(),
            "[F1D-SERVER-DEBUG-ANCHOR] status=applied drone_id=%u epoch=%llu source=DEBUG_TEST anchored_kfs=%llu",
            submap_id.drone_id,
            static_cast<unsigned long long>(submap_id.map_epoch),
            static_cast<unsigned long long>(result.anchored_keyframes));
        RCLCPP_WARN(
            get_logger(),
            "[F1D-POSESTORE-ANCHOR-SET] drone_id=%u epoch=%llu source=DEBUG_TEST replaced=%s",
            submap_id.drone_id,
            static_cast<unsigned long long>(submap_id.map_epoch),
            result.replaced_existing_anchor ? "true" : "false");

        const auto keyframe_ids = raw_db_.GetKeyFrameIdsForSubmap(submap_id);
        for (const auto& keyframe_id : keyframe_ids)
        {
            if (!pose_store_.HasWorldPose(keyframe_id))
            {
                continue;
            }

            RCLCPP_WARN(
                get_logger(),
                "[F1D-POSESTORE-ANCHOR-KF-POSE] drone_id=%u epoch=%llu kf=%llu source=DEBUG_TEST",
                keyframe_id.drone_id,
                static_cast<unsigned long long>(keyframe_id.map_epoch),
                static_cast<unsigned long long>(keyframe_id.local_kf_id));
        }

        RCLCPP_WARN(
            get_logger(),
            "[F1D-POSESTORE-ANCHOR-SUMMARY] drone_id=%u epoch=%llu source=DEBUG_TEST anchored_kfs=%llu world_t=(%.3f,%.3f,%.3f) yaw=%.3f",
            submap_id.drone_id,
            static_cast<unsigned long long>(submap_id.map_epoch),
            static_cast<unsigned long long>(result.anchored_keyframes),
            world_T_local(0, 3),
            world_T_local(1, 3),
            world_T_local(2, 3),
            YawFromTransform(world_T_local));
        LogPoseStoreStats("anchor_applied");
    }

    RawKeyFrameId SelectPoseStoreDebugOptimizationKeyFrame(const RawSubmapId& submap_id) const
    {
        RawKeyFrameId configured{
            submap_id.drone_id,
            submap_id.map_epoch,
            static_cast<uint64_t>(pose_store_debug_opt_kf_id_)};
        if (pose_store_.HasWorldPose(configured))
        {
            return configured;
        }

        const auto keyframe_ids = raw_db_.GetKeyFrameIdsForSubmap(submap_id);
        for (const auto& keyframe_id : keyframe_ids)
        {
            if (pose_store_.HasWorldPose(keyframe_id))
            {
                return keyframe_id;
            }
        }

        return configured;
    }

    void TryPoseStoreDebugOptimization(uint64_t journal_entries)
    {
        if (!pose_store_debug_opt_enabled_ ||
            pose_store_debug_opt_done_ ||
            !pose_store_debug_anchor_done_ ||
            journal_entries < static_cast<uint64_t>(pose_store_debug_opt_after_deltas_))
        {
            return;
        }

        const RawSubmapId submap_id = DebugPoseStoreSubmapId();
        const RawKeyFrameId keyframe_id =
            SelectPoseStoreDebugOptimizationKeyFrame(submap_id);
        const auto current_world_pose = pose_store_.GetWorldPose(keyframe_id);
        if (!current_world_pose)
        {
            RCLCPP_WARN(
                get_logger(),
                "[F1D-SERVER-DEBUG-OPT] status=waiting reason=no_world_pose drone_id=%u epoch=%llu kf=%llu",
                keyframe_id.drone_id,
                static_cast<unsigned long long>(keyframe_id.map_epoch),
                static_cast<unsigned long long>(keyframe_id.local_kf_id));
            return;
        }

        const Eigen::Matrix4d debug_delta =
            PlanarTransform(pose_store_debug_opt_dx_,
                            pose_store_debug_opt_dy_,
                            pose_store_debug_opt_dz_,
                            pose_store_debug_opt_dyaw_);
        const Eigen::Matrix4d optimized_world_T_kf =
            debug_delta * current_world_pose.value();

        RCLCPP_WARN(
            get_logger(),
            "[F1D-SERVER-DEBUG-OPT] status=request drone_id=%u epoch=%llu kf=%llu source=DEBUG_TEST_OPT dx=%.3f dy=%.3f dz=%.3f dyaw=%.3f",
            keyframe_id.drone_id,
            static_cast<unsigned long long>(keyframe_id.map_epoch),
            static_cast<unsigned long long>(keyframe_id.local_kf_id),
            pose_store_debug_opt_dx_,
            pose_store_debug_opt_dy_,
            pose_store_debug_opt_dz_,
            pose_store_debug_opt_dyaw_);

        const auto result =
            pose_store_.SetOptimizedKeyFramePose(
                keyframe_id,
                optimized_world_T_kf,
                raw_db_,
                "DEBUG_TEST_OPT");
        if (!result.success)
        {
            RCLCPP_ERROR(
                get_logger(),
                "[F1D-SERVER-DEBUG-OPT] status=failed drone_id=%u epoch=%llu kf=%llu reason=%s",
                keyframe_id.drone_id,
                static_cast<unsigned long long>(keyframe_id.map_epoch),
                static_cast<unsigned long long>(keyframe_id.local_kf_id),
                result.reason.c_str());
            return;
        }

        pose_store_debug_opt_done_ = true;
        RCLCPP_WARN(
            get_logger(),
            "[F1D-POSESTORE-OPT-POSE-SET] drone_id=%u epoch=%llu kf=%llu source=DEBUG_TEST_OPT",
            keyframe_id.drone_id,
            static_cast<unsigned long long>(keyframe_id.map_epoch),
            static_cast<unsigned long long>(keyframe_id.local_kf_id));
        RCLCPP_WARN(
            get_logger(),
            "[F1D-POSESTORE-CORRECTION-SET] drone_id=%u epoch=%llu kf=%llu source=DEBUG_TEST_OPT dx=%.3f dy=%.3f dz=%.3f dyaw=%.3f",
            keyframe_id.drone_id,
            static_cast<unsigned long long>(keyframe_id.map_epoch),
            static_cast<unsigned long long>(keyframe_id.local_kf_id),
            result.correction_T_latest(0, 3),
            result.correction_T_latest(1, 3),
            result.correction_T_latest(2, 3),
            YawFromTransform(result.correction_T_latest));
        RCLCPP_WARN(
            get_logger(),
            "[F1D-POSESTORE-CORRECTION-STATS] corrections=%llu optimized_kfs=%llu",
            static_cast<unsigned long long>(pose_store_.GetPoseStoreStats().submap_corrections),
            static_cast<unsigned long long>(pose_store_.GetPoseStoreStats().optimized_keyframes));
        LogPoseStoreStats("debug_opt_applied");
    }

    void RegisterPoseStoreKeyFramesFromMap(const OrbMap& map)
    {
        for (const auto& keyframe : map.keyframes)
        {
            const RawKeyFrameId keyframe_id{
                map.drone_id,
                map.map_epoch,
                keyframe.id};
            const auto result =
                pose_store_.RegisterNewKeyFrameIfAnchored(
                    keyframe_id,
                    raw_db_,
                    "DEBUG_TEST_NEW_KF");

            if (result.status == GlobalPoseNewKeyFrameStatus::PoseSet)
            {
                RCLCPP_WARN(
                    get_logger(),
                    "[F1D-POSESTORE-NEW-KF-POSE-SET] drone_id=%u epoch=%llu kf=%llu source=%s",
                    keyframe_id.drone_id,
                    static_cast<unsigned long long>(keyframe_id.map_epoch),
                    static_cast<unsigned long long>(keyframe_id.local_kf_id),
                    result.source.c_str());
            }
            else if (result.status == GlobalPoseNewKeyFrameStatus::CorrectionInherited)
            {
                RCLCPP_WARN(
                    get_logger(),
                    "[F1D-POSESTORE-NEW-KF-CORRECTION-INHERIT] drone_id=%u epoch=%llu kf=%llu correction_source=%s reason=%s",
                    keyframe_id.drone_id,
                    static_cast<unsigned long long>(keyframe_id.map_epoch),
                    static_cast<unsigned long long>(keyframe_id.local_kf_id),
                    result.source.c_str(),
                    result.reason.c_str());
            }
        }
    }

    // F1B/F1C: parámetros y contadores del adaptador ROS. La observabilidad del
    // servidor permanece aquí; el estado raw persistente vive en RawMapDatabase.
    bool use_sim_time_ = false;
    std::string world_frame_ = "world";
    std::string namespace_base_ = "dron";
    int n_drones_ = 1;
    double stats_period_s_ = 2.0;
    bool rawdb_record_enabled_ = true;
    bool rawdb_replay_enabled_ = false;
    std::string rawdb_record_path_;
    std::string rawdb_replay_path_;
    double rawdb_replay_period_sec_ = 0.5;
    uint64_t rawdb_next_arrival_id_ = 0;
    bool pose_store_debug_enabled_ = false;
    int pose_store_debug_anchor_after_deltas_ = 5;
    int pose_store_debug_anchor_drone_id_ = 1;
    int pose_store_debug_anchor_epoch_ = 0;
    double pose_store_debug_anchor_world_x_ = 2.0;
    double pose_store_debug_anchor_world_y_ = 0.0;
    double pose_store_debug_anchor_world_z_ = 0.0;
    double pose_store_debug_anchor_yaw_ = 0.0;
    bool pose_store_debug_opt_enabled_ = false;
    int pose_store_debug_opt_after_deltas_ = 10;
    int pose_store_debug_opt_kf_id_ = 0;
    double pose_store_debug_opt_dx_ = 0.15;
    double pose_store_debug_opt_dy_ = -0.03;
    double pose_store_debug_opt_dz_ = 0.0;
    double pose_store_debug_opt_dyaw_ = 0.05;
    bool pose_store_debug_anchor_done_ = false;
    bool pose_store_debug_opt_done_ = false;
    bool fiducial_sim_enabled_ = true;
    double fiducial_gt_max_dt_sec_ = 1.0;
    int fiducial_gt_buffer_max_samples_ = 250;
    std::vector<int64_t> fiducial_ids_;
    std::vector<double> fiducial_x_;
    std::vector<double> fiducial_y_;
    std::vector<double> fiducial_z_;
    std::vector<double> fiducial_yaw_;
    std::vector<double> fiducial_radius_;
    std::vector<FiducialConfig> fiducials_;
    double fiducial_revisit_error_threshold_m_ = 0.35;
    double fiducial_revisit_yaw_threshold_rad_ = 0.25;
    double fiducial_revisit_rot_threshold_rad_ = 0.35;
    double body_T_camera_x_ = 0.10;
    double body_T_camera_y_ = 0.03;
    double body_T_camera_z_ = 0.03;
    double body_T_camera_roll_deg_ = 0.0;
    double body_T_camera_pitch_deg_ = -90.0;
    double body_T_camera_yaw_deg_ = 90.0;
    bool use_camera_optical_frame_convention_ = true;
    std::string global_sparse_cloud_topic_ = "/global_sparse_cloud";
    double global_map_min_score_to_publish_ = 0.0;
    double global_map_publish_period_sec_ = 1.0;
    bool f1g_full_snapshot_enabled_ = true;
    double f1g_full_snapshot_startup_delay_sec_ = 35.0;
    double f1g_full_snapshot_period_sec_ = 35.0;
    bool f1g_debug_mark_optimized_kf_ = true;
    bool f1g_debug_optimized_kf_done_ = false;
    int pose_graph_max_vertices_ = 24;
    int pose_graph_min_vertices_ = 2;
    int pose_graph_max_path_length_ = 80;
    double pose_graph_fiducial_neighborhood_radius_m_ = 4.0;
    int pose_graph_fiducial_neighborhood_radius_kfs_ = 3;
    int pose_graph_max_temporal_edge_kf_gap_ = 15;
    bool pose_graph_anchor_stop_enabled_ = true;
    bool pose_graph_fiducial_connectivity_enabled_ = true;
    bool pose_graph_branch_selection_enabled_ = true;
    double pose_graph_subdivision_candidate_radius_m_ = 2.0;
    double pose_graph_max_temporal_edge_length_m_ = 4.0;
    double pose_graph_corner_yaw_threshold_rad_ = 0.5235987756;
    bool pose_graph_include_temporal_edges_ = true;
    double pose_graph_temporal_edge_weight_ = 25.0;
    double pose_graph_temporal_edge_weight_sparse_ = 10.0;
    double pose_graph_fiducial_hard_weight_ = 1000000.0;
    double pose_graph_fiducial_target_translation_weight_ = 5000.0;
    double pose_graph_fiducial_target_rotation_weight_ = 2500.0;
    double pose_graph_current_pose_soft_weight_ = 5.0;
    bool f1i_debug_force_task_enabled_ = false;
    double f1i_debug_task_dx_ = 0.75;
    double f1i_debug_task_dy_ = 0.0;
    double f1i_debug_task_dz_ = 0.0;
    double f1i_debug_task_dyaw_ = 0.20;
    bool f1i_debug_task_done_ = false;
    uint64_t f1i_debug_task_counter_ = 0;
    bool f1j_dryrun_enabled_ = true;
    double f1j_dryrun_min_improvement_ratio_ = 0.05;
    double f1j_dryrun_partial_min_improvement_ratio_ = 0.70;
    double f1j_dryrun_max_allowed_delta_t_ = 30.0;
    double f1j_dryrun_partial_max_allowed_delta_t_ = 35.0;
    double f1j_dryrun_max_allowed_delta_yaw_ = 1.2;
    double f1j_dryrun_max_final_error_t_ = 0.35;
    bool f1j_dryrun_require_cost_decrease_ = true;
    double f1j_solver_step_fraction_ = 0.95;
    bool f1j_export_debug_plot_ = false;
    std::string f1j_debug_output_dir_ = "src/codex/archivos_auxiliares";
    bool f1k_apply_enabled_ = true;
    bool f1l_validation_enabled_ = true;
    bool f1l_partial_apply_enabled_ = true;
    int f1l_max_partial_retries_ = 3;
    bool f1l_debug_force_reject_once_ = false;
    bool f1l_debug_force_reject_consumed_ = false;
    int f1l_debug_force_reject_task_id_ = -1;
    double f1l_post_apply_internal_broken_edge_t_ = 2.50;
    double f1l_post_apply_internal_max_growth_t_ = 1.50;
    double f1l_post_apply_fiducial_absurd_error_t_ = 10.0;
    bool f1l_gt_kf_debug_enabled_ = false;
    double f1l_gt_kf_debug_max_dt_sec_ = 1.0;
    bool f1l_debug_animation_enabled_ = false;
    std::string f1l_debug_animation_output_dir_ = "src/codex/archivos_auxiliares/html";
    bool f1l_graph_dump_enabled_ = false;
    std::string f1l_graph_dump_output_dir_ = "src/codex/archivos_auxiliares/repeticiones";

    std::vector<rclcpp::Subscription<OrbMap>::SharedPtr> orb_map_delta_subs_;
    std::vector<rclcpp::Client<GetOrbMap>::SharedPtr> full_snapshot_clients_;
    std::vector<bool> full_snapshot_pending_;
    std::vector<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr> gt_pose_subs_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr global_sparse_cloud_pub_;
    rclcpp::TimerBase::SharedPtr stats_timer_;
    rclcpp::TimerBase::SharedPtr rawdb_save_timer_;
    rclcpp::TimerBase::SharedPtr replay_timer_;
    rclcpp::TimerBase::SharedPtr global_map_publish_timer_;
    rclcpp::TimerBase::SharedPtr full_snapshot_startup_timer_;
    rclcpp::TimerBase::SharedPtr full_snapshot_periodic_timer_;
    sensor_msgs::msg::PointCloud2 last_global_sparse_cloud_;
    bool has_last_global_sparse_cloud_ = false;
    RawMapDatabase raw_db_;
    GlobalPoseStore pose_store_;
    FiducialAnchorManager fiducial_anchor_manager_;
    LandmarkScoreManager score_manager_;
    GlobalMapBuilder global_map_builder_;
    PoseGraphBuilder pose_graph_builder_;
    OptimizationManager optimization_manager_;
    OptimizationDebugExporter optimization_debug_exporter_;
    std::vector<RawJournalEntry> replay_entries_;
    std::map<uint64_t, std::vector<RecordedFiducialObservation>> replay_fiducials_by_arrival_;
    size_t replay_index_ = 0;
    bool replay_done_logged_ = false;
    std::map<uint32_t, std::vector<GroundTruthSample>> gt_buffers_;
    std::map<RawKeyFrameId, DebugGtKeyFramePose> f1l_gt_keyframe_store_;
    std::set<RawKeyFrameId> f1l_gt_debug_missing_logged_;
    std::set<RawKeyFrameId> live_fiducial_observed_keyframes_;
    std::set<uint64_t> f1i_pose_graph_built_task_ids_;
    std::map<RawSubmapId, uint64_t> f1l_partial_checkpoint_task_by_submap_;
    std::map<RawSubmapId, uint64_t> f1l_partial_retry_count_by_submap_;

    mutable std::mutex stats_mutex_;
    uint64_t total_maps_ = 0;
    uint64_t total_keyframes_ = 0;
    uint64_t total_mappoints_ = 0;
    std::set<uint32_t> drones_seen_;
    std::set<std::pair<uint32_t, uint64_t>> epochs_seen_;
    std::map<uint32_t, DroneRxStats> per_drone_stats_;
};

int main(int argc, char** argv)
{
    // F1B: punto de entrada del nodo minimo. Mantenerlo simple facilita saber
    // si un fallo viene del runtime ROS o de logica posterior aun no migrada.
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GlobalMapServer>());
    rclcpp::shutdown();
    return 0;
}

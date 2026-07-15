#pragma once

#include <rclcpp/rclcpp.hpp>

#include "orbslam3_multi/legacy/MultiDroneSystem_antiguo.hpp"

#include "orbslam3_msgs/msg/orb_map.hpp"
#include "orbslam3_msgs/msg/map_correction.hpp"
#include "orbslam3_msgs/msg/fiducial_lock_status.hpp"
#include "orbslam3_msgs/srv/get_orb_map.hpp"
#include "orbslam3_msgs/msg/corrected_key_frame_array.hpp"
#include "orbslam3_msgs/msg/corrected_key_frame.hpp"

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "std_msgs/msg/bool.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"

#include "tf2_ros/transform_broadcaster.h"
#include <tf2/LinearMath/Transform.h>

#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <deque>
#include <limits>
#include <set>
#include <thread>
#include <atomic>
#include <mutex>

/*
 * Servidor global MINIMAL hasta PASO 5.
 *
 * PASO 1: recibe OrbMap de cada wrapper.
 * PASO 2: mantiene una base local por dron.
 * PASO 3: importa KeyFrames/MapPoints a orbslam3_multi::MultiDroneSystem.
 * PASO 4: usa GlobalKeyFrameDatabase de ORBSLAM3_MULTI para candidatos BoW.
 * PASO 5: usa SearchByBoW de ORBSLAM3_MULTI para matches por FeatureVector.
 *
 * No incluye todavía:
 *   - RANSAC SE3 / Sim3.
 *   - g2o.
 *   - fusión de landmarks.
 *   - pose-graph residual antiguo.
 */

struct RecentCorrectedOrbPoseWorld
{
    Eigen::Vector3d position_world =
        Eigen::Vector3d::Zero();

    // Rotación cámara -> world.
    // Según la convención usada en ProjectWorldPointIntoKeyFrame,
    // el eje Z de la cámara es profundidad positiva.
    Eigen::Matrix3d world_R_camera =
        Eigen::Matrix3d::Identity();

    rclcpp::Time stamp;

    bool valid = false;
};

struct DroneMapData
{
    std::unordered_map<uint64_t, orbslam3_msgs::msg::OrbMapPoint> mappoints;
    std::unordered_map<uint64_t, orbslam3_msgs::msg::OrbKeyFrame> keyframes;

    bool has_any_update = false;
    bool has_full_snapshot = false;

    uint64_t current_map_epoch = 0;
    uint64_t last_sequence = 0;

    std::string drone_name;
    std::string local_map_frame;

    // Cámara usada por el wrapper ORB-SLAM3.
    // Necesaria para Fase 4: proyectar landmarks en KeyFrames y estimar
    // expected/seen/missed views.
    bool has_camera_info = false;

    double camera_fx = -1.0;
    double camera_fy = -1.0;
    double camera_cx = -1.0;
    double camera_cy = -1.0;
    double camera_bf = -1.0;

    uint32_t image_width = 0;
    uint32_t image_height = 0;

    geometry_msgs::msg::Transform world_T_local;
    bool has_correction = false;
    bool has_absolute_correction = false;

    geometry_msgs::msg::PoseStamped last_gt_body_pose;
    bool has_gt_body_pose = false;

    // GT queda reservado para fiduciales simulados/diagnóstico.
    // No debe alimentar fused ni filtros de trayectoria.
    rclcpp::Time last_gt_body_pose_receive_time;

    geometry_msgs::msg::PoseStamped last_orb_camera_pose;
    bool has_last_orb_camera_pose = false;

    rclcpp::Time last_orb_camera_pose_receive_time;

    // ORB-TRAJ1:
    // Pose ORB local transformada con world_T_local.
    // Esta trayectoria sí puede usarse para filtros de fused porque no usa GT.
    std::deque<RecentCorrectedOrbPoseWorld> recent_corrected_orb_trajectory_world;

    rclcpp::Time last_update_time;
    rclcpp::Time last_full_snapshot_time;
    rclcpp::Time last_full_snapshot_request_time;

    bool full_snapshot_request_pending = false;

    uint64_t last_delta_sequence = 0;
    bool has_last_delta_sequence = false;

    size_t last_mappoint_count = 0;
    size_t last_keyframe_count = 0;
    uint32_t empty_delta_count = 0;
};

struct HistoricalSubmapMirror
{
    uint32_t drone_id = 0;
    uint64_t map_epoch = 0;

    std::unordered_map<uint64_t, orbslam3_msgs::msg::OrbMapPoint> mappoints;
    std::unordered_map<uint64_t, orbslam3_msgs::msg::OrbKeyFrame> keyframes;

    bool has_any_update = false;
    bool has_full_snapshot = false;

    uint64_t last_sequence = 0;

    std::string drone_name;
    std::string local_map_frame;

    bool has_camera_info = false;

    double camera_fx = -1.0;
    double camera_fy = -1.0;
    double camera_cx = -1.0;
    double camera_cy = -1.0;
    double camera_bf = -1.0;

    uint32_t image_width = 0;
    uint32_t image_height = 0;

    rclcpp::Time first_seen_time;
    rclcpp::Time last_update_time;
    rclcpp::Time last_full_snapshot_time;
};

struct GlobalSparsePoint
{
    uint32_t drone_id = 0;
    uint64_t global_mappoint_id = 0;

    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

    std::array<uint8_t, 32> descriptor{};
};

struct FusedTrackObservation
{
    Eigen::Vector3d position_world =
        Eigen::Vector3d::Zero();

    std::array<uint8_t, 32> descriptor{};

    // Fuentes de esta observación. Si viene de un grupo merged,
    // puede contener varios drones/submapas/MapPoints.
    std::set<uint32_t> source_drones;
    std::set<std::pair<uint32_t, uint64_t>> source_submaps;
    std::unordered_set<uint64_t> source_mappoint_ids;

    uint32_t max_observations = 0;
    double max_found_ratio = 0.0;

    bool is_merged = false;
    bool has_inter_drone_support = false;
    bool has_confirmed_source = false;
    bool has_provisional_source = false;

    size_t group_size = 1;

    double weight = 1.0;
    double initial_score = 0.10;

    // ============================================================
    // FASE 6:
    // Observación creada explícitamente por fusión/refuerzo de loop.
    // ============================================================

    bool from_loop_fusion = false;

    uint64_t loop_event_id = 0;

    std::string loop_relation_type;

    std::string loop_fusion_reason;

    double loop_pair_distance_m =
        std::numeric_limits<double>::infinity();

    int loop_pair_hamming =
        std::numeric_limits<int>::max();
};


struct FusedLandmarkTrack
{
    uint64_t fused_id = 0;

    Eigen::Vector3d position_world =
        Eigen::Vector3d::Zero();

    std::array<uint8_t, 32> descriptor{};

    // Score normalizado [0, 1].
    double score = 0.0;

    // Peso acumulado para media ponderada.
    double total_weight = 0.0;

    uint32_t total_observations = 0;
    uint32_t successful_reobservations = 0;

    // Número total de veces que el servidor consideró que el track
    // estaba en una zona activa pero no recibió asociación.
    uint32_t expected_but_missed = 0;

    // Misses consecutivos. Se resetea cuando el track se observa.
    uint32_t consecutive_misses = 0;

    
    std::unordered_set<uint64_t> source_mappoint_ids;
    std::set<uint32_t> source_drones;
    std::set<std::pair<uint32_t, uint64_t>> source_submaps;

    bool has_confirmed_source = false;
    bool has_provisional_source = false;
    bool has_inter_drone_support = false;

    // TRACKS-1B:
    // True si alguna observación que actualizó este track venía de un grupo merged.
    // Los tracks sin soporte merged son singletons persistentes y deben publicarse
    // solo tras varias reobservaciones.
    bool has_merged_support = false;

    // Guardamos un número limitado de descriptores observados para calcular medoid.
    std::vector<std::pair<std::array<uint8_t, 32>, double>> descriptor_samples;

    rclcpp::Time first_seen;
    rclcpp::Time last_seen;
    rclcpp::Time last_update;

    bool valid = false;
};


struct FusedTrackUpdateStats
{
    size_t observations = 0;

    size_t tracks_before = 0;
    size_t tracks_after = 0;

    size_t associated = 0;
    size_t created = 0;
    size_t updated = 0;

    size_t published = 0;
    size_t published_inter_tracks = 0;
    size_t published_merged_tracks = 0;
    size_t published_singleton_tracks = 0;

    size_t hidden_low_score = 0;
    size_t hidden_singleton_not_reobserved = 0;
    size_t hidden_merged_not_reobserved = 0;
    size_t invalid_tracks = 0;

    size_t inter_observations = 0;
    size_t merged_observations = 0;
    size_t singleton_observations = 0;

    size_t assoc_tracks_checked = 0;
    size_t assoc_distance_tests = 0;
    size_t assoc_descriptor_tests = 0;

    // TRACKS-STAB1.
    size_t singleton_geometry_frozen = 0;

    // TRACKS-MERGE1 / PERF1C-safe.
    size_t track_merge_pairs_checked = 0;
    size_t track_merge_distance_tests = 0;
    size_t track_merge_descriptor_tests = 0;
    size_t track_merge_accepted = 0;
    size_t track_merge_erased = 0;

    size_t track_merge_intra = 0;
    size_t track_merge_cross_epoch = 0;
    size_t track_merge_inter = 0;

    size_t track_merge_reject_descriptor = 0;
    size_t track_merge_reject_distance = 0;
    size_t track_merge_limit_hit = 0;

    // TRACKS-PERF1A.
    size_t perf_observations_before_cap = 0;
    size_t perf_observations_after_cap = 0;
    size_t perf_observations_dropped_by_cap = 0;

    // TRACKS2A.
    size_t stale_decayed_tracks = 0;
    size_t expected_missed_tracks = 0;
    size_t removed_low_score_tracks = 0;
    size_t protected_tracks = 0;

    // TRACKS2B.
    // Diagnóstico expected-visible real por proyección.
    size_t expected_visible_tracks_checked = 0;
    size_t expected_visible_tracks_positive = 0;
    size_t expected_visible_tracks_penalized = 0;
    size_t expected_visible_kf_checks = 0;
    size_t expected_visible_kf_hits = 0;
    size_t expected_visible_skipped_no_camera = 0;
    size_t expected_visible_skipped_no_correction = 0;

    double mean_score = 0.0;
    double max_score = 0.0;

    // ============================================================
    // FASE 6:
    // Estadísticas específicas de fusión guiada por loops.
    // ============================================================

    size_t loop_fusion_observations = 0;
    size_t loop_fusion_track_updates = 0;
    size_t loop_fusion_track_creations = 0;
    size_t loop_fusion_source_track_merges = 0;
    size_t loop_fusion_score_updates = 0;

    double loop_fusion_score_gain_sum = 0.0;
    double loop_fusion_score_gain_max = 0.0;
};

struct GlobalKeyFrameLite
{
    uint32_t drone_id = 0;
    uint64_t map_epoch = 0;
    uint64_t local_keyframe_id = 0;
    uint64_t global_keyframe_id = 0;

    geometry_msgs::msg::Pose local_pose;
    geometry_msgs::msg::Pose global_pose;

    std::vector<orbslam3_msgs::msg::OrbKeyPoint> keypoints;
    std::vector<uint64_t> mappoint_ids;
};

struct BowCandidateJob
{
    uint64_t query_global_kf_id = 0;
    uint64_t candidate_global_kf_id = 0;
    double score = 0.0;

    // Fase 7:
    // Origen del par BoW.
    // Ejemplos:
    //   bow_db
    //   inter_anchor_explicit_pair
    //   inter_anchor_assist_requeue
    std::string reason;
};

struct LoopThroughputStats
{
    rclcpp::Time window_start;

    size_t kfs_enqueued_for_bow = 0;
    size_t bow_query_kfs_processed = 0;
    size_t bow_query_kfs_skipped_bad = 0;
    size_t bow_query_kfs_skipped_no_drone_data = 0;

    size_t bow_candidates_raw = 0;
    size_t bow_candidates_after_frustum = 0;

    size_t bow_candidates_queued = 0;
    size_t bow_reject_self = 0;
    size_t bow_reject_invalid = 0;
    size_t bow_reject_tested = 0;
    size_t bow_reject_already_queued = 0;

    size_t frustum_queries = 0;
    size_t frustum_candidates_before = 0;
    size_t frustum_candidates_after = 0;
    size_t frustum_rejected = 0;
    size_t frustum_no_visibility_info = 0;
    size_t frustum_fallback_used = 0;

    size_t candidate_jobs_processed = 0;
    size_t candidate_jobs_skipped_invalid = 0;
    size_t candidate_jobs_skipped_low_bow_matches = 0;
    size_t candidate_jobs_accepted_for_geometry = 0;

    size_t geometry_attempts = 0;
    size_t geometry_rejected = 0;
    size_t geometry_strong_loops = 0;

    size_t strong_loops_intra_same_epoch = 0;
    size_t strong_loops_cross_epoch_same_drone = 0;
    size_t strong_loops_inter_drone = 0;

    size_t loop_action_fuse_now = 0;
    size_t loop_action_optimize_then_fuse = 0;
    size_t loop_action_hold = 0;

    size_t loop_ready_for_fusion = 0;
    size_t loop_local_task_deferred = 0;

    size_t loop_fusion_events_ready = 0;
    size_t loop_fusion_events_processed = 0;
    size_t loop_fusion_pairs_accepted = 0;
    size_t loop_fusion_observations_added = 0;

    size_t global_fiducial_opt_tasks = 0;
    size_t local_loop_opt_tasks = 0;
};

struct LoopViewMapErrorInfo
{
    bool valid = false;

    uint64_t query_global_kf_id = 0;
    uint64_t candidate_global_kf_id = 0;

    Eigen::Matrix4d measured_candidate_T_query =
        Eigen::Matrix4d::Identity();

    Eigen::Matrix4d predicted_candidate_T_query =
        Eigen::Matrix4d::Identity();

    Eigen::Matrix4d delta_measured_to_predicted =
        Eigen::Matrix4d::Identity();

    double measured_t_norm_m =
        std::numeric_limits<double>::infinity();

    double predicted_t_norm_m =
        std::numeric_limits<double>::infinity();

    double delta_t_m =
        std::numeric_limits<double>::infinity();

    double delta_yaw_deg =
        std::numeric_limits<double>::infinity();

    double delta_roll_deg =
        std::numeric_limits<double>::infinity();

    double delta_pitch_deg =
        std::numeric_limits<double>::infinity();

    double delta_rot_deg =
        std::numeric_limits<double>::infinity();

    std::string query_pose_reason;
    std::string candidate_pose_reason;
    std::string reason;

    std::string action;
    std::string action_reason;

    // Documentación explícita:
    // - query/current view: MapPoints ORB del KeyFrame actual, sin filtrar por score fused.
    // - candidate/map side: KeyFrame candidato del mapa global con pose world actual.
    std::string query_source =
        "current_orb_keyframe_all_mappoints_no_fused_score_filter";

    std::string candidate_source =
        "global_candidate_keyframe_world_pose_current_map";
};

struct UnifiedLoopRuntimeEvent
{
    uint64_t event_id = 0;

    uint64_t query_global_kf_id = 0;
    uint64_t candidate_global_kf_id = 0;

    uint32_t query_drone = 0;
    uint64_t query_epoch = 0;
    uint64_t query_local_kf = 0;

    uint32_t candidate_drone = 0;
    uint64_t candidate_epoch = 0;
    uint64_t candidate_local_kf = 0;

    std::string relation_type;
    std::string source_reason;

    bool strong = false;
    bool stored = false;

    bool usable_for_optimization = false;
    bool usable_for_fusion = false;
    bool usable_for_anchor = false;

    double quality = 0.0;

    int initial_matches = 0;
    int ransac_inliers = 0;
    int projection_matches = 0;
    int final_inliers = 0;

    double inlier_ratio = 0.0;
    double mean_geom_error = 0.0;
    double max_geom_error = 0.0;

    Eigen::Matrix4d measured_candidate_T_query =
        Eigen::Matrix4d::Identity();

    // FASE 4:
    // Pares de MapPoints procedentes del matching que dio lugar al loop.
    //
    // En una fase ideal usaríamos solo inliers finales de geometría.
    // Por ahora los guardamos y más tarde los filtramos de forma estricta
    // por distancia world + descriptor + ready_for_fusion.
    std::vector<std::pair<uint64_t, uint64_t>> matched_mappoint_pairs;

    bool has_error_before_opt = false;
    double error_t_before_opt = std::numeric_limits<double>::infinity();
    double error_yaw_before_opt = std::numeric_limits<double>::infinity();
    double error_rot_before_opt = std::numeric_limits<double>::infinity();
    std::string error_before_reason;

    // ============================================================
    // FASE C:
    // Error explícito vista actual ↔ mapa.
    // Este error se calcula comparando:
    // measured_candidate_T_query, obtenido por geometría,
    // contra predicted_candidate_T_query, obtenido desde las poses actuales.
    // ============================================================

    double viewmap_measured_t_norm_m =
        std::numeric_limits<double>::infinity();

    double viewmap_predicted_t_norm_m =
        std::numeric_limits<double>::infinity();

    double viewmap_delta_roll_deg =
        std::numeric_limits<double>::infinity();

    double viewmap_delta_pitch_deg =
        std::numeric_limits<double>::infinity();

    std::string viewmap_query_source;
    std::string viewmap_candidate_source;

    std::string dryrun_action;
    std::string dryrun_reason;

    bool needs_optimization = false;
    bool ready_for_fusion = false;

    // ============================================================
    // FASE 5:
    // Control de optimización iterativa por loop.
    // ============================================================

    int optimization_attempts = 0;

    bool optimization_converged = false;
    bool optimization_failed = false;
    bool optimization_retry_requested = false;

    double best_error_t = std::numeric_limits<double>::infinity();
    double best_error_yaw = std::numeric_limits<double>::infinity();

    double last_optimization_error_t = std::numeric_limits<double>::infinity();
    double last_optimization_error_yaw = std::numeric_limits<double>::infinity();

    rclcpp::Time last_optimization_request_time;
    rclcpp::Time last_optimization_eval_time;

    std::string optimization_status;
    std::string optimization_status_reason;

    bool post_opt_checked = false;
    bool has_error_after_opt = false;
    double error_t_after_opt = std::numeric_limits<double>::infinity();
    double error_yaw_after_opt = std::numeric_limits<double>::infinity();
    double error_rot_after_opt = std::numeric_limits<double>::infinity();
    std::string error_after_reason;

    // FASE 4:
    // Estado de fusión/reobservación guiada por loop.
    bool fusion_attempted = false;
    bool fusion_completed = false;
    int fusion_attempts = 0;

    size_t fusion_pairs_total = 0;
    size_t fusion_pairs_accepted = 0;

    std::string fusion_last_reason;

    rclcpp::Time created_time;
    rclcpp::Time last_update_time;
};



// ============================================================
// SUBCLOUD PIPELINE:
// Estructuras nuevas para sustituir progresivamente la lógica
// legacy KeyFrame↔KeyFrame por comparación query-cloud ↔
// global-subcloud.
// En Fase 3 solo se definen y se diagnostican; no cambian
// comportamiento.
// ============================================================

struct LoopPoint
{
    uint64_t global_mappoint_id = 0;

    uint32_t drone_id = 0;
    uint64_t map_epoch = 0;
    uint64_t local_mappoint_id = 0;

    // FASE 10R-B:
    // KeyFrame que originó este punto dentro de la query/candidate cloud.
    // No modifica ORB-SLAM3; solo permite covisibilidad global del servidor.
    uint64_t source_global_kf_id = 0;

    Eigen::Vector3d p_world =
        Eigen::Vector3d::Zero();

    Eigen::Vector3d p_local =
        Eigen::Vector3d::Zero();

    std::array<uint8_t, 32> descriptor{};

    uint32_t observations_count = 0;
    double found_ratio = 0.0;

    bool has_world = false;
    bool valid = false;

    bool from_confirmed_anchor = false;
    bool from_provisional_anchor = false;

    std::string source;
    std::string reject_reason;
};

struct LoopPointMatch
{
    size_t query_index = 0;
    size_t candidate_index = 0;

    uint64_t query_global_mp_id = 0;

    // Si el candidato viene de fused tracks, se usará este id.
    uint64_t candidate_fused_id = 0;

    // Si el candidato viene de un MapPoint global/importado, se usará este id.
    uint64_t candidate_global_mp_id = 0;

    int hamming =
        std::numeric_limits<int>::max();

    double distance_before_m =
        std::numeric_limits<double>::infinity();

    double residual_after_m =
        std::numeric_limits<double>::infinity();

    bool ransac_inlier = false;

    // FASE 10R-B:
    // Snapshots de los puntos en el momento exacto del matching/RANSAC.
    // Esto evita que la fusión falle después por mirrors que ya cambiaron,
    // por IDs reconstruidos, o por candidatos procedentes de subnubes.
    bool query_snapshot_valid = false;
    bool candidate_snapshot_valid = false;

    LoopPoint query_snapshot;
    LoopPoint candidate_snapshot;
};

struct LoopSubcloudEvent
{
    uint64_t event_id = 0;

    uint64_t query_global_kf_id = 0;
    uint32_t query_drone = 0;
    uint64_t query_epoch = 0;
    uint64_t query_local_kf = 0;

    std::string relation_metadata;
    std::string search_mode;

    size_t query_points = 0;
    size_t candidate_points = 0;
    size_t matches_raw = 0;
    size_t ransac_inliers = 0;

    double inlier_ratio = 0.0;
    double coverage = 0.0;

    double error_t_m =
        std::numeric_limits<double>::infinity();

    double error_yaw_deg =
        std::numeric_limits<double>::infinity();

    double error_rot_deg =
        std::numeric_limits<double>::infinity();

    double mean_residual_m =
        std::numeric_limits<double>::infinity();

    double max_residual_m =
        std::numeric_limits<double>::infinity();

    Eigen::Matrix4d world_T_query_correction =
        Eigen::Matrix4d::Identity();

    std::string decision = "UNDECIDED";
    std::string reason = "not_processed_yet";

    // ============================================================
    // FASE 10R-A:
    // Estado de fusión real conservadora desde FUSION_EVENT subcloud.
    // ============================================================

    bool subcloud_fusion_attempted = false;
    bool subcloud_fusion_completed = false;

    int subcloud_fusion_attempts = 0;

    // ============================================================
    // FASE 11R-A:
    // Tarea local de optimización derivada de LOOP-SUBCLOUD-ERROR.
    // Dry-run: se crea y se diagnostica, pero no se ejecuta.
    // ============================================================

    bool local_loop_task_created = false;
    bool local_loop_task_dry_run = true;

    uint64_t local_loop_task_id = 0;
    uint64_t pending_opt_region_id = 0;

    std::string local_loop_task_state = "NONE";
    std::string local_loop_task_reason;

    size_t subcloud_fusion_pairs_total = 0;
    size_t subcloud_fusion_pairs_inliers = 0;
    size_t subcloud_fusion_pairs_accepted = 0;
    size_t subcloud_fusion_observations_added = 0;

    std::string subcloud_fusion_last_reason;

    std::vector<LoopPointMatch> matches;

    rclcpp::Time created_time;
    rclcpp::Time last_update_time;
};

enum class OptimizationTaskType
{
    LOCAL_LOOP,
    GLOBAL_FIDUCIAL
};

struct OptimizationTask
{
    uint64_t task_id = 0;

    OptimizationTaskType type =
        OptimizationTaskType::LOCAL_LOOP;

    uint64_t source_event_id = 0;

    uint32_t drone_id = 0;
    uint64_t map_epoch = 0;

    // FASE 11R-A:
    // KeyFrame exacto que disparó la tarea.
    uint64_t query_global_kf_id = 0;
    uint64_t query_local_kf_id = 0;

    uint64_t pending_opt_region_id = 0;

    size_t supporting_matches = 0;
    size_t supporting_inliers = 0;

    double inlier_ratio = 0.0;
    double coverage = 0.0;

    double mean_residual_m =
        std::numeric_limits<double>::infinity();

    double max_residual_m =
        std::numeric_limits<double>::infinity();

    double error_t_m =
        std::numeric_limits<double>::infinity();

    double error_yaw_deg =
        std::numeric_limits<double>::infinity();

    double error_rot_deg =
        std::numeric_limits<double>::infinity();

    double error_chi2 =
        std::numeric_limits<double>::infinity();

    int attempts = 0;
    int max_attempts = 1;

    bool dry_run = true;

    // ============================================================
    // FASE 11R-B:
    // Ventana local de optimización construida en dry-run.
    // No ejecuta optimizador todavía.
    // ============================================================

    bool local_window_attempted = false;
    bool local_window_built = false;

    uint64_t local_window_id = 0;

    size_t local_window_total_kfs = 0;
    size_t local_window_variable_kfs = 0;
    size_t local_window_fixed_kfs = 0;
    size_t local_window_query_side_kfs = 0;
    size_t local_window_candidate_side_kfs = 0;
    size_t local_window_boundary_kfs = 0;
    size_t local_window_excluded_pending_kfs = 0;

    std::string local_window_state = "NOT_BUILT";
    std::string local_window_reason;

    // ============================================================
    // FASE 11R-C:
    // Optimización numérica dry-run sobre copia.
    // No aplica poses al servidor todavía.
    // ============================================================

    bool local_numeric_dryrun_attempted = false;
    bool local_numeric_dryrun_done = false;
    bool local_numeric_dryrun_valid = false;

    uint64_t local_numeric_dryrun_result_id = 0;

    double local_numeric_before_mean_residual_m =
        std::numeric_limits<double>::infinity();

    double local_numeric_after_mean_residual_m =
        std::numeric_limits<double>::infinity();

    double local_numeric_before_max_residual_m =
        std::numeric_limits<double>::infinity();

    double local_numeric_after_max_residual_m =
        std::numeric_limits<double>::infinity();

    double local_numeric_predicted_improvement_m = 0.0;
    double local_numeric_predicted_improvement_ratio = 0.0;

    std::string local_numeric_dryrun_state = "NOT_RUN";
    std::string local_numeric_dryrun_reason;

    // ============================================================
    // FASE 12R-A:
    // Aplicación real, pero solo al servidor, de un dry-run local
    // ya validado. No toca ORB-SLAM3 interno.
    // ============================================================

    bool local_apply_attempted = false;
    bool local_apply_done = false;
    bool local_apply_valid = false;

    uint64_t local_apply_id = 0;
    uint64_t local_apply_result_id = 0;

    size_t local_apply_kfs_moved = 0;

    double local_apply_max_delta_t_m = 0.0;
    double local_apply_max_delta_yaw_deg = 0.0;
    double local_apply_max_delta_rot_deg = 0.0;

    std::string local_apply_state = "NOT_APPLIED";
    std::string local_apply_reason;

    rclcpp::Time created_time;
    rclcpp::Time last_attempt_time;

    std::string state = "PENDING";
    std::string reason;
};


// ============================================================
// FASE 5R:
// Estado ligero por KeyFrame y cola de trabajos subcloud.
// No procesa loops todavía: solo recuerda estado y agenda jobs.
// ============================================================

struct SubcloudKeyFrameLoopState
{
    uint64_t global_kf_id = 0;

    uint32_t drone_id = 0;
    uint64_t map_epoch = 0;
    uint64_t local_kf_id = 0;

    bool has_anchor = false;

    bool query_cloud_checked = false;

    bool candidate_subcloud_checked = false;
    bool candidate_subcloud_built = false;

    bool loop_job_scheduled = false;
    bool loop_job_processed = false;

    bool match_attempted = false;
    bool error_computed = false;

    // FASE 9R:
    // Decisión dry-run calculada desde LOOP-SUBCLOUD-ERROR.
    // No cambia comportamiento todavía.
    bool decision_computed = false;

    bool placement_validated = false;
    bool fusion_done = false;
    bool local_optimization_done = false;

    // FASE 11R-A:
    // KeyFrame recibido durante una optimización pendiente.
    // Se publica provisionalmente, pero no se usa para BoW/subcloud/fusión
    // hasta que la optimización local termine y se haga recheck.
    bool pending_after_opt_start = false;
    bool blocked_by_pending_optimization = false;

    uint64_t pending_opt_region_id = 0;
    uint64_t pending_opt_task_id = 0;

    rclcpp::Time pending_opt_mark_time;

    bool failed_no_candidate = false;
    bool failed_no_matches = false;
    bool failed_bad_geometry = false;

    size_t last_query_points = 0;
    size_t last_candidate_points = 0;
    size_t last_matches = 0;
    size_t last_inliers = 0;

    double last_error_t_m =
        std::numeric_limits<double>::infinity();

    double last_error_yaw_deg =
        std::numeric_limits<double>::infinity();

    double last_mean_residual_m =
        std::numeric_limits<double>::infinity();

    double last_coverage = 0.0;

    double last_max_residual_m =
        std::numeric_limits<double>::infinity();

    std::string last_loop_decision = "UNDECIDED";
    std::string last_loop_decision_reason;

    rclcpp::Time first_seen_time;
    rclcpp::Time last_update_time;
    rclcpp::Time last_attempt_time;
    rclcpp::Time last_success_time;
    rclcpp::Time cooldown_until;

    std::string status = "NEW";
    std::string reason;
};

struct SubcloudLoopCheckJob
{
    uint64_t job_id = 0;

    uint64_t query_global_kf_id = 0;

    uint32_t query_drone = 0;
    uint64_t query_epoch = 0;
    uint64_t query_local_kf = 0;

    std::string trigger;
    // BOW_REVISIT
    // BOW_ANCHOR_CANDIDATE
    // GEOMETRIC_OVERLAP
    // CONFLICT_CHECK
    // MANUAL_DIAG

    int priority = 0;

    // FASE 6R2:
    // KeyFrames candidatos producidos por BoW. Sirven como semillas
    // para construir candidate subcloud sin hacer scan global/frustum
    // de todo el mapa.
    std::vector<uint64_t> candidate_seed_kf_ids;
    std::vector<float> candidate_seed_scores;

    size_t bow_candidates_before_frustum = 0;
    size_t bow_candidates_after_frustum = 0;
    size_t bow_legacy_jobs_queued = 0;

    bool bow_frustum_gating_active = false;
    bool bow_frustum_fallback_used = false;

    std::string bow_frustum_gating_reason;

    rclcpp::Time created_time;

    std::string reason;
};

// Placeholder de región validada.
// En Fase 5R solo guardamos la estructura; se empezará a rellenar
// cuando tengamos LOOP-SUBCLOUD-ERROR y fusión desde eventos subcloud.
struct SubcloudValidatedRegion
{
    uint64_t region_id = 0;

    Eigen::Vector3d center_world =
        Eigen::Vector3d::Zero();

    double radius_m = 0.0;

    std::unordered_set<uint64_t> supporting_keyframes;
    std::unordered_set<uint64_t> supporting_fused_tracks;

    rclcpp::Time created_time;
    rclcpp::Time last_update_time;

    bool valid = false;

    std::string reason;
};



struct LoopFusionStats
{
    size_t events_seen = 0;
    size_t events_ready = 0;
    size_t events_processed = 0;
    size_t events_completed = 0;
    size_t events_skipped = 0;

    size_t pairs_total = 0;
    size_t pairs_accepted = 0;
    size_t pairs_same_global_mp = 0;
    size_t pairs_zero = 0;
    size_t pairs_missing_mp = 0;
    size_t pairs_bad_mp = 0;
    size_t pairs_no_world = 0;
    size_t pairs_bad_descriptor = 0;
    size_t pairs_reject_distance = 0;
    size_t pairs_reject_hamming = 0;

    size_t observations_added = 0;
    size_t merged_observations_added = 0;
    size_t reobserve_observations_added = 0;

    double mean_accept_dist = 0.0;
    double max_accept_dist = 0.0;
};

// ============================================================
// FASE 10R-B:
// Evidencia global por MapPoint y covisibilidad global por KeyFrame.
// Esto pertenece al servidor, no a ORB-SLAM3 interno.
// ============================================================

struct SubcloudMapPointEvidence
{
    uint64_t global_mappoint_id = 0;

    size_t ransac_inlier_count = 0;
    size_t ransac_outlier_count = 0;

    size_t fusion_positive_count = 0;
    size_t fusion_negative_count = 0;

    double score_delta = 0.0;

    rclcpp::Time first_update_time;
    rclcpp::Time last_update_time;

    std::string last_reason;
};

struct SubcloudKeyFrameCovisibilityEdge
{
    uint64_t kf_a = 0;
    uint64_t kf_b = 0;

    size_t events = 0;
    size_t accepted_pairs = 0;

    size_t ransac_inliers = 0;

    double residual_sum = 0.0;
    double hamming_sum = 0.0;

    double mean_residual = 0.0;
    double mean_hamming = 0.0;

    double max_residual = 0.0;

    std::unordered_set<uint64_t> source_mappoint_ids;

    rclcpp::Time first_update_time;
    rclcpp::Time last_update_time;

    bool valid = false;

    std::string last_reason;
};



// ============================================================
// FASE 12R-B:
// Grafo persistente de restricciones KeyFrame-KeyFrame.
// Este grafo NO toca ORB-SLAM3 interno.
// Sirve para dar rigidez a zonas fusionadas/validadas/fiduciales
// y para limitar movimientos en optimizaciones locales.
// ============================================================

struct LocalOptKeyFrameConstraintEdge
{
    uint64_t kf_a = 0;
    uint64_t kf_b = 0;

    // true cuando kf_a == kf_b y representa un prior absoluto.
    bool absolute_prior = false;

    // true para priors fiduciales duros.
    bool hard_fixed_prior = false;

    // ============================================================
    // FASE 12R-B2:
    // Clasificación del prior fiducial según error contra pose real.
    // No todo KF dentro de un fiducial es hard-fixed.
    // ============================================================

    bool good_fiducial_prior = false;
    bool soft_fiducial_prior = false;
    bool fiducial_global_debt = false;

    // Medida relativa a_T_b.
    // Para absolute_prior puede quedar Identity.
    Eigen::Matrix4d measured_a_T_b =
        Eigen::Matrix4d::Identity();

    // Pose absoluta usada por priors fiduciales.
    Eigen::Matrix4d measured_world_T_a =
        Eigen::Matrix4d::Identity();

    double strength = 0.0;
    double translation_weight = 0.0;
    double rotation_weight = 0.0;

    size_t support_events = 0;
    size_t support_pairs = 0;
    size_t support_inliers = 0;

    double residual_sum = 0.0;
    double mean_residual = 0.0;
    double max_residual = 0.0;

    double last_improvement_ratio = 0.0;
    double improvement_ratio_sum = 0.0;

    std::unordered_set<uint64_t> source_event_ids;
    std::unordered_set<uint64_t> source_mappoint_ids;

    std::string source_type = "UNKNOWN";
    std::string source_detail;
    std::string last_reason;

    rclcpp::Time first_update_time =
        rclcpp::Time(0, 0, RCL_ROS_TIME);

    rclcpp::Time last_update_time =
        rclcpp::Time(0, 0, RCL_ROS_TIME);

    bool active = false;
};

struct LocalOptKeyFrameConstraintSummary
{
    uint64_t global_kf_id = 0;

    size_t active_edges = 0;
    size_t fusion_edges = 0;
    size_t covisibility_edges = 0;
    size_t local_opt_edges = 0;

    // FASE 12R-C:
    // Aristas objetivo de optimización local.
    // No son rigidez del grafo; representan "este loop debe reducir error".
    size_t local_opt_objective_edges = 0;

    size_t fiducial_edges = 0;
    size_t hard_fiducial_edges = 0;
    size_t soft_fiducial_edges = 0;
    size_t fiducial_global_debt_edges = 0;
    size_t boundary_edges = 0;

    double strength_sum = 0.0;
    double max_strength = 0.0;

    double fusion_strength = 0.0;
    double covisibility_strength = 0.0;
    double local_opt_strength = 0.0;

    // FASE 12R-C:
    // Fuerza de objetivo de loop local. No entra en strength_sum rígido.
    double local_opt_objective_strength = 0.0;

    double fiducial_strength = 0.0;
    double hard_fiducial_strength = 0.0;
    double soft_fiducial_strength = 0.0;
    double fiducial_global_debt_strength = 0.0;
    double boundary_strength = 0.0;

    bool hard_fiducial_prior = false;
    bool soft_fiducial_prior = false;
    bool fiducial_global_debt = false;
    bool local_opt_objective_support = false;

    bool hard_fixed_by_graph = false;

    double alpha_scale = 1.0;
    double pose_prior_bonus = 0.0;

    std::string reason;
};



struct SubcloudPendingOptimizationRegion
{
    uint64_t region_id = 0;
    uint64_t source_event_id = 0;
    uint64_t source_task_id = 0;

    uint64_t query_global_kf_id = 0;

    uint32_t query_drone = 0;
    uint64_t query_epoch = 0;
    uint64_t query_local_kf = 0;

    std::set<uint64_t> affected_submap_keys;
    std::set<uint64_t> pending_after_opt_start_kfs;

    size_t matches = 0;
    size_t inliers = 0;

    double error_t_m =
        std::numeric_limits<double>::infinity();

    double error_yaw_deg =
        std::numeric_limits<double>::infinity();

    double error_rot_deg =
        std::numeric_limits<double>::infinity();

    double mean_residual_m =
        std::numeric_limits<double>::infinity();

    double max_residual_m =
        std::numeric_limits<double>::infinity();

    bool dry_run = true;
    bool active = true;

    rclcpp::Time created_time;
    rclcpp::Time last_update_time;

    std::string state = "OPTIMIZATION_PENDING";
    std::string reason;
};




// ============================================================
// FASE 11R-B:
// Ventana local de optimización.
// Solo se construye y se loguea. No modifica poses.
// ============================================================

struct LocalOptimizationWindowKeyFrame
{
    uint64_t global_kf_id = 0;

    uint32_t drone_id = 0;
    uint64_t map_epoch = 0;
    uint64_t local_kf_id = 0;

    std::string side = "UNKNOWN";
    std::string role = "UNKNOWN";

    bool variable = false;
    bool fixed = false;
    bool excluded = false;

    bool query_side = false;
    bool candidate_side = false;
    bool boundary = false;

    bool pending_after_opt_start = false;
    bool has_anchor = false;
    bool placement_validated = false;
    bool fusion_done = false;
    bool has_fiducial_prior = false;

    Eigen::Vector3d position_world =
        Eigen::Vector3d::Zero();

    double distance_to_query_center_m =
        std::numeric_limits<double>::infinity();

    double distance_to_candidate_center_m =
        std::numeric_limits<double>::infinity();

    size_t inlier_support = 0;
    size_t covisibility_support = 0;
    size_t temporal_distance = 0;

    double confidence_weight = 1.0;
    double pose_prior_weight = 1.0;

    // ============================================================
    // FASE 12R-B:
    // Resumen del grafo de restricciones para este KF.
    // Se usa para fijar fiduciales y reducir movilidad de zonas fuertes.
    // ============================================================

    size_t constraint_edges = 0;
    double constraint_strength = 0.0;
    double constraint_max_strength = 0.0;
    double constraint_alpha_scale = 1.0;
    double constraint_pose_prior_bonus = 0.0;

    // FASE 12R-C:
    // Soporte del objetivo de loop local. No es rigidez.
    size_t local_opt_objective_edges = 0;
    double local_opt_objective_strength = 0.0;
    bool local_opt_objective_support = false;

    bool hard_fiducial_constraint = false;
    bool hard_fixed_by_constraint_graph = false;

    std::string constraint_reason;

    std::string reason;
};

struct LocalOptimizationWindow
{
    uint64_t window_id = 0;

    uint64_t task_id = 0;
    uint64_t source_event_id = 0;
    uint64_t pending_region_id = 0;

    uint64_t query_global_kf_id = 0;

    uint32_t query_drone = 0;
    uint64_t query_epoch = 0;
    uint64_t query_local_kf = 0;

    Eigen::Vector3d query_center_world =
        Eigen::Vector3d::Zero();

    Eigen::Vector3d candidate_center_world =
        Eigen::Vector3d::Zero();

    bool has_query_center = false;
    bool has_candidate_center = false;

    std::unordered_map<uint64_t, LocalOptimizationWindowKeyFrame> keyframes;

    std::vector<uint64_t> candidate_seed_kfs;
    std::vector<uint64_t> excluded_pending_kfs;

    size_t query_seed_count = 0;
    size_t candidate_seed_count = 0;

    // FASE 11R-B:
    // Total de KeyFrames pertenecientes a cada lado de la ventana.
    // No confundir con *_seed_count, que solo cuenta semillas iniciales.
    size_t query_side_count = 0;
    size_t candidate_side_count = 0;

    size_t query_neighbor_count = 0;
    size_t candidate_neighbor_count = 0;
    size_t covisibility_neighbor_count = 0;
    size_t boundary_count = 0;
    size_t excluded_pending_count = 0;

    size_t variable_count = 0;
    size_t fixed_count = 0;
    size_t fiducial_prior_count = 0;
    size_t anchored_count = 0;
    size_t pending_count = 0;

    size_t rejected_no_pose = 0;
    size_t rejected_bad_kf = 0;
    size_t rejected_limit = 0;

    double mean_confidence_weight = 0.0;
    double max_confidence_weight = 0.0;

    bool valid = false;

    rclcpp::Time created_time =
        rclcpp::Time(0, 0, RCL_ROS_TIME);

    rclcpp::Time last_update_time =
        rclcpp::Time(0, 0, RCL_ROS_TIME);

    std::string state = "BUILT_DRYRUN";
    std::string reason;
};



// ============================================================
// FASE 11R-C:
// Resultado de optimización local numérica dry-run.
// No modifica el mapa real.
// ============================================================

struct LocalOptimizationDryRunKeyFrameDelta
{
    uint64_t global_kf_id = 0;

    uint32_t drone_id = 0;
    uint64_t map_epoch = 0;
    uint64_t local_kf_id = 0;

    std::string side;
    std::string role;

    bool variable = false;
    bool fixed = false;
    bool excluded = false;

    double alpha = 0.0;

    // Alpha antes/después de aplicar el grafo de restricciones.
    double alpha_before_constraint_scale = 0.0;
    double constraint_alpha_scale = 1.0;

    double confidence_weight = 1.0;
    double pose_prior_weight = 1.0;

    size_t constraint_edges = 0;
    double constraint_strength = 0.0;
    double constraint_max_strength = 0.0;

    // FASE 12R-C:
    // Objetivo local heredado de la ventana.
    size_t local_opt_objective_edges = 0;
    double local_opt_objective_strength = 0.0;
    bool local_opt_objective_support = false;

    // ============================================================
    // FASE 12R-D3:
    // Selección distribuida de variables + propagación.
    // selected_variable=true significa que el KF entró como variable real
    // del solver.
    // propagated_delta=true significa que el KF no fue variable del solver,
    // pero recibió una corrección interpolada desde variables cercanas.
    // ============================================================

    bool pose_graph_selected_variable = false;
    bool pose_graph_propagated_delta = false;

    uint64_t pose_graph_representative_kf = 0;

    size_t pose_graph_represented_kfs = 0;

    double pose_graph_selection_score = 0.0;
    double pose_graph_represented_prior_bonus = 0.0;
    double pose_graph_propagation_weight_sum = 0.0;

    bool hard_fiducial_constraint = false;
    bool hard_fixed_by_constraint_graph = false;

    double delta_t_m = 0.0;
    double delta_yaw_deg = 0.0;
    double delta_rot_deg = 0.0;

    Eigen::Matrix4d dryrun_delta_T =
        Eigen::Matrix4d::Identity();

    std::string reason;
};

struct LocalOptimizationDryRunResult
{
    uint64_t result_id = 0;

    uint64_t task_id = 0;
    uint64_t window_id = 0;
    uint64_t source_event_id = 0;
    uint64_t pending_region_id = 0;

    bool valid = false;

    size_t inliers_used = 0;
    size_t skipped_matches = 0;

    double full_correction_t_m = 0.0;
    double full_correction_yaw_deg = 0.0;
    double full_correction_rot_deg = 0.0;

    double query_side_weight_sum = 0.0;
    double candidate_side_weight_sum = 0.0;

    double query_side_mobility_sum = 0.0;
    double candidate_side_mobility_sum = 0.0;

    double query_side_alpha = 0.0;
    double candidate_side_alpha = 0.0;

    size_t query_variable_kfs = 0;
    size_t candidate_variable_kfs = 0;
    size_t fixed_kfs = 0;
    size_t excluded_kfs = 0;

    // ============================================================
    // FASE 12R-D3:
    // Diagnóstico de selección/propagación.
    // ============================================================

    size_t pose_graph_variable_candidates = 0;
    size_t pose_graph_selected_variables = 0;
    size_t pose_graph_selected_query_variables = 0;
    size_t pose_graph_selected_candidate_variables = 0;

    size_t pose_graph_represented_kfs = 0;
    size_t pose_graph_propagated_kfs = 0;

    double pose_graph_represented_prior_bonus_sum = 0.0;
    double pose_graph_represented_prior_bonus_max = 0.0;

    double before_mean_residual_m =
        std::numeric_limits<double>::infinity();

    double after_mean_residual_m =
        std::numeric_limits<double>::infinity();

    double before_max_residual_m =
        std::numeric_limits<double>::infinity();

    double after_max_residual_m =
        std::numeric_limits<double>::infinity();

    double predicted_improvement_m = 0.0;
    double predicted_improvement_ratio = 0.0;

    double max_kf_delta_t_m = 0.0;
    double max_kf_delta_yaw_deg = 0.0;
    double max_kf_delta_rot_deg = 0.0;

    std::unordered_map<uint64_t, LocalOptimizationDryRunKeyFrameDelta>
        kf_deltas;

    rclcpp::Time created_time =
        rclcpp::Time(0, 0, RCL_ROS_TIME);

    std::string state = "DRYRUN_NOT_RUN";
    std::string reason;
};



// ============================================================
// FASE 12R-A:
// Resultado de aplicar al servidor una optimización local
// calculada previamente en dry-run.
// ============================================================

struct LocalOptimizationApplyStats
{
    uint64_t apply_id = 0;

    uint64_t task_id = 0;
    uint64_t dryrun_result_id = 0;
    uint64_t window_id = 0;
    uint64_t event_id = 0;
    uint64_t region_id = 0;

    size_t kfs_seen = 0;
    size_t kfs_moved = 0;
    size_t kfs_skipped = 0;
    size_t kfs_fixed = 0;
    size_t kfs_excluded = 0;
    size_t kfs_no_pose = 0;
    size_t kfs_not_finite = 0;

    size_t pending_kfs_cleared = 0;
    size_t pending_kfs_requeued = 0;
    size_t blocked_submaps_released = 0;
    size_t sibling_tasks_closed = 0;

    // FASE 12R-B:
    size_t kfs_skipped_hard_fiducial = 0;
    size_t kfs_skipped_constraint_graph = 0;
    size_t kfs_constraint_scaled = 0;

    // FASE 12R-B3:
    // Guards adicionales para evitar aplicar una optimización local
    // si la ventana no tiene soporte real del grafo.
    size_t apply_precheck_movable_kfs = 0;
    size_t apply_precheck_constrained_movable_kfs = 0;

    // FASE 12R-C:
    size_t apply_precheck_objective_movable_kfs = 0;
    size_t apply_precheck_supported_movable_kfs = 0;

    size_t apply_precheck_hard_fixed_kfs = 0;
    size_t apply_precheck_zero_constraint_kfs = 0;

    double apply_precheck_supported_ratio = 0.0;

    size_t objective_edges_registered = 0;

    size_t kfs_skipped_fiducial_direct_without_constraint = 0;

    bool apply_rejected_low_constraint_support = false;

    size_t constraint_edges_before = 0;
    size_t constraint_edges_after = 0;
    size_t constraint_edges_strengthened = 0;

    double max_constraint_strength_seen = 0.0;

    double max_delta_t_m = 0.0;
    double max_delta_yaw_deg = 0.0;
    double max_delta_rot_deg = 0.0;

    bool applied = false;

    rclcpp::Time created_time =
        rclcpp::Time(0, 0, RCL_ROS_TIME);

    std::string state = "NOT_APPLIED";
    std::string reason;
};



struct DirectLandmarkAssociationJob
{
    uint64_t query_global_kf_id = 0;
    uint64_t candidate_global_kf_id = 0;
    std::string reason;
};

struct InterDirectLandmarkAssociationJob
{
    uint64_t query_global_mp_id = 0;
    uint64_t candidate_global_mp_id = 0;

    uint64_t query_global_kf_id = 0;
    uint64_t candidate_global_kf_id = 0;

    double world_distance_m = 0.0;
    int descriptor_distance = 0;

    std::string reason;
};

struct KeyFramePairKey
{
    uint64_t a = 0;
    uint64_t b = 0;

    bool operator==(const KeyFramePairKey& other) const
    {
        return a == other.a && b == other.b;
    }
};

struct KeyFramePairKeyHash
{
    std::size_t operator()(const KeyFramePairKey& p) const
    {
        std::size_t seed = 0;
        seed ^= std::hash<uint64_t>{}(p.a) + 0x9e3779b97f4a7c15ULL + (seed << 6) + (seed >> 2);
        seed ^= std::hash<uint64_t>{}(p.b) + 0x9e3779b97f4a7c15ULL + (seed << 6) + (seed >> 2);
        return seed;
    }
};

struct SimulatedFiducial
{
    uint32_t id = 0;
    tf2::Transform world_T_fiducial;
    double detection_radius_m = 2.0;
};


struct ActiveFiducialWindow
{
    bool active = false;

    uint32_t drone_id = 0;
    uint64_t map_epoch = 0;
    uint32_t fiducial_id = 0;

    rclcpp::Time enter_time;
    rclcpp::Time last_seen_time;

    rclcpp::Time enter_stamp;
    rclcpp::Time last_seen_stamp;

    double min_distance_to_fiducial =
        std::numeric_limits<double>::max();

    tf2::Transform T_world_camera_last;
    tf2::Transform T_local_camera_last;

    tf2::Transform T_world_fiducial;
    double fiducial_radius = 2.0;

    uint64_t max_local_kf_id_at_enter = 0;

    tf2::Transform best_T_world_camera;
    tf2::Transform best_T_local_camera;
    bool has_best_observation = false;

    std::unordered_set<uint64_t> constrained_local_kfs;

    // Fase 8: diagnóstico de captura de fiducial.
    std::unordered_set<uint64_t> prior_local_kfs;
    std::unordered_set<uint64_t> good_prior_local_kfs;

    // ============================================================
    // FASE 12R-D2:
    // KFs que han entrado en fiducial pero con error alto contra la
    // pose real. No deben entrar como prior fuerte ni disparar el
    // optimizador legacy. Son deuda para optimización global futura.
    // ============================================================
    std::unordered_set<uint64_t> debt_prior_local_kfs;

    size_t good_prior_count = 0;
    size_t total_prior_count = 0;
    size_t debt_prior_count = 0;
    

    bool local_warp_applied = false;
    size_t local_warp_moved_kfs = 0;

    bool graph_optimized_after_priors = false;

    rclcpp::Time first_ready_time;
    bool has_been_ready = false;

    bool initial_anchor_created = false;

    double last_optimization_final_chi2 =
        std::numeric_limits<double>::infinity();

    double last_optimization_chi2_per_edge =
        std::numeric_limits<double>::infinity();

    bool last_optimization_low_chi =
        false;

    // FASE 4.3:
    // Diagnóstico/protección contra reintentos de optimizaciones malas.
    bool last_optimization_guard_rejected = false;

    int optimization_guard_reject_count = 0;

    rclcpp::Time last_optimization_guard_reject_time;

    std::string last_optimization_guard_reject_reason;

    double last_optimization_guard_reject_chi2 =
        std::numeric_limits<double>::infinity();

    double last_optimization_guard_reject_chi2_per_edge =
        std::numeric_limits<double>::infinity();
};



struct OptimizedSubmapTransform
{
    uint32_t drone_id = 0;
    uint64_t map_epoch = 0;

    double stamp = -std::numeric_limits<double>::infinity();

    Eigen::Matrix4d world_T_local =
        Eigen::Matrix4d::Identity();

    bool valid = false;
};



struct InterDroneFusionDiag
{
    uint32_t drone_a = 0;
    uint32_t drone_b = 0;

    uint64_t epoch_a = 0;
    uint64_t epoch_b = 0;

    bool anchor_a = false;
    bool anchor_b = false;

    bool confirmed_a = false;
    bool confirmed_b = false;

    size_t kfs_a = 0;
    size_t kfs_b = 0;

    size_t mps_a = 0;
    size_t mps_b = 0;

    size_t good_mps_a = 0;
    size_t good_mps_b = 0;

    size_t rejected_quality_a = 0;
    size_t rejected_quality_b = 0;

    size_t candidate_kf_pairs = 0;

    size_t mp_pairs_checked = 0;
    size_t mp_pairs_world_near = 0;
    size_t mp_pairs_descriptor_near = 0;

    size_t rejected_no_world = 0;
    size_t rejected_distance = 0;
    size_t rejected_descriptor = 0;
    size_t rejected_quality = 0;
    size_t rejected_same_source = 0;

    size_t accepted_potential = 0;

    bool max_pair_checks_reached = false;

    double min_world_distance_m =
        std::numeric_limits<double>::infinity();

    double mean_world_distance_m = 0.0;

    double min_descriptor_distance =
        std::numeric_limits<double>::infinity();

    double mean_descriptor_distance = 0.0;
};



class GlobalMapServer : public rclcpp::Node
{
public:
    GlobalMapServer();
    ~GlobalMapServer() override;

private:
    using OrbMap = orbslam3_msgs::msg::OrbMap;
    using FiducialLockStatus = orbslam3_msgs::msg::FiducialLockStatus;
    using GetOrbMap = orbslam3_msgs::srv::GetOrbMap;

    void OrbMapDeltaCallback(const OrbMap::SharedPtr msg);
    void OrbPoseCallback(uint32_t drone_id, const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void GtPoseCallback(uint32_t drone_id, const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    void FullSnapshotTimerCallback();
    void PublishTimerCallback();
    void BowTimerCallback();
    void FiducialTimerCallback();

    void FiducialLockStatusTimerCallback();
    void RunDiagnosticsTimerCallback();

    // ============================================================
    // FASE 0:
    // Auditoría estructurada del estado actual del servidor.
    // No cambia lógica; solo imprime resumen por tick.
    // ============================================================

    void LogPhase0SubmapAudit(
        const char* context);

    void LogPhase0FusedLegacyAudit(
        const char* context,
        bool cache_reused,
        size_t published_points,
        double cache_age_s);

    const char* AnchorSourceToString(
        orbslam3_multi::AnchorSource source) const;

    const char* AnchorMaturityToString(
        orbslam3_multi::AnchorMaturity maturity) const;

    void DiagnoseConfirmedInterDroneFusion();

    int DescriptorHammingDistance(
        const std::array<uint8_t, 32>& a,
        const std::array<uint8_t, 32>& b) const;

    bool IsValidOrbDescriptor(
        const std::array<uint8_t, 32>& descriptor) const;

    FiducialLockStatus BuildFiducialLockStatus(
        uint32_t drone_id,
        const DroneMapData& data,
        const SimulatedFiducial& fiducial,
        double distance_to_fiducial,
        const ActiveFiducialWindow* window) const;

    bool EvaluateFiducialReadyToLeave(
        const DroneMapData& data,
        const ActiveFiducialWindow* window,
        double distance_to_fiducial,
        const SimulatedFiducial& fiducial,
        std::string& state_out,
        std::string& reason_out) const;

    uint64_t MakeGlobalKeyFrameId(
        uint32_t drone_id,
        uint64_t map_epoch,
        uint64_t local_kf_id) const;

    size_t CountOptimizedConstrainedKfsNearFiducial(
        const ActiveFiducialWindow& window,
        const SimulatedFiducial& fiducial) const;

    void PublishFiducialLockStatus(
        const FiducialLockStatus& status);

    void ApplyOrbMapUpdate(const OrbMap& msg, bool full_snapshot);

    void InsertOrUpdateMapPointInMulti(
        uint32_t drone_id,
        uint64_t map_epoch,
        const orbslam3_msgs::msg::OrbMapPoint& mp_msg);

    void InsertOrUpdateKeyFrameInMultiAndQueueBow(
        uint32_t drone_id,
        uint64_t map_epoch,
        const orbslam3_msgs::msg::OrbKeyFrame& kf_msg);

    orbslam3_multi::ImportedMapPoint ConvertMapPointMsg(
        uint32_t drone_id,
        uint64_t map_epoch,
        const orbslam3_msgs::msg::OrbMapPoint& msg) const;

    orbslam3_multi::ImportedKeyFrame ConvertKeyFrameMsg(
        uint32_t drone_id,
        uint64_t map_epoch,
        const orbslam3_msgs::msg::OrbKeyFrame& msg) const;
    
    std::deque<uint64_t> pending_bow_query_kfs_;
    std::unordered_set<uint64_t> pending_bow_query_kfs_set_;

    int max_bow_query_keyframes_per_tick_ = 3;
    int max_pending_bow_query_keyframes_ = 2000;

    void EnqueueBowQueryForKeyFrame(
        const orbslam3_multi::ImportedKeyFrame& kf);

    void ProcessPendingBowQueries();

    void QueueBowCandidatesForKeyFrame(const orbslam3_multi::ImportedKeyFrame& kf);
    void ProcessBowCandidateQueue();

    int SearchMatchesByBoW(
        uint64_t query_global_kf_id,
        uint64_t candidate_global_kf_id,
        std::vector<orbslam3_multi::FeatureMatch>& matches_out);

    std::vector<orbslam3_multi::BowCandidate> ApplyLoopCandidateFrustumGating(
        const orbslam3_multi::ImportedKeyFrame& query_kf,
        const std::vector<orbslam3_multi::BowCandidate>& candidates,
        std::unordered_map<uint64_t, size_t>& visible_counts_out,
        bool& gating_active_out,
        bool& fallback_used_out,
        std::string& reason_out) const;

    size_t CountVisibleMapPointsForCandidateKeyFrameFromQueryFrustum(
        uint64_t query_global_kf_id,
        uint64_t candidate_global_kf_id,
        size_t& checked_mappoints_out,
        std::string& reason_out) const;

    bool ProjectWorldPointIntoCameraFrustum(
        const Eigen::Vector3d& world_p,
        const Eigen::Matrix4d& world_T_camera,
        double fx,
        double fy,
        double cx,
        double cy,
        uint32_t image_width,
        uint32_t image_height,
        double depth_min_m,
        double depth_max_m,
        double margin_px,
        double& u_out,
        double& v_out,
        double& depth_out) const;

    std::string UnifiedLoopRelationType(
        uint32_t query_drone,
        uint64_t query_epoch,
        uint32_t candidate_drone,
        uint64_t candidate_epoch) const;

    bool ComputeUnifiedLoopGraphError(
        uint64_t query_global_kf_id,
        uint64_t candidate_global_kf_id,
        const Eigen::Matrix4d& measured_candidate_T_query,
        double& error_t_m_out,
        double& error_yaw_deg_out,
        double& error_rot_deg_out,
        std::string& reason_out) const;

    bool ComputeLoopViewMapErrorDetailed(
        uint64_t query_global_kf_id,
        uint64_t candidate_global_kf_id,
        const Eigen::Matrix4d& measured_candidate_T_query,
        LoopViewMapErrorInfo& info_out) const;

    void LogLoopViewMapError(
        const std::string& context,
        uint64_t event_id,
        const std::string& relation_type,
        uint32_t query_drone,
        uint64_t query_epoch,
        uint64_t query_local_kf,
        uint32_t candidate_drone,
        uint64_t candidate_epoch,
        uint64_t candidate_local_kf,
        const LoopViewMapErrorInfo& info,
        bool usable_for_optimization,
        bool usable_for_fusion,
        bool usable_for_anchor) const;

    bool GetWorldPoseForLoopKeyFrame(
        uint64_t global_kf_id,
        Eigen::Matrix4d& world_T_camera_out,
        std::string& reason_out) const;

    void RegisterUnifiedLoopRuntimeEvent(
        const UnifiedLoopRuntimeEvent& event);

    bool CreateSubcloudLocalLoopEventFromUnifiedLoop(
        const UnifiedLoopRuntimeEvent& event,
        LoopSubcloudEvent& subcloud_event_out,
        std::string& reason_out) const;

    void EvaluateUnifiedLoopEventsAfterOptimization(
        const std::string& context);

    bool RequestLoopOptimizationRetryFromEvents(
        const std::string& context);

    std::string LoopOptimizationEventStatus(
        const UnifiedLoopRuntimeEvent& event) const;

    bool ComputeFusedWorldPointForMapPoint(
        uint32_t drone_id,
        uint64_t map_epoch,
        const orbslam3_msgs::msg::OrbMapPoint& mp_msg,
        const HistoricalSubmapMirror& hist,
        const Eigen::Matrix4d& fallback_world_T_local,
        Eigen::Vector3d& world_point_out,
        std::string& source_out) const;

    bool ComputeWorldPointFromOptimizedKeyFrameObservation(
        const orbslam3_msgs::msg::OrbMapPoint& mp_msg,
        const orbslam3_msgs::msg::OrbKeyFrame& kf_msg,
        uint64_t global_kf_id,
        Eigen::Vector3d& world_point_out,
        std::string& reason_out) const;
        
    bool GetWorldPointForGlobalMapPoint(
        uint64_t global_mappoint_id,
        Eigen::Vector3d& world_point_out,
        std::array<uint8_t, 32>& descriptor_out,
        uint32_t& observations_out,
        double& found_ratio_out,
        bool& from_confirmed_anchor_out,
        bool& from_provisional_anchor_out,
        std::string& reason_out) const;

    void AddLoopFusionObservationsFromReadyEvents(
        std::vector<FusedTrackObservation>& observations,
        const rclcpp::Time& stamp,
        LoopFusionStats& stats);

    bool BuildLoopFusionObservationFromMapPointPair(
        const UnifiedLoopRuntimeEvent& event,
        uint64_t query_global_mp_id,
        uint64_t candidate_global_mp_id,
        FusedTrackObservation& obs_out,
        double& pair_distance_out,
        int& pair_hamming_out,
        std::string& reject_reason_out) const;

    // ============================================================
    // FASE 10R-A:
    // Fusión real conservadora desde LoopSubcloudEvent decision=FUSION_EVENT.
    // No usa loops legacy ni ancla ni optimiza.
    // ============================================================

    void AddSubcloudFusionObservationsFromDecisionEvents(
        std::vector<FusedTrackObservation>& observations,
        const rclcpp::Time& stamp,
        LoopFusionStats& stats);

    bool BuildSubcloudFusionObservationFromMatch(
        const LoopSubcloudEvent& event,
        const LoopPointMatch& match,
        FusedTrackObservation& obs_out,
        double& pair_distance_out,
        int& pair_hamming_out,
        std::string& reject_reason_out) const;

    void RegisterSubcloudMapPointEvidenceFromEvent(
        const LoopSubcloudEvent& event,
        const std::vector<LoopPointMatch>& accepted_matches,
        const rclcpp::Time& stamp);

    void ApplySubcloudMapPointEvidenceToObservations(
        std::vector<FusedTrackObservation>& observations,
        const rclcpp::Time& stamp);

    void RegisterSubcloudKeyFrameCovisibilityFromAcceptedMatches(
        const LoopSubcloudEvent& event,
        const std::vector<LoopPointMatch>& accepted_matches,
        const rclcpp::Time& stamp);

    // ============================================================
    // FASE 11R-A:
    // Optimización local pendiente y bloqueo de fusiones.
    // ============================================================

    bool SubcloudLocalLoopTaskAlreadyExists(
        uint64_t source_event_id) const;

    bool HasPendingSubcloudLocalLoopOptimizationForDrone(
        uint32_t drone_id,
        std::string& reason_out) const;

    bool IsSubmapOptimizationPending(
        uint32_t drone_id,
        uint64_t map_epoch,
        std::string& reason_out) const;

    bool DoesEventTouchOptimizationPendingSubmap(
        const LoopSubcloudEvent& event,
        std::string& reason_out) const;

    void CollectAffectedSubmapsFromLoopEvent(
        const LoopSubcloudEvent& event,
        std::set<uint64_t>& affected_submap_keys_out,
        std::string& reason_out) const;

    uint64_t RegisterPendingOptimizationRegionFromEvent(
        const LoopSubcloudEvent& event,
        uint64_t task_id,
        const rclcpp::Time& stamp,
        const std::string& context);

    void MarkKeyFramePendingAfterOptimizationStart(
        uint64_t global_kf_id,
        uint32_t drone_id,
        uint64_t map_epoch,
        uint64_t local_kf_id,
        const std::string& reason);

    void EnqueueSubcloudLocalLoopOptimizationTasksFromEvents(
        const rclcpp::Time& stamp,
        const std::string& context);

    // ============================================================
    // FASE 11R-B:
    // Construcción dry-run de ventana local de optimización.
    // ============================================================

    bool GetKeyFrameWorldPositionForLocalOpt(
        uint64_t global_kf_id,
        Eigen::Vector3d& position_world_out,
        std::string& reason_out) const;

    bool LocalOptKeyFrameHasFiducialPrior(
        uint64_t global_kf_id) const;

    size_t CountLocalOptCovisibilitySupport(
        uint64_t global_kf_id) const;

    // ============================================================
    // FASE 12R-B:
    // Grafo persistente de restricciones KF-KF.
    // ============================================================

    KeyFramePairKey MakeOrderedKeyFramePairKey(
        uint64_t kf_a,
        uint64_t kf_b) const;

    bool GetRelativeTransformBetweenKeyFramesForConstraint(
        uint64_t kf_a,
        uint64_t kf_b,
        Eigen::Matrix4d& a_T_b_out,
        std::string& reason_out) const;

    LocalOptKeyFrameConstraintSummary BuildLocalOptConstraintSummaryForKeyFrame(
        uint64_t global_kf_id) const;

    double ComputeLocalOptConstraintAlphaScale(
        const LocalOptKeyFrameConstraintSummary& summary) const;

    bool IsLocalOptHardFixedByConstraintGraph(
        uint64_t global_kf_id,
        std::string& reason_out) const;

    void UpsertLocalOptKeyFrameConstraintEdge(
        uint64_t kf_a,
        uint64_t kf_b,
        const Eigen::Matrix4d& measured_a_T_b,
        bool absolute_prior,
        bool hard_fixed_prior,
        const Eigen::Matrix4d& measured_world_T_a,
        double strength_gain,
        double translation_weight_gain,
        double rotation_weight_gain,
        size_t support_pairs,
        size_t support_inliers,
        double mean_residual,
        double max_residual,
        uint64_t source_event_id,
        const std::unordered_set<uint64_t>& source_mappoint_ids,
        const std::string& source_type,
        const std::string& source_detail,
        const rclcpp::Time& stamp,
        const std::string& reason);

    void RememberLocalOptFiducialPriorKeyFrame(
        uint32_t drone_id,
        uint64_t map_epoch,
        uint64_t local_kf_id,
        const Eigen::Matrix4d& measured_world_T_kf,
        double fiducial_weight,
        const std::string& fiducial_classification,
        bool hard_fixed_prior,
        bool global_optimization_debt,
        double fiducial_error_t_m,
        double fiducial_error_yaw_deg,
        double fiducial_error_rot_deg,
        bool has_current_world_pose,
        const std::string& current_pose_source,
        const rclcpp::Time& stamp,
        const std::string& reason);

    size_t RegisterLocalOptConstraintEdgesFromLoopEvent(
        const LoopSubcloudEvent& event,
        const std::string& source_type,
        double strength_multiplier,
        const rclcpp::Time& stamp,
        const std::string& reason);

    size_t RegisterLocalOptObjectiveConstraintEdgesFromEvent(
        const LoopSubcloudEvent& event,
        const rclcpp::Time& stamp,
        const std::string& reason);

    size_t RegisterLocalOptConstraintEdgesAfterApply(
        const OptimizationTask& task,
        const LocalOptimizationDryRunResult& result,
        const LocalOptimizationApplyStats& stats,
        const rclcpp::Time& stamp,
        const std::string& reason);

    void LogLocalOptConstraintGraphSummary(
        const std::string& context) const;

    void AccumulateLocalOptKeyFrameSupportFromMapPoint(
        uint64_t global_mappoint_id,
        size_t support_increment,
        std::unordered_map<uint64_t, size_t>& kf_support_out,
        size_t& ref_kf_support_out,
        size_t& obs_kf_support_out,
        size_t& scan_kf_support_out,
        size_t& missing_mp_out) const;

    void AccumulateLocalOptCandidateKeyFrameSupportFromMatch(
        const LoopPointMatch& match,
        std::unordered_map<uint64_t, size_t>& candidate_kf_support_out,
        size_t& direct_kf_support_out,
        size_t& mp_ref_kf_support_out,
        size_t& mp_obs_kf_support_out,
        size_t& mp_scan_kf_support_out,
        size_t& fused_track_source_mp_support_out,
        size_t& missing_mp_out) const;

    size_t AddNearestCandidateKeyFramesFromAffectedSubmapsToLocalOptWindow(
        LocalOptimizationWindow& window,
        const SubcloudPendingOptimizationRegion* region,
        uint32_t query_drone,
        uint64_t query_epoch,
        int max_to_add,
        const std::string& reason) const;

    double ComputeLocalOptKeyFrameConfidenceWeight(
        const LocalOptimizationWindowKeyFrame& kf) const;

    void UpsertLocalOptimizationWindowKeyFrame(
        LocalOptimizationWindow& window,
        uint64_t global_kf_id,
        const std::string& side,
        const std::string& role,
        bool variable,
        bool fixed,
        bool excluded,
        size_t inlier_support,
        const std::string& reason) const;

    void AddSameSubmapKeyFramesToLocalOptWindow(
        LocalOptimizationWindow& window,
        uint64_t seed_global_kf_id,
        const std::string& side,
        const std::string& role,
        double radius_m,
        int temporal_span,
        int max_to_add,
        bool variable,
        bool fixed,
        const std::string& reason) const;

    void AddCovisibilityNeighborsToLocalOptWindow(
        LocalOptimizationWindow& window,
        int max_to_add,
        const std::string& reason) const;

    void AddBoundaryKeyFramesToLocalOptWindow(
        LocalOptimizationWindow& window,
        const SubcloudPendingOptimizationRegion* region,
        double boundary_radius_m,
        int max_to_add,
        const std::string& reason) const;

    void AddPendingAfterOptStartKeyFramesToLocalOptWindow(
        LocalOptimizationWindow& window,
        const SubcloudPendingOptimizationRegion* region,
        const std::string& reason) const;

    bool BuildLocalOptimizationWindowFromTask(
        const OptimizationTask& task,
        LocalOptimizationWindow& window_out,
        std::string& reason_out) const;

    void LogLocalOptimizationWindow(
        const LocalOptimizationWindow& window,
        const std::string& context) const;

    void BuildAndLogSubcloudLocalOptimizationWindowsFromPendingTasks(
        const rclcpp::Time& stamp,
        const std::string& context);

    // ============================================================
    // FASE 11R-C:
    // Optimización numérica dry-run sobre copia.
    // ============================================================

    Eigen::Matrix4d BuildPartialLocalOptCorrection(
        const Eigen::Matrix4d& full_T,
        double alpha) const;

    Eigen::Vector3d TransformPointLocalOpt(
        const Eigen::Matrix4d& T,
        const Eigen::Vector3d& p) const;

    void ComputeLocalOptDryRunResiduals(
        const LoopSubcloudEvent& event,
        const Eigen::Matrix4d& query_delta_T,
        const Eigen::Matrix4d& candidate_delta_T,
        double& before_mean_out,
        double& before_max_out,
        double& after_mean_out,
        double& after_max_out,
        size_t& inliers_used_out,
        size_t& skipped_out) const;

    bool RunLocalOptimizationDryRunForWindow(
        const OptimizationTask& task,
        const LocalOptimizationWindow& window,
        LocalOptimizationDryRunResult& result_out,
        std::string& reason_out) const;

    void LogLocalOptimizationDryRunResult(
        const LocalOptimizationDryRunResult& result,
        const std::string& context) const;

    void RunSubcloudLocalOptimizationDryRunsFromWindows(
        const rclcpp::Time& stamp,
        const std::string& context);

    // ============================================================
    // FASE 12R-A:
    // Aplicación real al servidor de dry-run local validado.
    // ============================================================

    bool GetKeyFrameWorldPoseForLocalOptApply(
        uint64_t global_kf_id,
        Eigen::Matrix4d& world_T_kf_out,
        std::string& reason_out) const;

    void RequeuePendingKeyFramesAfterLocalOptApply(
        const SubcloudPendingOptimizationRegion& region,
        LocalOptimizationApplyStats& stats);

    void FinishPendingOptimizationRegionAfterLocalOptApply(
        uint64_t region_id,
        uint64_t applied_task_id,
        const rclcpp::Time& stamp,
        LocalOptimizationApplyStats& stats);

    bool ApplyLocalOptimizationDryRunResultToServer(
        OptimizationTask& task,
        const LocalOptimizationDryRunResult& result,
        const rclcpp::Time& stamp,
        const std::string& context,
        LocalOptimizationApplyStats& stats_out,
        std::string& reason_out);

    void LogLocalOptimizationApplyStats(
        const LocalOptimizationApplyStats& stats,
        const std::string& context) const;

    void ApplySubcloudLocalOptimizationDryRunResults(
        const rclcpp::Time& stamp,
        const std::string& context);

    void MarkSubcloudFusionRegionValidated(
        const LoopSubcloudEvent& event,
        const std::vector<Eigen::Vector3d>& accepted_positions,
        size_t accepted_pairs,
        const rclcpp::Time& stamp);

    std::string DecideUnifiedLoopDryRunAction(
        bool has_graph_error,
        bool usable_for_optimization,
        bool usable_for_fusion,
        double error_t_m,
        double error_yaw_deg,
        std::string& reason_out) const;

    double RotationAngleDeg(
        const Eigen::Matrix3d& R) const;

    bool IsValidBowCandidate(uint64_t query_global_kf_id, uint64_t candidate_global_kf_id) const;
    void RememberTestedPair(uint64_t a, uint64_t b);

    void UpdateGlobalKeyFramesLite();
    void UpdateGlobalSparseMap();
    void PublishGlobalSparseCloud();
    sensor_msgs::msg::PointCloud2 BuildPointCloud(const std::vector<GlobalSparsePoint>& points) const;

    // ============================================================
    // FASE 1:
    // Ruta nueva simple para /global_sparse_map_fused.
    //
    // Objetivo:
    // - No depender de confirmed-only/min_sources de la ruta legacy.
    // - Publicar puntos fuertes de submapas anclados/publicables.
    // - Fusionar duplicados cercanos con spatial hash.
    // - Mantener coste bajo y logs resumen.
    // ============================================================

    struct SimpleFusedCandidate
    {
        uint32_t drone_id = 0;
        uint64_t map_epoch = 0;
        uint64_t global_mappoint_id = 0;

        Eigen::Vector3d world_pos =
            Eigen::Vector3d::Zero();

        std::array<uint8_t, 32> descriptor{};

        uint32_t observations = 0;
        double found_ratio = 0.0;

        bool from_provisional_anchor = false;
        bool from_confirmed_anchor = false;

        double weight = 1.0;
    };

    struct SimpleFusedGroup
    {
        std::vector<size_t> candidate_indices;

        Eigen::Vector3d weighted_pos_sum =
            Eigen::Vector3d::Zero();

        double weight_sum = 0.0;

        std::array<uint8_t, 32> representative_descriptor{};

        size_t representative_candidate_index = 0;
        double representative_weight = -1.0;

        bool has_provisional_source = false;
        bool has_confirmed_source = false;
    };

    void PublishFusedLandmarkCloudSimple();

    bool GetWorldTransformForFusedSubmap(
        uint32_t drone_id,
        uint64_t map_epoch,
        Eigen::Matrix4d& world_T_local_out,
        std::string& source_out,
        std::string& maturity_out,
        bool& from_provisional_anchor_out,
        bool& from_confirmed_anchor_out,
        std::string& reason_out) const;

    double ComputeSimpleFusedCandidateWeight(
        const orbslam3_msgs::msg::OrbMapPoint& mp_msg,
        bool from_provisional_anchor,
        bool from_confirmed_anchor) const;

    double ComputeProposedSimpleGroupSpread(
        const SimpleFusedGroup& group,
        const std::vector<SimpleFusedCandidate>& candidates,
        const SimpleFusedCandidate& new_candidate) const;

    void GetSimpleMergePolicyForGroup(
        const SimpleFusedGroup& group,
        const std::vector<SimpleFusedCandidate>& candidates,
        const SimpleFusedCandidate& new_candidate,
        double& merge_radius_out,
        int& max_hamming_out,
        double& max_group_spread_out,
        std::string& policy_out) const;

    std::array<uint8_t, 32> ComputeSimpleGroupMedoidDescriptor(
        const SimpleFusedGroup& group,
        const std::vector<SimpleFusedCandidate>& candidates) const;

    double ClampScore01(
        double v) const;

    double ComputeFusedTrackObservationWeight(
        const FusedTrackObservation& obs) const;

    double ComputeFusedTrackInitialScore(
        const FusedTrackObservation& obs) const;

    uint64_t FindBestFusedTrackForObservation(
        const FusedTrackObservation& obs,
        double& best_distance_out,
        int& best_hamming_out) const;

    void UpdateFusedTrackWithObservation(
        FusedLandmarkTrack& track,
        const FusedTrackObservation& obs,
        const rclcpp::Time& stamp,
        bool* geometry_frozen_out = nullptr);

    std::array<uint8_t, 32> ComputeFusedTrackMedoidDescriptor(
        const FusedLandmarkTrack& track) const;

    std::vector<GlobalSparsePoint> UpdateFusedTracksAndBuildCloud(
        const std::vector<FusedTrackObservation>& observations,
        const rclcpp::Time& stamp,
        FusedTrackUpdateStats& stats);

    bool ShouldMergeFusedTracks(
        const FusedLandmarkTrack& a,
        const FusedLandmarkTrack& b,
        double& distance_out,
        int& hamming_out,
        bool& is_inter_out,
        bool& is_cross_epoch_out) const;

    void MergeFusedTrackInto(
        FusedLandmarkTrack& keep,
        const FusedLandmarkTrack& remove,
        const rclcpp::Time& stamp);

    void ConsolidateFusedTracks(
        const rclcpp::Time& stamp,
        FusedTrackUpdateStats& stats);

    void UpdateRecentCorrectedOrbTrajectoryFromLocalPose(
        uint32_t drone_id,
        const geometry_msgs::msg::PoseStamped& local_pose_msg,
        const rclcpp::Time& receive_time);

    bool IsPointNearRecentCorrectedOrbTrajectory(
        uint32_t source_drone_id,
        const Eigen::Vector3d& p_world,
        double& best_xy_distance_out,
        double& best_z_distance_out) const;

    bool IsLowQualityCandidateForTrajectoryFilter(
        const SimpleFusedCandidate& candidate) const;

    bool IsLowQualityCandidateForFreeSpaceFilter(
        const SimpleFusedCandidate& candidate) const;

    bool IsPointInsideRecentCorrectedOrbFreeSpaceTube(
        uint32_t source_drone_id,
        const Eigen::Vector3d& p_world,
        double& best_depth_out,
        double& best_lateral_distance_out) const;

    bool ProjectWorldPointIntoKeyFrame(
        const Eigen::Vector3d& world_p,
        const DroneMapData& data,
        const orbslam3_msgs::msg::OrbKeyFrame& kf_msg,
        const Eigen::Matrix4d& world_T_local,
        double& u_out,
        double& v_out,
        double& depth_out) const;

    size_t CountExpectedVisibleKeyFramesForTrack(
        const FusedLandmarkTrack& track,
        size_t& checked_kfs_out,
        size_t& skipped_no_camera_out,
        size_t& skipped_no_correction_out) const;

    std::vector<GlobalSparsePoint> ApplySimpleVoxelFilter(
        const std::vector<GlobalSparsePoint>& input,
        double voxel_size_m,
        size_t max_points) const;

    void PublishFusedDebugCloud(
        const std::vector<GlobalSparsePoint>& all_valid_points,
        const std::vector<GlobalSparsePoint>& confirmed_candidate_points,
        const std::vector<GlobalSparsePoint>& pre_voxel_points,
        size_t total_landmarks,
        size_t valid_landmarks,
        size_t merged_groups,
        size_t singleton_groups,
        size_t stable_kept,
        size_t density_kept_merged,
        size_t density_kept_singletons,
        size_t final_published_points);

    struct ExpectedViewStats
    {
        uint32_t expected_views = 0;
        uint32_t seen_expected_views = 0;
        uint32_t missed_expected_views = 0;

        uint32_t tested_keyframes = 0;
        uint32_t projected_keyframes = 0;
        uint32_t skipped_no_camera = 0;
        uint32_t skipped_no_pose = 0;
        uint32_t skipped_out_of_radius = 0;
        uint32_t skipped_behind_or_depth = 0;
        uint32_t skipped_outside_image = 0;

        double score_before = 0.0;
        double score_after = 0.0;
    };

    ExpectedViewStats ComputeExpectedViewStatsForLandmark(
        const orbslam3_multi::FusedLandmark& lm,
        double score_before) const;

    bool TryGetWorldTCameraForKeyFrame(
        uint32_t drone_id,
        uint64_t map_epoch,
        const DroneMapData& data,
        const orbslam3_msgs::msg::OrbKeyFrame& kf_msg,
        tf2::Transform& T_world_camera_out,
        bool& used_optimized_pose_out) const;

    bool KeyFrameObservesAnySourceMapPoint(
        uint32_t drone_id,
        uint64_t map_epoch,
        const orbslam3_msgs::msg::OrbKeyFrame& kf_msg,
        const std::unordered_set<uint64_t>& source_mappoint_ids) const;

    tf2::Transform EigenMatrix4dToTf2(
        const Eigen::Matrix4d& T) const;

    void PublishCorrections();
    void BroadcastTfCorrections();

    bool CanRequestFullSnapshot(uint32_t drone_id) const;
    void RequestFullSnapshot(uint32_t drone_id, const std::string& reason);
    bool ShouldRequestFullSnapshotAfterDelta(uint32_t drone_id, const OrbMap& msg, std::string& reason_out);

    void LoadSimulatedFiducialsFromParameters();
    bool TryInitializeCorrection(uint32_t drone_id);
    bool ProcessSimulatedFiducialObservation(uint32_t drone_id, DroneMapData& data, const SimulatedFiducial& fiducial, double distance_to_fiducial);
    bool FindBestNearbyFiducial(const tf2::Transform& T_world_body, SimulatedFiducial& best_fiducial_out, double& best_distance_out) const;

    uint64_t MakeGlobalId(uint32_t drone_id, uint64_t map_epoch, uint64_t local_id) const;
    uint32_t ExtractDroneId(uint64_t global_id) const;
    uint64_t ExtractMapEpoch(uint64_t global_id) const;
    uint64_t ExtractLocalId(uint64_t global_id) const;

    tf2::Transform PoseMsgToTf2(const geometry_msgs::msg::Pose& pose) const;
    tf2::Transform TransformMsgToTf2(const geometry_msgs::msg::Transform& transform) const;
    geometry_msgs::msg::Pose TransformToPoseMsg(const tf2::Transform& tf) const;
    geometry_msgs::msg::Transform Tf2ToTransformMsg(const tf2::Transform& tf) const;

    bool GetLocalKeyFramePose(
        const DroneMapData& data,
        uint64_t local_kf_id,
        tf2::Transform& T_local_kf_out) const;

    std::string Tf2TranslationToString(
        const tf2::Transform& T) const;
    

    tf2::Transform GetBodyCameraTransform() const;

    bool IsFinitePose(const geometry_msgs::msg::Pose& pose) const;
    bool IsFiniteTransform(const geometry_msgs::msg::Transform& tf) const;

    bool IsFiniteTransform(
        const tf2::Transform& tf) const;

    std::string world_frame_ = "world";
    std::string namespace_base_ = "dron";
    int n_drones_ = 1;

    double full_snapshot_period_s_ = 300.0;
    double publish_period_s_ = 0.5;
    double bow_timer_period_s_ = 0.5;
    double fiducial_check_period_s_ = 0.2;
    double fiducial_prior_max_kf_distance_m_ = 0.75;
    double fiducial_prior_min_interval_same_fiducial_s_ = 1.0;

    double min_time_between_forced_snapshots_s_ = 5.0;
    uint32_t max_empty_delta_count_ = 20;
    bool request_snapshot_on_sequence_jump_ = true;
    bool request_snapshot_on_empty_deltas_ = true;

    bool enable_bow_candidate_detection_ = true;
    bool enable_intra_drone_loop_detection_ = true;
    bool enable_inter_drone_loop_detection_ = true;
    int min_intra_drone_loop_kf_gap_ = 20;

    int max_bow_candidates_per_keyframe_ = 10;
    int max_bow_queue_size_ = 500;
    int max_bow_jobs_per_tick_ = 20;
    int min_common_words_ = 8;
    double min_bow_score_ = 0.015;

    int max_hamming_distance_ = 60;
    double bow_ratio_test_ = 0.75;
    int min_bow_matches_ = 20;

    // ============================================================
    // FASE 3B:
    // Asistente de encuentro inter-dron.
    //
    // No crea anchors artificiales.
    // Solo reencola KeyFrames cuando un dron anclado y otro flotante
    // están cerca según GT, para aumentar la probabilidad de loop BoW.
    // ============================================================

    bool enable_inter_drone_anchor_assist_ = true;

    double inter_drone_anchor_assist_distance_m_ = 1.50;
    double inter_drone_anchor_assist_period_s_ = 0.75;

    int inter_drone_anchor_assist_recent_kfs_ = 12;
    int inter_drone_anchor_assist_min_kfs_per_drone_ = 1;

    // ============================================================
    // Fase 7:
    // Pares explícitos inter-dron para anclaje por loop.
    // Cuando un dron anclado y otro flotante están cerca, además de
    // reencolar queries BoW, generamos pares explícitos recent-floating
    // contra recent-anchored.
    // ============================================================

    bool inter_drone_anchor_assist_enqueue_explicit_pairs_ = true;

    int inter_drone_anchor_assist_explicit_recent_anchored_kfs_ = 12;
    int inter_drone_anchor_assist_explicit_recent_floating_kfs_ = 12;

    int inter_drone_anchor_assist_max_explicit_pairs_per_tick_ = 80;

    // Si true, borra tested/queued entre drones justo antes de meter pares explícitos.
    // Útil en simulación porque un par pudo fallar cuando aún no había suficientes MPs.
    bool inter_drone_anchor_assist_clear_pairs_before_explicit_ = true;

    // Logs de muestra de pares explícitos.
    bool inter_drone_anchor_assist_log_explicit_pairs_ = true;
    int inter_drone_anchor_assist_max_explicit_pair_logs_ = 30;

    // Diagnóstico de Fase 7.
    bool phase7_log_anchor_state_ = true;

    bool inter_drone_anchor_assist_clear_tested_pairs_ = true;

    // FASE 3C:
    // Durante unos segundos después de detectar un encuentro útil,
    // procesamos más queries/candidatos BoW por tick para reducir
    // la latencia hasta LOOP-ANCHOR-ACCEPT.
    double inter_drone_anchor_assist_boost_duration_s_ = 3.0;

    int inter_drone_anchor_assist_max_bow_queries_per_tick_ = 12;
    int inter_drone_anchor_assist_max_bow_jobs_per_tick_ = 60;

    rclcpp::Time inter_drone_anchor_assist_boost_until_;

    std::unordered_map<uint64_t, rclcpp::Time>
        last_inter_drone_anchor_assist_time_;

    double merged_cloud_voxel_size_m_ = 0.05;
    size_t max_published_global_points_ = 200000;

    std::vector<rclcpp::Subscription<OrbMap>::SharedPtr> orb_map_delta_subs_;
    std::vector<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr> orb_pose_subs_;
    std::vector<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr> gt_pose_subs_;

    std::unordered_map<uint32_t, rclcpp::Publisher<FiducialLockStatus>::SharedPtr>
        fiducial_lock_status_pubs_;

    std::unordered_map<uint32_t, rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr>
        fiducial_ready_to_leave_pubs_;

    std::unordered_map<uint32_t, rclcpp::Client<GetOrbMap>::SharedPtr> full_map_clients_;

    // ============================================================
    // FASE 5.2:
    // Flag para pausar el dron cuando el backend global está ocupado
    // optimizando, esperando optimización o publicando fused pesado.
    // ============================================================

    std::unordered_map<uint32_t, rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr>
        slam_wait_for_optimization_pubs_;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr optimization_busy_pub_;


    std::unordered_map<uint32_t, rclcpp::Publisher<orbslam3_msgs::msg::MapCorrection>::SharedPtr> correction_pubs_;

    std::unordered_map<
        uint32_t,
        rclcpp::Publisher<orbslam3_msgs::msg::CorrectedKeyFrameArray>::SharedPtr>
        corrected_keyframes_pubs_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr global_cloud_raw_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr global_cloud_optimized_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr global_cloud_fused_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr global_cloud_fused_debug_pub_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    rclcpp::TimerBase::SharedPtr full_snapshot_timer_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
    rclcpp::TimerBase::SharedPtr bow_timer_;
    rclcpp::TimerBase::SharedPtr fiducial_timer_;
    rclcpp::TimerBase::SharedPtr optimization_busy_timer_;

    std::unordered_map<uint32_t, DroneMapData> drone_maps_;
    std::unordered_map<uint64_t, HistoricalSubmapMirror> submap_mirrors_;

    std::unordered_map<uint32_t, FiducialLockStatus>
        last_fiducial_status_by_drone_;
    std::unordered_map<uint64_t, GlobalKeyFrameLite> global_keyframes_by_id_;
    std::vector<GlobalSparsePoint> global_sparse_points_;

    bool map_dirty_ = false;
    bool keyframes_dirty_ = false;

    std::vector<SimulatedFiducial> simulated_fiducials_;
    std::unordered_map<uint32_t, std::unordered_map<uint64_t, rclcpp::Time>> last_fiducial_correction_time_;
    std::unordered_map<uint64_t, ActiveFiducialWindow> active_fiducial_windows_;

    std::shared_ptr<orbslam3_multi::MultiDroneSystem> multi_system_;

    std::vector<BowCandidateJob> bow_candidate_queue_;
    std::unordered_set<KeyFramePairKey, KeyFramePairKeyHash> tested_pairs_;
    std::unordered_set<KeyFramePairKey, KeyFramePairKeyHash> queued_pairs_;


    int geometry_ransac_iterations_ = 200;
    double geometry_ransac_threshold_m_ = 0.30;
    int geometry_min_ransac_inliers_ = 8;
    double geometry_min_inlier_ratio_ = 0.15;
    double geometry_max_mean_error_m_ = 0.30;
    double geometry_projection_radius_px_ = 8.0;
    int geometry_projection_descriptor_threshold_ = 60;
    int geometry_min_projection_matches_ = 12;
    int geometry_min_final_inliers_ = 10;

    void RequeueExistingKeyFramesForDrone(uint32_t drone_id);
    void RequeueExistingKeyFramesForAllCorrectedDrones();
    void RequeueAllAtlasKeyFramesForBow();

    bool AreKeyFramesDirectlyConnected(
        uint64_t global_kf_id_a,
        uint64_t global_kf_id_b) const;

    Eigen::Matrix4d Tf2ToEigenMatrix(
        const tf2::Transform& tf) const;

    void UpdateMultiSystemMapAnchor(
        uint32_t drone_id,
        const DroneMapData& data);

    bool enable_pose_graph_building_ = true;
    double pose_graph_build_period_s_ = 2.0;
    int min_covisibility_weight_for_pose_graph_ = 30;
    int max_covisibility_edges_per_kf_ = 5;

    rclcpp::TimerBase::SharedPtr pose_graph_timer_;

    void PoseGraphTimerCallback();

    void PublishFusedLandmarkCloud();

    size_t CountAnchoredDronesForFusedPolicy() const;

    bool enable_pose_graph_optimization_ = false;
    double pose_graph_optimization_period_s_ = 3.0;
    int pose_graph_optimization_iterations_ = 20;

    double pose_graph_optimization_max_step_m_ = 0.8;
    double pose_graph_max_fiducial_prior_weight_ = 120.0;

    bool pose_graph_use_active_window_filter_ = true;
    int pose_graph_active_window_local_kfs_ = 25;

    size_t last_optimized_loop_edge_count_ = 0;
    size_t last_optimized_fiducial_constraint_count_ = 0;
    
    bool optimization_running_ = false;

    // ============================================================
    // FASE 5.2:
    // Estado de backend ocupado para que el controlador pueda pausar.
    // ============================================================

    bool optimization_busy_flag_enabled_ = true;

    double optimization_busy_publish_period_s_ = 0.20;

    // Tiempo extra manteniendo busy=true tras terminar optimización/force.
    double optimization_busy_settle_time_s_ = 1.00;

    // Si publish_fused tarda más que esto, se marca busy temporalmente.
    double optimization_busy_fused_publish_ms_threshold_ = 450.0;

    // Si true, cualquier loop con needs_optimization activa busy.
    bool optimization_busy_include_loop_pending_ = true;

    // Si true, cualquier ventana fiducial esperando optimización activa busy.
    bool optimization_busy_include_fiducial_waiting_ = true;

    // Si true, un fused lento activa busy para que el dron no siga moviéndose
    // mientras RViz/correcciones/fused van por detrás.
    bool optimization_busy_include_slow_fused_publish_ = true;

    rclcpp::Time optimization_busy_hold_until_;
    rclcpp::Time last_slow_fused_publish_time_;

    bool last_global_optimization_busy_ = false;

    std::unordered_map<uint32_t, bool> last_drone_optimization_busy_;

    // FASE 5.3:
    // Publicación de último estado busy desde un hilo ligero.
    // Esto evita que el tópico deje de refrescarse mientras
    // PoseGraphOptimizationTimerCallback está bloqueado en OptimizeEssentialGraph.
    bool optimization_busy_background_thread_enabled_ = true;

    std::atomic<bool> optimization_busy_thread_running_{false};

    std::thread optimization_busy_publish_thread_;

    mutable std::mutex optimization_busy_state_mutex_;

    std::unordered_map<uint32_t, bool> last_drone_optimization_busy_snapshot_;

    bool last_global_optimization_busy_snapshot_ = false;

    rclcpp::TimerBase::SharedPtr pose_graph_optimization_timer_;

    std::unordered_map<uint64_t, Eigen::Matrix4d> optimized_kf_poses_;

    void PoseGraphOptimizationTimerCallback();

    bool HasExplicitOptimizationTaskPlaceholder() const;

    const char* OptimizationTaskTypeToString(
        OptimizationTaskType type) const;

    void LogSubcloudPipelineState(
        const std::string& context);

    bool BuildQueryKeyFrameObservedCloud(
        uint64_t query_global_kf_id,
        std::vector<LoopPoint>& query_points_out,
        Eigen::Matrix4d& world_T_local_out,
        std::string& world_pose_source_out,
        std::string& reason_out) const;

    void LogQueryKeyFrameObservedCloudDiagnostic(
        uint64_t query_global_kf_id,
        const std::string& context) const;

    bool BuildGlobalCandidateSubcloudForQueryKeyFrame(
        uint64_t query_global_kf_id,
        const std::vector<LoopPoint>& query_points,
        std::vector<LoopPoint>& candidate_points_out,
        std::string& search_mode_out,
        std::string& reason_out) const;

    int ComputeOrbDescriptorHammingDistance(
        const std::array<uint8_t, 32>& a,
        const std::array<uint8_t, 32>& b) const;

    bool MatchQueryCloudToGlobalSubcloud(
        const std::vector<LoopPoint>& query_points,
        const std::vector<LoopPoint>& candidate_points,
        std::vector<LoopPointMatch>& matches_out,
        std::string& reason_out) const;

    bool EstimateRigidTransformSvd(
        const std::vector<Eigen::Vector3d>& query_points,
        const std::vector<Eigen::Vector3d>& candidate_points,
        Eigen::Matrix4d& candidate_T_query_out,
        std::string& reason_out) const;

    bool ComputeLoopSubcloudError(
        uint64_t query_global_kf_id,
        const std::string& relation_metadata,
        const std::string& search_mode,
        const std::vector<LoopPoint>& query_points,
        const std::vector<LoopPoint>& candidate_points,
        const std::vector<LoopPointMatch>& matches,
        LoopSubcloudEvent& event_out,
        std::string& reason_out) const;

    bool DecideLoopActionFromSubcloudError(
        const LoopSubcloudEvent& event,
        std::string& decision_out,
        std::string& reason_out) const;

    void LogGlobalCandidateSubcloudDiagnostic(
        uint64_t query_global_kf_id,
        const std::string& context) const;

    SubcloudKeyFrameLoopState& GetOrCreateSubcloudKeyFrameLoopState(
        uint64_t global_kf_id,
        uint32_t drone_id,
        uint64_t map_epoch,
        uint64_t local_kf_id);

    bool SubcloudSubmapHasAnchor(
        uint32_t drone_id,
        uint64_t map_epoch,
        std::string& reason_out) const;

    void UpdateSubcloudKeyFrameLoopStateOnIngest(
        const orbslam3_multi::ImportedKeyFrame& kf,
        const orbslam3_msgs::msg::OrbKeyFrame& kf_msg);

    bool EnqueueSubcloudLoopCheckJob(
        uint64_t query_global_kf_id,
        const std::string& trigger,
        int priority,
        const std::string& reason,
        const std::vector<uint64_t>& candidate_seed_kf_ids = {},
        const std::vector<float>& candidate_seed_scores = {},
        size_t bow_candidates_before_frustum = 0,
        size_t bow_candidates_after_frustum = 0,
        size_t bow_legacy_jobs_queued = 0,
        bool bow_frustum_gating_active = false,
        bool bow_frustum_fallback_used = false,
        const std::string& bow_frustum_gating_reason = "");

    bool TryScheduleSubcloudLoopCheckFromBow(
        const orbslam3_multi::ImportedKeyFrame& kf,
        const std::vector<orbslam3_multi::BowCandidate>& candidates,
        size_t queued_legacy_candidates,
        size_t candidates_before_frustum,
        size_t candidates_after_frustum,
        bool frustum_gating_active,
        bool frustum_fallback_used,
        const std::string& frustum_gating_reason);

    void LogSubcloudLoopQueueState(
        const std::string& context);

    bool BuildBowSeedCandidateSubcloudForJob(
        const SubcloudLoopCheckJob& job,
        std::vector<LoopPoint>& candidate_points_out,
        std::string& search_mode_out,
        std::string& reason_out) const;

    bool ProcessNextSubcloudLoopCheckJob();

    void ProcessSubcloudLoopCheckQueue();

    void UpdateOptimizedSparseMap();

    void OptimizationBusyTimerCallback();

    void PublishOptimizationBusyFlags(
        const std::string& context);

    void OptimizationBusyBackgroundLoop();

    void PublishOptimizationBusyLastState(
        const std::string& context);

    bool IsOptimizationBusyForDrone(
        uint32_t drone_id,
        std::string& reason_out) const;

    bool HasPendingLoopOptimizationForDrone(
        uint32_t drone_id,
        std::string& reason_out) const;

    bool HasFiducialWaitingOptimizationForDrone(
        uint32_t drone_id,
        std::string& reason_out) const;

    std::vector<GlobalSparsePoint> global_sparse_points_optimized_;

    bool optimized_cloud_use_nearest_kf_fallback_ = true;
    double optimized_cloud_nearest_kf_max_distance_m_ = 8.0;

    rclcpp::Time last_loop_edge_time_;
    rclcpp::Time last_keyframe_update_time_;

    double min_seconds_after_last_keyframe_for_optimization_ = 0.0;
    double min_seconds_after_last_loop_for_optimization_ = 2.0;
    double min_seconds_after_last_map_update_for_optimization_ = 1.0;
    size_t min_loop_edges_for_optimization_ = 5;
    bool optimize_on_new_fiducial_priors_ = true;
    bool optimize_on_force_even_without_loops_ = true;
    bool precheck_pose_graph_before_optimization_ = true;

    size_t min_pose_graph_vertices_for_optimization_ = 2;
    size_t min_pose_graph_fixed_vertices_for_optimization_ = 1;

    bool continue_optimization_if_not_converged_ = true;
    double optimization_min_chi2_reduction_ratio_ = 0.35;
    double optimization_large_final_chi2_threshold_ = 1000.0;
    double optimization_large_final_chi2_per_edge_threshold_ = 3.0;
    int optimization_max_continuation_passes_ = 5;
    int optimization_continuation_pass_count_ = 0;

    // ============================================================
    // Fase 8D:
    // Guardarraíl de optimización.
    // Evita publicar una optimización que mueve demasiado anchors fiduciales
    // o que deja chi2 demasiado alto.
    // ============================================================

    bool opt_guard_enable_ = true;

    double opt_guard_max_final_chi2_ = 500.0;
    double opt_guard_max_chi2_per_edge_ = 2.5;

    double opt_guard_max_fiducial_translation_jump_m_ = 0.75;
    double opt_guard_max_fiducial_z_jump_m_ = 0.35;
    double opt_guard_max_fiducial_yaw_jump_deg_ = 8.0;

    double opt_guard_max_any_translation_jump_m_ = 3.0;
    double opt_guard_max_any_yaw_jump_deg_ = 35.0;

    bool opt_guard_reject_on_fiducial_jump_ = true;
    bool opt_guard_reject_on_large_chi_ = true;
    bool opt_guard_log_details_ = true;

    // ============================================================
    // Fase 8B:
    // Promoción de anchors inter-dron provisionales a confirmados.
    // ============================================================

    bool anchor_maturity_enable_promotion_ = true;

    int anchor_maturity_min_inter_opt_loops_ = 2;
    int anchor_maturity_min_inter_fusion_loops_ = 1;

    size_t anchor_maturity_min_inter_supported_groups_ = 50;
    size_t anchor_maturity_min_inter_union_pairs_ = 150;

    double anchor_maturity_max_chi2_per_edge_ = 2.5;

    uint64_t FindBestKeyFrameForCurrentLocalPose(
        uint32_t drone_id,
        const DroneMapData& data,
        const tf2::Transform& T_local_camera,
        double& best_distance_out) const;

    bool force_pose_graph_optimization_ = false;

    bool SyncActiveMapAnchorFromMultiSystem(
        uint32_t drone_id);

    bool UpdateLoopAnchorMaturityFromEvidence(
        const std::string& reason,
        double chi2_final,
        double chi2_per_edge);

    void PropagateAnchorsFromStoredLoops(
        const std::string& reason);

    // ============================================================
    // Fase E:
    // Guardia de seguridad para anchors creados por loops inter-dron.
    // Evita crear anchors si el candidato contradice la pose GT/local
    // actual del dron flotante.
    // ============================================================

    bool phase_e_enable_loop_anchor_guard_ = true;
    bool phase_e_use_gt_current_pose_guard_ = true;

    double phase_e_anchor_guard_max_position_error_m_ = 1.50;
    double phase_e_anchor_guard_max_xy_error_m_ = 1.20;
    double phase_e_anchor_guard_max_z_error_m_ = 0.45;

    bool ValidateStoredLoopEdgeForAnchorUse(
        const orbslam3_multi::VerifiedLoopEdge& edge,
        std::string& reject_reason_out);

    bool ComputeStoredLoopAnchorCandidateWorldTLocal(
        const orbslam3_multi::VerifiedLoopEdge& edge,
        uint32_t& target_drone_id_out,
        uint64_t& target_map_epoch_out,
        Eigen::Matrix4d& candidate_world_T_local_out,
        std::string& reject_reason_out) const;

    Eigen::Matrix4d PoseMsgToEigenMatrix(
        const geometry_msgs::msg::Pose& pose) const;

    Eigen::Matrix4d TransformMsgToEigenMatrix(
        const geometry_msgs::msg::Transform& transform) const;

    std::unordered_map<uint64_t, OptimizedSubmapTransform>
        optimized_world_T_local_by_submap_;

    uint64_t MakeSubmapKey(
        uint32_t drone_id,
        uint64_t map_epoch) const;

    bool IsActiveDroneEpoch(
        uint32_t drone_id,
        uint64_t map_epoch) const;

    Eigen::Matrix4d KeyFrameLocalPoseToMatrix(
        const orbslam3_multi::ImportedKeyFrame& kf) const;

    geometry_msgs::msg::Transform EigenMatrixToTransformMsg(
        const Eigen::Matrix4d& T) const;


    void BuildOptimizedSubmapTransformsFromPoses(
        const std::unordered_map<uint64_t, Eigen::Matrix4d>& optimized_poses,
        std::unordered_map<uint64_t, OptimizedSubmapTransform>& out) const;

    double MatrixYawDeg(
        const Eigen::Matrix4d& T) const;

    double MatrixRollDeg(
        const Eigen::Matrix4d& T) const;

    double MatrixPitchDeg(
        const Eigen::Matrix4d& T) const;

    double NormalizeAngleDeg(
        double a) const;

    bool CheckOptimizationGuard(
        const std::unordered_map<uint64_t, OptimizedSubmapTransform>& candidate_submaps,
        double chi2_final,
        double chi2_per_edge,
        std::string& reject_reason_out,
        double& max_any_jump_out,
        double& max_fiducial_jump_out,
        double& max_fiducial_yaw_jump_out) const;


    void RebuildOptimizedSubmapTransforms();

    void ApplyOptimizedSubmapCorrectionsToActiveDrones();

    void ApplyFastAnchorDeltaToOptimizedCaches(
        uint32_t drone_id,
        uint64_t map_epoch,
        const Eigen::Matrix4d& delta_world_T_world);

    bool AnyDroneHasCorrection() const;

    uint64_t MakeFiducialWindowKey(
        uint32_t drone_id,
        uint64_t map_epoch,
        uint32_t fiducial_id) const;

    bool CheckAndRequestFiducialWaitingOptimization(
        const std::string& context);

    bool FiducialWindowNeedsForcedOptimization(
        const ActiveFiducialWindow& window,
        const rclcpp::Time& t_now,
        std::string& reason_out) const;

    bool FiducialWindowHasOptimizationDebt(
        const ActiveFiducialWindow& window,
        const rclcpp::Time& t_now,
        std::string& reason_out) const;

    bool HasAnyFiducialOptimizationDebt(
        std::string& reason_out) const;

    bool ComputeFiducialWindowTransformError(
        const ActiveFiducialWindow& window,
        double& error_t_out,
        double& error_yaw_deg_out,
        std::string& reason_out) const;

    bool TryMarkFiducialWindowOptimizedIfLowError(
        ActiveFiducialWindow& window,
        const std::string& context,
        std::string& reason_out);

    bool UnifiedLoopEventIsPreOptimizationConflict(
        const UnifiedLoopRuntimeEvent& event,
        std::string& reason_out) const;

    bool UnifiedLoopEventHasOptimizationDebt(
        const UnifiedLoopRuntimeEvent& event,
        std::string& reason_out) const;

    bool HasAnyLoopOptimizationDebt(
        std::string& reason_out) const;

    void AdvanceOptimizationCountersForBenignTriggers(
        size_t current_loop_edges,
        size_t current_fiducial_constraints,
        bool advance_loops,
        bool advance_priors,
        const std::string& context);
        
    bool PhaseAHasGlobalFiducialOptimizationTask(
        bool fiducial_force_requested,
        bool forced_optimization,
        bool effective_new_fiducial_priors_for_optimization,
        const std::string& fiducial_debt_reason,
        std::string& task_reason_out) const;

    bool PhaseAHasLocalLoopOptimizationTask(
        bool loop_force_requested,
        bool forced_optimization,
        bool effective_new_loop_edges_for_optimization,
        const std::string& loop_debt_reason,
        std::string& task_reason_out) const;

    void LogPhaseADecision(
        const std::string& context,
        bool forced_optimization,
        bool fiducial_force_requested,
        bool loop_force_requested,
        bool has_new_loop_edges,
        bool has_new_fiducial_priors,
        bool effective_new_loop_edges_for_optimization,
        bool effective_new_fiducial_priors_for_optimization,
        bool has_fiducial_optimization_debt,
        bool has_loop_optimization_debt,
        bool has_global_fiducial_task,
        bool has_local_loop_task,
        const std::string& fiducial_reason,
        const std::string& loop_reason) const;

    void PhaseBMaybeResetLoopThroughputWindow();

    void PhaseBLogLoopThroughputSummary(
        const std::string& context);

    void PhaseBLogKeyFrameFunnel(
        const std::string& context,
        uint32_t drone_id,
        uint64_t epoch,
        uint64_t local_kf_id,
        uint64_t global_kf_id,
        size_t bow_candidates_raw,
        size_t bow_candidates_after_frustum,
        size_t queued,
        size_t reject_self,
        size_t reject_invalid,
        size_t reject_tested,
        size_t reject_queued,
        const std::string& reason_if_zero);

    uint64_t GetMaxLocalKeyFrameId(
        const DroneMapData& data) const;

    bool HasSeenDifferentFiducialInEpoch(
        uint32_t drone_id,
        uint64_t map_epoch,
        uint32_t fiducial_id) const;

    void UpdateActiveFiducialWindow(
        uint32_t drone_id,
        DroneMapData& data,
        const SimulatedFiducial& fiducial,
        double distance_to_fiducial,
        const tf2::Transform& T_world_camera,
        const tf2::Transform& T_local_camera);

    void CloseExpiredFiducialWindows();

    void TryCreateFiducialPriorsForWindow(
        ActiveFiducialWindow& window,
        DroneMapData& data);

    bool TryCreateFiducialPriorForKeyFrame(
        const ActiveFiducialWindow& window,
        DroneMapData& data,
        uint64_t local_kf_id,
        const std::string& reason,
        bool allow_update_existing = false);
    
    void RefreshExistingFiducialPriorsForWindow(
        ActiveFiducialWindow& window,
        DroneMapData& data,
        const std::string& reason);
    
    void TryCreateBackfillPriorsForWindow(
        ActiveFiducialWindow& window,
        DroneMapData& data);

    double fiducial_window_timeout_s_ = 5.0;
    double fiducial_window_prior_weight_ = 12.0;
    int fiducial_window_max_new_kfs_per_tick_ = 20;

    // Fase 8: condiciones para permitir salir del fiducial.
    int fiducial_lock_min_good_priors_ = 3;
    int fiducial_lock_min_constrained_kfs_ = 3;

    double fiducial_lock_good_prior_max_dist_m_ = 1.5;
    double fiducial_lock_max_min_distance_m_ = 1.5;
    double fiducial_lock_min_inside_duration_s_ = 1.0;
    double fiducial_lock_timeout_s_ = 5.0;

    bool fiducial_lock_require_local_warp_ = false;
    bool fiducial_lock_require_graph_optimization_ = false;

    double fiducial_lock_publish_period_s_ = 0.2;

    rclcpp::TimerBase::SharedPtr fiducial_lock_status_timer_;
    rclcpp::TimerBase::SharedPtr run_diagnostics_timer_;
    double run_diagnostics_period_s_ = 2.0;

    // ============================================================
    // FASE 0:
    // Logs de auditoría. Mantener activados solo durante diagnóstico.
    // No alteran el pipeline, solo imprimen resúmenes.
    // ============================================================

    bool phase0_audit_logs_enabled_ = true;

    // Log por submapa en RunDiagnosticsTimerCallback().
    bool phase0_submap_audit_enabled_ = true;

    // Log de fused legacy/cache en PublishFusedLandmarkCloud().
    bool phase0_fused_legacy_audit_enabled_ = true;

    // Log de tiempos del PublishTimerCallback().
    bool phase0_perf_tick_enabled_ = true;

    // Throttle común para no inundar consola.
    int phase0_audit_throttle_ms_ = 2000;

    // ============================================================
    // Fase B:
    // Diagnóstico inter-dron entre submapas confirmados.
    // No crea asociaciones ni fusiona landmarks.
    // ============================================================

    bool enable_inter_drone_fusion_diagnostics_ = true;

    double inter_fusion_diag_max_world_distance_m_ = 0.15;
    int inter_fusion_diag_max_descriptor_distance_ = 70;

    uint32_t inter_fusion_diag_min_observations_ = 2;
    double inter_fusion_diag_min_found_ratio_ = 0.25;

    int inter_fusion_diag_max_mps_per_submap_ = 1500;
    int inter_fusion_diag_max_pair_checks_ = 250000;

    int inter_fusion_diag_max_sample_logs_ = 20;

    bool enable_fast_fiducial_anchor_guess_ = false;
    bool fast_anchor_only_if_no_other_fiducial_in_epoch_ = true;

    double fast_fiducial_anchor_blend_ = 0.6;
    double fast_fiducial_anchor_min_improvement_m_ = 0.3;

    bool enable_local_fiducial_cache_warp_ = false;
    double local_fiducial_cache_warp_blend_ = 0.8;
    int local_fiducial_cache_warp_margin_kfs_ = 15;
    double local_fiducial_cache_warp_max_distance_m_ = 4.0;

    void TryAssociateKeyFrameToActiveFiducialWindows(
        uint32_t drone_id,
        uint64_t map_epoch,
        uint64_t local_kf_id);

    bool ShouldApplyLocalFiducialWarp(
        const ActiveFiducialWindow& window,
        const std::string& reason) const;

    double fiducial_window_kf_time_margin_before_s_ = 0.5;
    double fiducial_window_kf_time_margin_after_s_ = 1.0;

    double ComputeFiducialWindowPriorWeight(
        const tf2::Transform& T_world_kf_measured,
        const ActiveFiducialWindow& window) const;

    int fiducial_window_backfill_kfs_ = 3;

    double fiducial_prior_reject_distance_m_ = 3.0;
    double fiducial_prior_strong_distance_m_ = 1.5;
    bool reject_far_fiducial_priors_ = true;

    size_t fiducial_window_max_constraints_per_window_ = 12;

    tf2::Transform InterpolateTf2Transform(
        const tf2::Transform& A,
        const tf2::Transform& B,
        double alpha) const;

    bool ApplyFastFiducialAnchorGuess(
        uint32_t drone_id,
        DroneMapData& data,
        const ActiveFiducialWindow& window,
        const tf2::Transform& T_world_camera,
        const tf2::Transform& T_local_camera,
        double distance_to_fiducial);

    bool ApplyLocalFiducialCacheWarp(
        uint32_t drone_id,
        DroneMapData& data,
        ActiveFiducialWindow& window,
        const tf2::Transform& T_world_camera,
        const tf2::Transform& T_local_camera,
        double distance_to_fiducial,
        const std::string& reason);

    void EigenMatrixToTf2(
        const Eigen::Matrix4d& T,
        tf2::Transform& out) const;
    
    // ============================================================
    // FASE 1:
    // Condiciones explícitas para publicar fiducial_ready_to_leave.
    // Estos parámetros convierten el fiducial en un semáforo de misión
    // manual: el dron puede salir cuando el submapa está suficientemente
    // anclado, no simplemente cuando entró en el radio.
    // ============================================================

    int fiducial_ready_min_window_kfs_ = 5;
    int fiducial_ready_min_good_priors_ = 3;

    bool fiducial_ready_require_optimization_ = true;
    bool fiducial_ready_require_low_chi_ = true;
    bool fiducial_ready_allow_initial_anchor_without_optimization_ = false;

    double fiducial_ready_min_inside_duration_s_ = 1.0;
    double fiducial_ready_max_seconds_since_seen_s_ = 1.5;

    double fiducial_ready_max_min_distance_m_ = 1.5;

    double fiducial_ready_max_final_chi2_ = 50.0;
    double fiducial_ready_max_chi2_per_edge_ = 0.50;

    std::unordered_map<uint64_t, Eigen::Matrix4d>
        pending_optimization_kf_poses_;

    bool has_pending_optimization_guess_ = false;

    void InterDroneAnchorAssistTick();

    uint64_t MakeDronePairKey(
        uint32_t drone_a,
        uint32_t drone_b) const;

    size_t RequeueRecentKeyFramesForDroneForAnchorAssist(
        uint32_t drone_id,
        int max_recent_kfs);

    size_t ClearTestedPairsBetweenDrones(
        uint32_t drone_a,
        uint32_t drone_b);

    size_t ClearQueuedPairsBetweenDrones(
        uint32_t drone_a,
        uint32_t drone_b);

    bool IsInterDroneAnchorAssistBoostActive() const;

    std::vector<uint64_t> CollectRecentGlobalKeyFramesForDrone(
        uint32_t drone_id,
        int max_recent_kfs) const;

    bool EnqueueExplicitBowCandidatePair(
        uint64_t query_global_kf_id,
        uint64_t candidate_global_kf_id,
        double score,
        const std::string& reason);

    size_t EnqueueExplicitInterDroneAnchorPairs(
        uint32_t anchored_drone,
        uint32_t floating_drone,
        int max_anchored_kfs,
        int max_floating_kfs,
        int max_pairs);

    void PublishImmediatelyAfterLoopAnchor(
        const std::string& reason,
        uint32_t q_drone,
        uint32_t c_drone);


    bool publish_corrected_keyframes_ = true;

    // ============================================================
    // FASE 4.1:
    // Anti-bloqueo de publicación pesada.
    // Evita publicar raw/optimized/fused dentro del callback de optimización.
    // ============================================================

    // Si false, /global_sparse_map_raw y /global_sparse_map_optimized no se publican
    // en cada tick. Muy útil para pruebas donde solo queremos /global_sparse_map_fused.
    bool publish_raw_and_optimized_clouds_ = true;

    // Si false, PoseGraphOptimizationTimerCallback no publica raw/optimized/fused
    // inmediatamente después de optimizar. Solo marca dirty y deja que PublishTimerCallback
    // publique después.
    bool optimization_immediate_heavy_publish_ = false;

    // Si true, aunque optimization_immediate_heavy_publish_=false,
    // se publican correcciones/corrected keyframes/TF tras optimizar.
    bool optimization_immediate_light_publish_ = true;

    // ============================================================
    // FASE 4.3:
    // Publicar MapPoints fused usando poses optimizadas de KeyFrames.
    // Esto permite corregir deriva interna dentro del mismo epoch,
    // no solo aplicar una transformación rígida world_T_local.
    // ============================================================

    bool fused_simple_use_optimized_keyframe_points_ = true;

    // Número máximo de KeyFrames observadores usados por cada MapPoint.
    int fused_simple_optimized_point_max_observations_ = 4;

    // Profundidad máxima permitida del punto en la cámara del KeyFrame usado.
    double fused_simple_optimized_point_max_camera_depth_m_ = 40.0;

    // Si la posición optimizada difiere absurdamente de la rígida,
    // se usa fallback rígido. Esto evita explosiones por poses malas.
    double fused_simple_optimized_point_max_delta_from_rigid_m_ = 6.0;

    bool fused_simple_log_optimized_point_usage_ = true;

    // ============================================================
    // FASE 4.2:
    // Forzar optimización cuando una ventana fiducial está esperando
    // optimización, aunque el número total de constraints no haya aumentado.
    // ============================================================

    bool fiducial_force_optimization_when_waiting_ = true;

    // Evita spamear force cada tick. Si sigue esperando, se reintenta
    // cada N segundos.
    double fiducial_force_optimization_min_interval_s_ = 1.0;

    // Condiciones mínimas para forzar optimización desde fiducial.
    int fiducial_force_optimization_min_good_priors_ = 3;
    int fiducial_force_optimization_min_total_priors_ = 5;

    // Si una optimización forzada por fiducial fue rechazada por OPT-GUARD,
    // no repetirla inmediatamente.
    double fiducial_force_optimization_reject_backoff_s_ = 8.0;

    // Si true, solo fuerza optimización para ventanas activas dentro del fiducial.
    bool fiducial_force_optimization_only_active_window_ = true;

    // Última vez que se forzó optimización por cada ventana fiducial.
    std::unordered_map<uint64_t, rclcpp::Time> last_fiducial_force_optimization_time_;

    int corrected_keyframes_max_per_drone_ = 200;
    double corrected_keyframes_publish_period_s_ = 0.5;

    // Publica también si no hay pose graph optimizado,
    // usando fallback rígido world_T_local * T_local_kf.
    // Útil para que el corrector deje de tener corrected_kfs_age=inf.
    bool corrected_keyframes_publish_rigid_fallback_ = true;

    void PublishCorrectedKeyFrames();

    orbslam3_msgs::msg::CorrectedKeyFrameArray
    BuildCorrectedKeyFrameArrayForDrone(
        uint32_t drone_id,
        const DroneMapData& data);

    geometry_msgs::msg::Pose MatrixToPoseMsg(
        const Eigen::Matrix4d& T) const;

    bool bow_debug_intra_candidates_ = true;
    bool bow_log_rejected_candidates_ = true;

    // ============================================================
    // FASE 2:
    // Diagnóstico unificado de loops.
    // No cambia comportamiento: solo logs.
    // ============================================================

    bool unified_loop_diag_enabled_ = true;

    // Loggea candidatos BoW aunque no lleguen a geometría.
    bool unified_loop_diag_log_candidates_ = true;

    // Loggea rechazos geométricos. Puede generar bastante salida,
    // pero es útil en esta fase.
    bool unified_loop_diag_log_rejected_geometry_ = true;

    // Umbrales SOLO para decisión dry-run.
    // No se usan todavía para cambiar comportamiento.
    double unified_loop_diag_opt_error_t_m_ = 0.20;
    double unified_loop_diag_opt_error_yaw_deg_ = 5.0;

    // Para evitar logs enormes si el sistema se dispara.
    // 0 significa sin límite práctico por tick.
    int unified_loop_diag_max_logs_per_tick_ = 80;

    uint64_t next_unified_loop_event_id_ = 1;

    // ============================================================
    // FASE 3:
    // Runtime de loops unificados.
    // Sigue siendo diagnóstico: todavía no fusiona MapPoints.
    // ============================================================

    std::unordered_map<uint64_t, UnifiedLoopRuntimeEvent> unified_loop_events_;

    // ============================================================
    // SUBCLOUD PIPELINE:
    // Eventos y tareas explícitas del pipeline nuevo. En Fase 3
    // solo se crean y se diagnostican; no se consumen para cambiar
    // comportamiento.
    // ============================================================

    std::unordered_map<uint64_t, LoopSubcloudEvent> loop_subcloud_events_;

    std::deque<OptimizationTask> optimization_tasks_;

    uint64_t next_loop_subcloud_event_id_ = 1;
    uint64_t next_optimization_task_id_ = 1;

    bool subcloud_pipeline_diagnostics_enabled_ = true;
    int subcloud_pipeline_diag_throttle_ms_ = 2000;

    // ============================================================
    // FASE 4R:
    // Las funciones query/candidate/match quedan como herramientas
    // bajo demanda. No deben ejecutarse automáticamente desde el
    // callback de llegada de KeyFrames.
    // ============================================================

    bool subcloud_tool_diagnostics_enabled_ = true;

    bool subcloud_kf_ingest_diagnostics_enabled_ = true;
    int subcloud_kf_ingest_throttle_ms_ = 2000;

    bool subcloud_query_cloud_diagnostics_enabled_ = false;

    // Si hay menos puntos válidos que este umbral, el log [QUERY-KF-CLOUD]
    // se marca como query_small=true. No filtra ni descarta puntos.
    int subcloud_query_cloud_min_valid_points_warn_ = 15;

    // ============================================================
    // FASE 5:
    // Subnube global candidata para el nuevo pipeline.
    // En esta fase solo se construye y loguea; no decide fusión ni
    // optimización.
    // ============================================================

    bool subcloud_candidate_cloud_diagnostics_enabled_ = false;

    int subcloud_candidate_cloud_min_points_warn_ = 30;
    int subcloud_candidate_frustum_min_points_ = 30;

    double subcloud_candidate_score_threshold_ = 0.60;
    double subcloud_candidate_supported_score_threshold_ = 0.20;
    bool subcloud_candidate_include_supported_unpublished_ = true;

    double subcloud_candidate_radius_m_ = 3.0;
    size_t subcloud_candidate_max_points_ = 1200;

    double subcloud_candidate_frustum_min_depth_m_ = 0.30;
    double subcloud_candidate_frustum_max_depth_m_ = 20.0;
    double subcloud_candidate_frustum_margin_px_ = 20.0;

    // ============================================================
    // FASE 6:
    // Matching query cloud ↔ candidate global subcloud.
    // Diagnóstico only: no fusiona, no optimiza, no ancla.
    // ============================================================

    bool subcloud_match_diagnostics_enabled_ = false;

    // No hacemos matching sobre todos los KeyFrames para evitar coste extra.
    // 1 = todos, 2 = uno de cada dos, 3 = uno de cada tres, etc.
    int subcloud_match_every_n_keyframes_ = 2;

    int subcloud_match_min_query_points_ = 30;
    int subcloud_match_min_candidate_points_ = 30;

    size_t subcloud_match_max_query_points_ = 350;
    size_t subcloud_match_max_candidate_points_ = 800;
    size_t subcloud_match_max_matches_ = 250;

    int subcloud_match_max_hamming_ = 75;

    bool subcloud_match_use_ratio_test_ = true;
    double subcloud_match_ratio_ = 0.85;

    bool subcloud_match_use_spatial_gate_ = true;
    double subcloud_match_max_spatial_distance_m_ = 2.0;

    // FASE 7R:
    // Permite que BOW_ANCHOR_CANDIDATE haga matching descriptor-only.
    // La query no anclada no tiene p_world, pero sí tiene p_local y descriptor.
    bool subcloud_match_allow_query_without_world_ = true;

    // ============================================================
    // FASE 8R:
    // LOOP-SUBCLOUD-ERROR con RANSAC 3D-3D.
    // Diagnóstico only: no fusiona, no optimiza, no ancla.
    // ============================================================

    bool subcloud_error_diagnostics_enabled_ = true;
    bool subcloud_error_store_events_ = true;

    int subcloud_error_ransac_iterations_ = 80;
    int subcloud_error_min_matches_ = 20;
    int subcloud_error_min_inliers_ = 15;

    double subcloud_error_inlier_threshold_m_ = 0.35;
    double subcloud_error_min_inlier_ratio_ = 0.25;

    int subcloud_error_max_events_ = 1000;

    // ============================================================
    // FASE 9R:
    // Decisión única dry-run desde LOOP-SUBCLOUD-ERROR.
    // No fusiona, no optimiza, no ancla.
    // ============================================================

    bool subcloud_decision_dry_run_enabled_ = true;

    // Requisitos mínimos generales para considerar una decisión fuerte.
    int subcloud_decision_min_inliers_ = 20;
    double subcloud_decision_min_inlier_ratio_ = 0.30;
    double subcloud_decision_min_coverage_ = 0.03;

    // Si el error está por debajo de estos umbrales y el residual es bueno,
    // el loop se clasifica como FUSION_EVENT dry-run.
    // Importante: el umbral de 0.35 m evita optimizar casos visualmente buenos
    // donde RANSAC estima 15-25 cm por ruido/degeneración planar.
    double subcloud_decision_fusion_max_error_t_m_ = 0.35;
    double subcloud_decision_fusion_max_yaw_deg_ = 4.0;
    double subcloud_decision_fusion_max_rot_deg_ = 6.0;
    double subcloud_decision_fusion_max_mean_residual_m_ = 0.10;
    double subcloud_decision_fusion_max_residual_m_ = 0.35;
    int subcloud_decision_fusion_min_inliers_ = 25;
    double subcloud_decision_fusion_min_inlier_ratio_ = 0.45;
    double subcloud_decision_fusion_min_coverage_ = 0.03;

    // Si el error es alto, pero los inliers/residual son buenos,
    // se clasifica como LOCAL_LOOP_OPT_TASK dry-run.
    double subcloud_decision_opt_min_error_t_m_ = 0.60;
    double subcloud_decision_opt_min_yaw_deg_ = 8.0;
    double subcloud_decision_opt_min_rot_deg_ = 10.0;
    double subcloud_decision_opt_max_mean_residual_m_ = 0.15;
    double subcloud_decision_opt_max_residual_m_ = 0.45;
    int subcloud_decision_opt_min_inliers_ = 25;
    double subcloud_decision_opt_min_inlier_ratio_ = 0.35;
    double subcloud_decision_opt_min_coverage_ = 0.03;

    // FASE 11R-A:
    // Regla hard para errores grandes con geometría razonable.
    // Evita que un caso tipo 3 m / 30 grados se quede en HOLD solo
    // por tener menos inliers que el umbral normal.
    bool subcloud_decision_hard_opt_enabled_ = true;

    double subcloud_decision_hard_opt_min_error_t_m_ = 0.80;
    double subcloud_decision_hard_opt_min_yaw_deg_ = 10.0;
    double subcloud_decision_hard_opt_min_rot_deg_ = 12.0;

    int subcloud_decision_hard_opt_min_inliers_ = 15;
    double subcloud_decision_hard_opt_min_inlier_ratio_ = 0.25;
    double subcloud_decision_hard_opt_min_coverage_ = 0.03;

    double subcloud_decision_hard_opt_max_mean_residual_m_ = 0.18;
    double subcloud_decision_hard_opt_max_residual_m_ = 0.50;

    // ============================================================
    // FASE 11R-C.2:
    // Reglas adicionales para que errores claramente grandes no queden
    // bloqueados como HOLD por un residual ligeramente superior.
    // No fusiona, no optimiza; solo permite crear LOCAL_LOOP_OPT_TASK.
    // ============================================================

    // Error extremo: permite residual medio algo más alto si el error de pose
    // es claramente incompatible con una simple fusión.
    bool subcloud_decision_very_hard_opt_enabled_ = true;

    double subcloud_decision_very_hard_opt_min_error_t_m_ = 2.0;
    double subcloud_decision_very_hard_opt_min_yaw_deg_ = 20.0;
    double subcloud_decision_very_hard_opt_min_rot_deg_ = 20.0;

    int subcloud_decision_very_hard_opt_min_inliers_ = 20;
    double subcloud_decision_very_hard_opt_min_inlier_ratio_ = 0.35;
    double subcloud_decision_very_hard_opt_min_coverage_ = 0.05;

    double subcloud_decision_very_hard_opt_max_mean_residual_m_ = 0.22;
    double subcloud_decision_very_hard_opt_max_residual_m_ = 0.40;

    // Error moderado/fuerte: para casos como 0.6 m con muchos inliers,
    // buena cobertura y residual todavía razonable.
    bool subcloud_decision_relaxed_opt_enabled_ = true;

    double subcloud_decision_relaxed_opt_min_error_t_m_ = 0.60;
    double subcloud_decision_relaxed_opt_min_yaw_deg_ = 8.0;
    double subcloud_decision_relaxed_opt_min_rot_deg_ = 8.0;

    int subcloud_decision_relaxed_opt_min_inliers_ = 30;
    double subcloud_decision_relaxed_opt_min_inlier_ratio_ = 0.50;
    double subcloud_decision_relaxed_opt_min_coverage_ = 0.08;

    double subcloud_decision_relaxed_opt_max_mean_residual_m_ = 0.18;
    double subcloud_decision_relaxed_opt_max_residual_m_ = 0.35;

    // ============================================================
    // FASE 11R-C3:
    // Regla específica para deriva lateral/traslacional.
    // Captura casos con error_t medio, yaw bajo, muchos inliers,
    // pero mean_residual algo alto por separación lateral/ruido de nube.
    // No aplica optimización real; solo permite crear dry-run.
    // ============================================================

    bool subcloud_decision_lateral_opt_enabled_ = true;

    double subcloud_decision_lateral_opt_min_error_t_m_ = 0.50;

    double subcloud_decision_lateral_opt_max_yaw_deg_ = 5.0;
    double subcloud_decision_lateral_opt_max_rot_deg_ = 6.0;

    int subcloud_decision_lateral_opt_min_inliers_ = 35;
    double subcloud_decision_lateral_opt_min_inlier_ratio_ = 0.45;
    double subcloud_decision_lateral_opt_min_coverage_ = 0.07;

    double subcloud_decision_lateral_opt_max_mean_residual_m_ = 0.27;
    double subcloud_decision_lateral_opt_max_residual_m_ = 0.36;

    // ============================================================
    // 12R-D4:
    // Puente conservador desde diagnóstico unified hacia evento subcloud.
    // LOOP-ERROR-UNIFIED no decide solo: debe aportar geometría fuerte,
    // pares 3D reconstruibles y terminar como LoopSubcloudEvent moderno.
    // ============================================================

    bool subcloud_decision_unified_aux_opt_enabled_ = true;

    double subcloud_decision_unified_aux_min_error_t_m_ = 0.60;
    double subcloud_decision_unified_aux_min_yaw_deg_ = 8.0;
    double subcloud_decision_unified_aux_min_rot_deg_ = 10.0;

    int subcloud_decision_unified_aux_min_final_inliers_ = 20;
    double subcloud_decision_unified_aux_min_inlier_ratio_ = 0.25;
    int subcloud_decision_unified_aux_min_reconstructed_pairs_ = 15;

    double subcloud_decision_unified_aux_max_mean_geom_error_m_ = 0.20;
    double subcloud_decision_unified_aux_max_geom_error_m_ = 0.50;

    // Para BOW_ANCHOR_CANDIDATE no hay error_t/yaw contra pose world previa.
    // Se decide solo con inliers, cobertura y residual.
    int subcloud_decision_anchor_min_inliers_ = 25;
    double subcloud_decision_anchor_min_inlier_ratio_ = 0.35;
    double subcloud_decision_anchor_min_coverage_ = 0.03;
    double subcloud_decision_anchor_max_mean_residual_m_ = 0.10;
    double subcloud_decision_anchor_max_residual_m_ = 0.35;

    // ============================================================
    // FASE 10R-A:
    // Fusión real conservadora desde FUSION_EVENT.
    // ============================================================

    bool subcloud_fusion_enabled_ = true;

    // Déjalo false en Fase 10R-A para que sea fusión real.
    // Si algo sale raro, se puede poner true para volver a modo solo log.
    bool subcloud_fusion_dry_run_ = false;

    bool subcloud_fusion_process_event_once_ = true;

    int subcloud_fusion_max_events_per_tick_ = 4;
    int subcloud_fusion_max_pairs_per_event_ = 120;
    int subcloud_fusion_min_accepted_pairs_ = 12;

    double subcloud_fusion_max_pair_distance_m_ = 0.35;
    double subcloud_fusion_max_residual_m_ = 0.35;
    int subcloud_fusion_max_hamming_ = 85;

    double subcloud_fusion_observation_weight_scale_ = 2.5;
    double subcloud_fusion_initial_score_ = 0.70;

    bool subcloud_fusion_validate_region_ = true;
    double subcloud_fusion_validated_region_radius_m_ = 1.50;

    bool subcloud_fusion_log_pairs_ = false;
    int subcloud_fusion_max_pair_logs_ = 20;

    // FASE 10R-B:
    // Usar snapshots guardados en LoopPointMatch para fusionar.
    // Esto es la corrección principal de 10R-A.
    bool subcloud_fusion_use_match_snapshots_ = true;

    // Evita reintentar indefinidamente eventos que ya se han procesado.
    // En tu log había eventos con attempts=20 y pairs_accepted=0.
    int subcloud_fusion_max_attempts_per_event_ = 2;

    // Evidencia por MapPoint para score global del servidor.
    // No modifica MapPoint ORB-SLAM3 interno.
    bool subcloud_mappoint_evidence_enabled_ = true;
    double subcloud_evidence_inlier_gain_ = 0.025;
    double subcloud_evidence_outlier_penalty_ = 0.008;
    double subcloud_evidence_max_positive_ = 0.25;
    double subcloud_evidence_max_negative_ = -0.12;
    double subcloud_evidence_apply_weight_scale_ = 1.0;

    // Covisibilidad global por KeyFrames del servidor.
    bool subcloud_kf_covisibility_enabled_ = true;
    int subcloud_kf_covisibility_min_accepted_pairs_ = 8;
    bool subcloud_kf_covisibility_log_edges_ = true;

    std::unordered_map<uint64_t, SubcloudMapPointEvidence> subcloud_mappoint_evidence_;

    std::unordered_map<
        KeyFramePairKey,
        SubcloudKeyFrameCovisibilityEdge,
        KeyFramePairKeyHash> subcloud_kf_covisibility_edges_;

    // ============================================================
    // FASE 11R-A:
    // LOCAL_LOOP_OPT_TASK dry-run + pending region + bloqueo.
    // ============================================================

    bool subcloud_local_loop_task_generation_enabled_ = true;
    bool subcloud_local_loop_task_dry_run_ = true;
    bool subcloud_local_loop_task_process_once_ = true;

    bool subcloud_local_loop_task_publish_busy_ = true;
    bool subcloud_local_loop_task_block_fusion_ = true;
    bool subcloud_local_loop_task_mark_new_kfs_pending_ = true;
    bool subcloud_local_loop_task_publish_new_kfs_provisional_ = true;

    int subcloud_local_loop_task_max_per_tick_ = 4;
    int subcloud_local_loop_task_max_attempts_ = 1;

    bool subcloud_local_loop_task_mark_state_dirty_ = true;
    bool subcloud_local_loop_task_log_existing_ = true;
    bool subcloud_local_loop_task_log_blocked_fusion_ = true;

    std::unordered_map<uint64_t, SubcloudPendingOptimizationRegion>
        subcloud_pending_opt_regions_;

    std::unordered_map<uint64_t, uint64_t>
        submap_key_to_pending_opt_region_id_;

    uint64_t next_subcloud_pending_opt_region_id_ = 1;

    // ============================================================
    // FASE 11R-B:
    // Construcción dry-run de ventana local.
    // ============================================================

    bool subcloud_local_opt_window_enabled_ = true;
    bool subcloud_local_opt_window_build_once_ = true;

    int subcloud_local_opt_window_max_per_tick_ = 2;

    double subcloud_local_opt_window_query_radius_m_ = 2.0;
    double subcloud_local_opt_window_candidate_radius_m_ = 2.0;
    double subcloud_local_opt_window_boundary_radius_m_ = 3.0;

    int subcloud_local_opt_window_query_temporal_span_ = 8;
    int subcloud_local_opt_window_candidate_temporal_span_ = 6;

    int subcloud_local_opt_window_max_query_kfs_ = 18;
    int subcloud_local_opt_window_max_candidate_seed_kfs_ = 8;
    int subcloud_local_opt_window_max_candidate_kfs_ = 24;
    int subcloud_local_opt_window_max_covisibility_kfs_ = 12;
    int subcloud_local_opt_window_max_boundary_kfs_ = 16;
    int subcloud_local_opt_window_max_total_kfs_ = 64;

    int subcloud_local_opt_window_min_candidate_inlier_support_ = 3;

    bool subcloud_local_opt_window_include_same_submap_temporal_ = true;
    bool subcloud_local_opt_window_include_covisibility_ = true;
    bool subcloud_local_opt_window_include_boundary_ = true;
    bool subcloud_local_opt_window_include_pending_as_excluded_ = true;

    double subcloud_local_opt_window_weight_base_ = 1.0;
    double subcloud_local_opt_window_weight_query_root_ = 2.0;
    double subcloud_local_opt_window_weight_candidate_seed_ = 3.0;
    double subcloud_local_opt_window_weight_anchor_ = 8.0;
    double subcloud_local_opt_window_weight_fiducial_prior_ = 12.0;
    double subcloud_local_opt_window_weight_validated_ = 4.0;
    double subcloud_local_opt_window_weight_fusion_done_ = 3.0;
    double subcloud_local_opt_window_weight_covis_per_pair_ = 0.02;
    double subcloud_local_opt_window_weight_covis_cap_ = 6.0;
    double subcloud_local_opt_window_weight_boundary_ = 20.0;
    double subcloud_local_opt_window_weight_pending_excluded_ = 0.05;

    bool subcloud_local_opt_window_log_kfs_ = true;
    int subcloud_local_opt_window_max_kf_logs_ = 80;

    std::unordered_map<uint64_t, LocalOptimizationWindow>
        subcloud_local_opt_windows_;

    uint64_t next_subcloud_local_opt_window_id_ = 1;

    // ============================================================
    // FASE 11R-C:
    // Optimización numérica dry-run.
    // ============================================================

    bool subcloud_local_opt_dryrun_enabled_ = true;
    bool subcloud_local_opt_dryrun_run_once_ = true;

    int subcloud_local_opt_dryrun_max_per_tick_ = 2;

    double subcloud_local_opt_dryrun_min_improvement_m_ = 0.05;
    double subcloud_local_opt_dryrun_min_improvement_ratio_ = 0.20;

    double subcloud_local_opt_dryrun_weight_epsilon_ = 1e-6;

    double subcloud_local_opt_dryrun_max_query_alpha_ = 0.95;
    double subcloud_local_opt_dryrun_max_candidate_alpha_ = 0.95;

    double subcloud_local_opt_dryrun_min_side_alpha_ = 0.05;

    double subcloud_local_opt_dryrun_max_kf_delta_t_warn_m_ = 3.0;
    double subcloud_local_opt_dryrun_max_kf_delta_yaw_warn_deg_ = 35.0;

    bool subcloud_local_opt_dryrun_log_kfs_ = true;
    int subcloud_local_opt_dryrun_max_kf_logs_ = 80;

    std::unordered_map<uint64_t, LocalOptimizationDryRunResult>
        subcloud_local_opt_dryrun_results_;

    uint64_t next_subcloud_local_opt_dryrun_result_id_ = 1;

    // ============================================================
    // FASE 12R-A:
    // Aplicar al servidor los dry-runs locales útiles.
    // - No toca ORB-SLAM3 interno.
    // - No modifica world_T_local rígido de fiduciales directos.
    // - Escribe optimized_kf_poses_.
    // - Reconstruye optimized_world_T_local_by_submap_.
    // ============================================================

    bool subcloud_local_opt_apply_enabled_ = true;
    bool subcloud_local_opt_apply_once_ = true;

    int subcloud_local_opt_apply_max_per_tick_ = 1;

    double subcloud_local_opt_apply_min_improvement_m_ = 0.05;
    double subcloud_local_opt_apply_min_improvement_ratio_ = 0.20;

    double subcloud_local_opt_apply_max_after_mean_residual_m_ = 0.30;

    double subcloud_local_opt_apply_max_kf_delta_t_m_ = 2.0;
    double subcloud_local_opt_apply_max_kf_delta_yaw_deg_ = 25.0;
    double subcloud_local_opt_apply_max_kf_delta_rot_deg_ = 30.0;

    double subcloud_local_opt_apply_min_kf_delta_t_m_ = 0.005;

    bool subcloud_local_opt_apply_publish_immediately_ = true;
    bool subcloud_local_opt_apply_requeue_pending_kfs_ = true;
    bool subcloud_local_opt_apply_clear_pending_region_ = true;

    bool subcloud_local_opt_apply_log_kfs_ = true;
    int subcloud_local_opt_apply_max_kf_logs_ = 80;

    std::unordered_map<uint64_t, LocalOptimizationApplyStats>
        subcloud_local_opt_apply_stats_;

    uint64_t next_subcloud_local_opt_apply_id_ = 1;

    // ============================================================
    // FASE 12R-B:
    // Grafo persistente de restricciones KF-KF.
    // ============================================================

    bool local_opt_constraint_graph_enabled_ = true;
    bool local_opt_constraint_graph_log_edges_ = true;
    bool local_opt_constraint_graph_log_summary_ = true;

    int local_opt_constraint_min_support_pairs_ = 3;
    int local_opt_constraint_max_candidate_kfs_per_event_ = 12;

    double local_opt_constraint_fusion_strength_base_ = 25.0;
    double local_opt_constraint_covisibility_strength_base_ = 8.0;
    double local_opt_constraint_local_apply_strength_base_ = 35.0;
    double local_opt_constraint_fiducial_strength_ = 10000.0;

    // FASE 12R-B2:
    // Priors fiduciales según error contra pose real.
    double local_opt_constraint_soft_fiducial_strength_ = 80.0;
    double local_opt_constraint_fiducial_debt_strength_ = 5.0;

    double local_opt_constraint_strength_max_ = 20000.0;

    double local_opt_constraint_translation_weight_scale_ = 1.0;
    double local_opt_constraint_rotation_weight_scale_ = 1.0;

    double local_opt_constraint_alpha_strength_divisor_ = 35.0;
    double local_opt_constraint_min_alpha_scale_ = 0.03;
    double local_opt_constraint_max_alpha_scale_ = 1.0;

    double local_opt_constraint_pose_prior_weight_scale_ = 1.0;

    // Cap para KFs no fixed. Sin esto, una arista fuerte puede meter
    // pose_prior_weight enorme aunque no deba congelar el nodo.
    double local_opt_constraint_pose_prior_bonus_cap_ = 300.0;

    // Desde 12R-B2 este valor queda solo para diagnóstico.
    // NO debe fijar automáticamente por max_strength general.
    double local_opt_constraint_hard_fix_strength_ = 5000.0;

    bool local_opt_constraint_hard_fix_fiducials_ = true;
    bool local_opt_constraint_skip_hard_fixed_on_apply_ = true;

    // Clasificación del fiducial por error entre pose actual del KF
    // y pose real/medida por el fiducial.
    double local_opt_fiducial_hard_max_error_t_m_ = 0.25;
    double local_opt_fiducial_hard_max_yaw_deg_ = 4.0;
    double local_opt_fiducial_hard_max_rot_deg_ = 6.0;

    double local_opt_fiducial_soft_max_error_t_m_ = 0.80;
    double local_opt_fiducial_soft_max_yaw_deg_ = 15.0;
    double local_opt_fiducial_soft_max_rot_deg_ = 20.0;

    // Si el submapa aún no tiene pose mundo previa, el primer fiducial
    // puede crear ancla inicial porque no hay contradicción medible.
    bool local_opt_fiducial_no_current_pose_as_hard_anchor_ = true;

    // ============================================================
    // FASE 12R-D2:
    // Cuarentena de FIDUCIAL_GLOBAL_DEBT.
    // Una deuda fiducial NO debe entrar al MultiDroneSystem como prior
    // ni disparar force_pose_graph_optimization_.
    // ============================================================

    bool local_opt_fiducial_debt_add_to_multisystem_ = false;
    bool local_opt_fiducial_debt_force_legacy_optimization_ = false;
    bool local_opt_fiducial_debt_count_as_ready_prior_ = false;
    bool local_opt_fiducial_debt_refresh_enabled_ = false;

    int local_opt_fiducial_debt_max_new_per_window_ = 20;

    double local_opt_constraint_post_apply_strength_gain_ = 25.0;
    double local_opt_constraint_post_apply_min_improvement_ratio_ = 0.20;

    // ============================================================
    // FASE 12R-B3:
    // Guards de aplicación real.
    // La optimización local solo puede aplicar si la ventana tiene
    // soporte real del grafo. Esto evita mover KFs fiduciales o zonas
    // buenas cuando constraint_strength=0.
    // ============================================================

    bool local_opt_apply_require_constraint_support_ = true;

    int local_opt_apply_min_constrained_movable_kfs_ = 2;

    double local_opt_apply_min_max_constraint_strength_ = 40.0;

    bool local_opt_apply_reject_all_zero_constraints_ = true;

    bool local_opt_apply_skip_fiducial_direct_without_constraint_ = true;

    // ============================================================
    // FASE 12R-C:
    // Aristas objetivo para optimización local.
    // Estas aristas representan el objetivo "reducir error del loop".
    // No son rigidez de grafo.
    // ============================================================

    bool local_opt_objective_edges_enabled_ = true;
    bool local_opt_objective_register_at_task_creation_ = true;

    double local_opt_objective_strength_base_ = 80.0;
    double local_opt_apply_min_objective_strength_ = 20.0;

    int local_opt_apply_min_supported_movable_kfs_ = 2;
    double local_opt_apply_min_supported_movable_ratio_ = 0.25;

    // ============================================================
    // FASE 12R-D:
    // Optimizador local real de pose graph.
    // Sustituye el reparto por alpha por una minimización local.
    // Variables: incrementos SE3 por KeyFrame variable.
    // Coste: objetivo loop + restricciones KF-KF + priors.
    // ============================================================

    bool local_pose_graph_optimizer_enabled_ = true;
    bool local_pose_graph_log_solver_ = true;

    int local_pose_graph_iterations_ = 6;

    double local_pose_graph_lm_lambda_ = 1e-3;
    double local_pose_graph_numeric_eps_t_m_ = 1e-3;
    double local_pose_graph_numeric_eps_rot_rad_ = 1e-4;

    double local_pose_graph_step_scale_ = 1.0;
    double local_pose_graph_max_step_t_m_ = 0.25;
    double local_pose_graph_max_step_rot_deg_ = 4.0;

    double local_pose_graph_objective_weight_ = 4.0;

    // Convierte strength del grafo a peso de residual.
    // Ejemplo: strength=40, scale=0.02 -> peso^2≈0.8.
    double local_pose_graph_edge_weight_scale_ = 0.02;
    double local_pose_graph_prior_weight_scale_ = 0.02;

    double local_pose_graph_max_edge_weight_ = 15.0;
    double local_pose_graph_max_prior_weight_ = 6.0;

    double local_pose_graph_min_cost_reduction_ratio_ = 0.01;

    // Seguridad extra: si el solver produce deltas más grandes que esto,
    // el dry-run se marcará como no válido antes del apply.
    double local_pose_graph_max_result_delta_t_m_ = 2.0;
    double local_pose_graph_max_result_delta_rot_deg_ = 30.0;

    // FASE 12R-D2/D3:
    // Límites duros para evitar que el solver numérico bloquee el ordenador.
    // En 12R-D3 no rechazamos directamente por exceso de variables;
    // seleccionamos un subconjunto representativo.
    int local_pose_graph_max_variables_ = 12;
    int local_pose_graph_max_objective_matches_ = 40;
    int local_pose_graph_max_graph_edges_ = 80;

    bool local_pose_graph_reject_if_too_many_variables_ = false;

    // ============================================================
    // FASE 12R-D3:
    // Selección distribuida de variables.
    // ============================================================

    bool local_pose_graph_variable_selection_enabled_ = true;
    bool local_pose_graph_log_variable_selection_ = true;

    int local_pose_graph_min_query_variables_ = 2;
    int local_pose_graph_min_candidate_variables_ = 2;
    int local_pose_graph_max_query_variables_ = 8;
    int local_pose_graph_max_candidate_variables_ = 6;

    double local_pose_graph_var_score_objective_support_ = 10000.0;
    double local_pose_graph_var_score_objective_edge_ = 1500.0;
    double local_pose_graph_var_score_inlier_support_ = 80.0;
    double local_pose_graph_var_score_covis_support_ = 8.0;
    double local_pose_graph_var_score_seed_bonus_ = 700.0;
    double local_pose_graph_var_score_query_seed_bonus_ = 1200.0;
    double local_pose_graph_var_score_distance_penalty_ = 0.20;

    // Bonifica candidatos lejos de los ya seleccionados para cubrir el radio.
    double local_pose_graph_var_score_spatial_diversity_ = 180.0;

    // ============================================================
    // FASE 12R-D3:
    // Masa representativa de KFs no seleccionados.
    // ============================================================

    double local_pose_graph_representation_radius_m_ = 2.5;
    double local_pose_graph_represented_prior_weight_scale_ = 0.50;
    double local_pose_graph_represented_prior_bonus_cap_ = 250.0;

    // ============================================================
    // FASE 12R-D3:
    // Propagación barata de delta a KFs no seleccionados.
    // ============================================================

    bool local_pose_graph_propagate_unselected_kfs_ = true;
    bool local_pose_graph_propagate_same_side_only_ = true;

    int local_pose_graph_propagation_max_neighbors_ = 3;

    double local_pose_graph_propagation_radius_m_ = 2.5;
    double local_pose_graph_propagation_scale_ = 0.80;
    double local_pose_graph_propagation_min_weight_sum_ = 1e-6;

    std::unordered_map<
        KeyFramePairKey,
        LocalOptKeyFrameConstraintEdge,
        KeyFramePairKeyHash> local_opt_kf_constraint_edges_;

    // ============================================================
    // FASE 5R:
    // Estado por KeyFrame/zona y cola de loop jobs.
    // Esta fase solo agenda trabajos; no los procesa.
    // ============================================================

    bool subcloud_loop_state_enabled_ = true;
    bool subcloud_loop_job_queue_enabled_ = true;
    bool subcloud_loop_schedule_from_bow_enabled_ = true;

    // Si está true, solo creamos job subcloud si también se ha encolado
    // al menos un BowCandidateJob legacy. Esto evita programar jobs para
    // candidatos ya rechazados/tested/already_queued por la ruta BoW actual.
    bool subcloud_loop_schedule_require_legacy_bow_queued_ = true;

    int subcloud_loop_job_max_queue_ = 200;

    // Umbral barato usando mappoint_ids del KeyFrame.
    // No construye query cloud real todavía.
    int subcloud_loop_state_min_query_mappoints_ = 30;

    double subcloud_loop_cooldown_no_candidate_s_ = 5.0;
    double subcloud_loop_cooldown_no_matches_s_ = 8.0;
    double subcloud_loop_cooldown_bad_geometry_s_ = 10.0;

    // ============================================================
    // FASE 6R.1:
    // Procesamiento presupuestado de la cola subcloud.
    // Solo construye query cloud + candidate subcloud dentro del job.
    // No matching, no RANSAC, no fusión, no optimización.
    // ============================================================

    bool subcloud_loop_job_processing_enabled_ = true;
    bool subcloud_loop_candidate_build_enabled_ = true;

    int subcloud_loop_max_jobs_per_tick_ = 1;

    bool subcloud_loop_drop_stale_jobs_ = true;
    double subcloud_loop_max_job_age_s_ = 20.0;

    // ============================================================
    // FASE 6R2:
    // Candidate subcloud basada en candidatos BoW.
    // Esto evita scan global/frustum cuando ya tenemos semillas BoW.
    // ============================================================

    bool subcloud_bow_seed_candidate_enabled_ = true;

    // Para BOW_REVISIT anclado:
    // primero usar BoW-seed candidate. Si sale demasiado pequeño,
    // permitir fallback a BuildGlobalCandidateSubcloudForQueryKeyFrame.
    bool subcloud_bow_seed_fallback_to_frustum_for_revisit_ = true;

    // Para BOW_ANCHOR_CANDIDATE flotante:
    // NO hacer fallback global/frustum porque la query no tiene world.
    // Solo usar los candidate seed KFs anclados.
    bool subcloud_bow_seed_allow_anchor_candidate_ = true;

    int subcloud_bow_seed_max_seed_kfs_ = 8;
    int subcloud_bow_seed_max_points_per_seed_kf_ = 250;

    std::unordered_map<uint64_t, SubcloudKeyFrameLoopState> subcloud_kf_loop_states_;

    std::deque<SubcloudLoopCheckJob> subcloud_loop_jobs_;

    std::unordered_set<uint64_t> subcloud_loop_job_kfs_set_;

    std::vector<SubcloudValidatedRegion> subcloud_validated_regions_;

    uint64_t next_subcloud_loop_job_id_ = 1;
    uint64_t next_subcloud_validated_region_id_ = 1;

    bool unified_loop_post_opt_eval_enabled_ = true;
    int unified_loop_max_post_opt_logs_ = 80;

    // ============================================================
    // FASE 5:
    // Optimización iterativa controlada por loop.
    // ============================================================

    bool loop_opt_control_enabled_ = true;

    // Máximo de evaluaciones/optimizaciones asociadas a un mismo loop.
    int loop_opt_max_attempts_per_event_ = 3;

    // Mejora mínima absoluta para considerar que merece la pena otro intento.
    double loop_opt_min_abs_improvement_m_ = 0.03;

    // Mejora relativa mínima respecto al error anterior.
    double loop_opt_min_relative_improvement_ = 0.10;

    // No reintentar el mismo loop más rápido que este intervalo.
    double loop_opt_retry_cooldown_s_ = 1.0;

    // Si true, un loop fuerte con error alto puede forzar una optimización
    // aunque no hayan aparecido constraints nuevas.
    bool loop_opt_force_retry_optimization_ = true;

    // Logs de Fase 5.
    bool loop_opt_log_events_ = true;

    // ============================================================
    // FASE 5.1:
    // Histéresis para decisión optimize/fuse.
    // - Por encima del umbral alto: optimizar.
    // - Por debajo del umbral bajo: fusionar.
    // - En la banda intermedia: no seguir optimizando y permitir fusión
    //   solo si el loop es usable_for_fusion; los filtros de pares
    //   distancia/descriptor siguen protegiendo contra fusiones malas.
    // ============================================================

    bool loop_error_hysteresis_enabled_ = true;

    double loop_error_optimize_enter_t_m_ = 0.60;
    double loop_error_optimize_enter_yaw_deg_ = 8.0;

    double loop_error_fusion_exit_t_m_ = 0.40;
    double loop_error_fusion_exit_yaw_deg_ = 5.0;

    bool loop_error_hysteresis_allow_middle_band_fusion_ = true;

    bool loop_error_hysteresis_log_events_ = true;

    // ============================================================
    // FASE 5.3:
    // Evitar reoptimizar sin causa real.
    // ============================================================

    bool optimization_skip_stale_forced_requests_ = true;

    // Si un loop sigue por encima de este error después de optimizar,
    // probablemente es falso positivo, fachada repetida o constraint conflictivo.
    bool loop_opt_conflict_guard_enabled_ = true;

    double loop_opt_conflict_error_t_m_ = 2.0;
    double loop_opt_conflict_error_yaw_deg_ = 20.0;

    int loop_opt_conflict_max_attempts_ = 1;

    // ============================================================
    // FASE 6:
    // Fusión explícita tras error bajo.
    // ============================================================

    bool loop_fusion_score_boost_enabled_ = true;

    double loop_fusion_score_gain_merged_ = 0.10;
    double loop_fusion_score_gain_reobserve_ = 0.05;

    bool loop_fusion_source_track_merge_enabled_ = true;

    bool loop_fusion_log_score_updates_ = true;
    int loop_fusion_max_score_update_logs_per_tick_ = 60;

    // ============================================================
    // FASE 6:
    // Reducir optimizaciones fiduciales demasiado frecuentes.
    // ============================================================

    bool fiducial_batch_new_priors_for_optimization_enabled_ = true;

    int fiducial_min_new_priors_for_optimization_ = 5;

    // ============================================================
    // FASE 6.1:
    // El batch skip de priors fiduciales solo debe aplicarse cuando
    // no existe una ventana fiducial con deuda real de optimización.
    // ============================================================

    bool fiducial_batch_skip_respects_optimization_debt_ = true;

    bool fiducial_debt_log_events_ = true;

    // ============================================================
    // FASE 6.2:
    // No optimizar fiducial si la transformación medida por fiducial
    // ya coincide suficientemente con la transformación world_T_local actual.
    // ============================================================

    bool fiducial_low_error_skip_optimization_enabled_ = true;

    double fiducial_low_error_skip_max_t_m_ = 0.30;
    double fiducial_low_error_skip_max_yaw_deg_ = 5.0;

    // Si true, solo permite este skip cuando ya existe una transform world
    // válida para el submapa. Evita saltarse la primera creación real de anchor.
    bool fiducial_low_error_skip_require_existing_world_ = true;

    bool fiducial_low_error_skip_log_events_ = true;

    // ============================================================
    // FASE 6.2:
    // Rechazar loops conflictivos ANTES de pedir optimización.
    // ============================================================

    bool loop_opt_pre_reject_conflicts_enabled_ = true;

    double loop_opt_pre_reject_error_t_m_ = 2.0;
    double loop_opt_pre_reject_error_yaw_deg_ = 45.0;
    double loop_opt_pre_reject_error_rot_deg_ = 60.0;

    bool loop_opt_pre_reject_log_events_ = true;

    // ============================================================
    // FASE 6.3:
    // Solo entrar al optimizador si existe deuda real de optimización.
    // Evita optimizar por "new_priors=true" o "new_loops=true" cuando
    // esos priors/loops ya están en error bajo o son solo fusionables.
    // ============================================================

    bool optimize_only_with_real_debt_ = true;

    bool advance_processed_counters_on_low_error_skip_ = true;

    bool loop_new_edges_require_optimization_debt_ = true;

    bool fiducial_new_priors_require_optimization_debt_ = true;

    bool real_debt_log_events_ = true;

    // ============================================================
    // FASE A:
    // Separar evento de acción.
    //
    // Objetivo:
    // - new_loop_edge no implica optimización.
    // - new_fiducial_prior no implica optimización.
    // - force_pose_graph_optimization_ no implica optimización automática.
    // - solo una tarea explícita puede activar el optimizador.
    // ============================================================

    bool phase_a_decision_gate_enabled_ = true;

    // En esta fase todavía NO implementamos optimización local.
    // Si aparece LoopDebt alto, se loguea como tarea local pendiente,
    // pero NO se llama al optimizador global.
    bool phase_a_loop_debt_diagnostic_only_ = true;

    // Solo permitimos optimización global si hay deuda fiducial real.
    bool phase_a_global_opt_only_for_fiducial_debt_ = true;

    // Si true, los new_loop_edges sin tarea global se marcan como procesados
    // para que no disparen optimización global por contadores.
    bool phase_a_advance_loop_counters_when_local_task_deferred_ = true;

    // Si true, los priors fiduciales sin deuda real se marcan como procesados.
    bool phase_a_advance_fiducial_counters_when_no_debt_ = true;

    // Busy no debe activarse por force residual ni por evento sin tarea.
    bool phase_a_busy_only_for_explicit_tasks_ = true;

    bool phase_a_log_decisions_ = true;

    // ============================================================
    // FASE B:
    // Diagnóstico de cantidad de loops.
    // No cambia comportamiento. Solo mide el embudo completo:
    // KF -> BoW -> frustum -> geometry -> loop fuerte -> fusión/tarea.
    // ============================================================

    bool phase_b_loop_throughput_enabled_ = true;

    double phase_b_loop_throughput_period_s_ = 2.0;

    bool phase_b_log_kf_funnel_ = true;

    int phase_b_max_kf_funnel_logs_per_period_ = 40;

    LoopThroughputStats phase_b_loop_stats_;

    rclcpp::Time phase_b_last_summary_time_;

    int phase_b_kf_funnel_logs_this_period_ = 0;

    // ============================================================
    // FASE C:
    // Medición explícita del error vista actual ↔ mapa.
    // ============================================================

    bool phase_c_viewmap_error_enabled_ = true;

    bool phase_c_log_viewmap_error_ = true;

    bool phase_c_log_stored_false_viewmap_error_ = true;

    bool phase_c_log_post_opt_viewmap_error_ = true;

    // ============================================================
    // FASE 4:
    // Fusión/refuerzo de tracks guiada por loops ya validados.
    // ============================================================

    bool loop_fusion_reinforcement_enabled_ = true;

    // Número máximo de eventos ready_for_fusion procesados por tick fused.
    int loop_fusion_max_events_per_tick_ = 12;

    // Número máximo de pares MapPoint-MapPoint procesados por evento.
    int loop_fusion_max_pairs_per_event_ = 250;

    // Si true, un evento se procesa una sola vez.
    // Para esta fase lo queremos en true para evitar subir score infinito
    // por el mismo loop en cada publicación.
    bool loop_fusion_process_event_once_ = true;

    // Distancias máximas tras corrección/optimización.
    // No se usan para detectar el loop; solo para fusionar/reforzar cuando
    // el loop ya está validado y ready_for_fusion.
    double loop_fusion_max_distance_intra_m_ = 0.18;
    double loop_fusion_max_distance_cross_epoch_m_ = 0.28;
    double loop_fusion_max_distance_inter_m_ = 0.35;

    // Umbrales descriptor.
    int loop_fusion_max_hamming_intra_ = 75;
    int loop_fusion_max_hamming_cross_epoch_ = 85;
    int loop_fusion_max_hamming_inter_ = 90;

    // Peso extra aplicado a observaciones procedentes de loop.
    double loop_fusion_observation_weight_scale_ = 2.0;

    // Initial score mínimo para observaciones creadas por loop.
    double loop_fusion_initial_score_ = 0.65;

    bool loop_fusion_log_events_ = true;
    bool loop_fusion_log_summary_ = true;
    int loop_fusion_max_sample_logs_ = 40;

    // ============================================================
    // FASE 2.5:
    // Gating de candidatos BoW por frustum/visibilidad.
    // La pose solo limita dónde buscar. La geometría sigue decidiendo
    // si el loop es fuerte.
    // ============================================================

    bool loop_frustum_gating_enabled_ = true;

    // Si true, solo se aplica gating cuando el query KeyFrame tiene pose global.
    // Si no hay pose, se deja BoW global.
    bool loop_frustum_gating_require_query_world_pose_ = true;

    // Si todos los candidatos quedan rechazados por frustum, mantener el BoW
    // original para no perder loops por mala pose/grafo.
    bool loop_frustum_gating_fallback_if_empty_ = true;

    // Profundidad de puntos potencialmente visibles desde la cámara query.
    double loop_frustum_depth_min_m_ = 0.30;
    double loop_frustum_depth_max_m_ = 12.00;

    // Margen de imagen. Positivo hace el frustum más estricto.
    // Para ser permisivo al principio se puede usar -30.0 o 0.0.
    // Empezamos con -20.0 para no perder candidatos por calibración/ruido.
    double loop_frustum_margin_px_ = -20.0;

    // Número mínimo de MapPoints del candidate KeyFrame que deben caer
    // en el frustum del query KeyFrame.
    int loop_frustum_min_visible_mappoints_ = 5;

    // Límite de MapPoints que se revisan por candidate KeyFrame para no disparar coste.
    int loop_frustum_max_mappoint_checks_per_candidate_ = 120;

    // Límite de logs de muestras por tick/llamada.
    int loop_frustum_max_sample_logs_ = 20;

    bool loop_frustum_log_summary_ = true;
    bool loop_frustum_log_samples_ = true;

    void IntraDroneLoopAssistTimerCallback();

    size_t RequeueIntraDroneLoopCandidatesForDrone(
        uint32_t drone_id,
        const DroneMapData& data);

    void ClearTestedPairsForDroneIfRequested(
        uint32_t drone_id);

    rclcpp::TimerBase::SharedPtr intra_drone_loop_assist_timer_;

    bool enable_intra_drone_loop_assist_ = true;
    double intra_drone_loop_assist_period_s_ = 1.0;

    int intra_drone_loop_assist_recent_kfs_ = 12;
    int intra_drone_loop_assist_history_kfs_ = 24;
    int intra_drone_loop_assist_history_stride_ = 5;

    bool intra_drone_loop_assist_only_anchored_ = false;
    bool intra_drone_loop_assist_clear_tested_pairs_ = false;

    int intra_drone_loop_assist_max_requeue_per_drone_ = 40;

    // ============================================================
    // FASE 5D-B:
    // Asociación directa de landmarks dentro del mismo dron.
    // No espera a intra-loop.
    // ============================================================

    rclcpp::TimerBase::SharedPtr direct_landmark_association_timer_;

    bool enable_direct_landmark_association_ = true;
    double direct_landmark_association_period_s_ = 0.5;

    int direct_assoc_recent_kfs_ = 20;
    int direct_assoc_history_kfs_ = 30;
    int direct_assoc_history_stride_ = 4;

    double direct_assoc_revisit_local_distance_m_ = 1.0;
    uint64_t direct_assoc_min_revisit_kf_gap_ = 10;

    int direct_assoc_max_jobs_per_tick_ = 40;
    int direct_assoc_max_queue_size_ = 2000;

    int direct_assoc_min_raw_matches_ = 8;
    int direct_assoc_min_accepted_matches_ = 5;

    double direct_assoc_max_local_mp_distance_m_ = 0.18;
    int direct_assoc_max_descriptor_distance_ = 70;

    bool direct_assoc_use_consecutive_kfs_ = true;
    bool direct_assoc_use_revisit_kfs_ = true;

    std::deque<DirectLandmarkAssociationJob> direct_assoc_queue_;

    std::unordered_set<
        KeyFramePairKey,
        KeyFramePairKeyHash>
        direct_assoc_queued_pairs_;

    std::unordered_set<
        KeyFramePairKey,
        KeyFramePairKeyHash>
        direct_assoc_processed_pairs_;

    void DirectLandmarkAssociationTimerCallback();

    void QueueDirectLandmarkAssociationsForDrone(
        uint32_t drone_id,
        const DroneMapData& data);

    bool EnqueueDirectLandmarkAssociationPair(
        uint64_t query_global_kf_id,
        uint64_t candidate_global_kf_id,
        const std::string& reason);

    void ProcessDirectLandmarkAssociationQueue();

    bool ProcessDirectLandmarkAssociationJob(
        const DirectLandmarkAssociationJob& job);

    // ============================================================
    // Fase C:
    // Asociación directa inter-dron de MapPoints ya anclados.
    // ============================================================

    bool enable_inter_drone_direct_landmark_association_ = true;

    double inter_direct_assoc_period_s_ = 1.0;

    double inter_direct_assoc_max_world_distance_m_ = 0.12;
    int inter_direct_assoc_max_descriptor_distance_ = 60;

    uint32_t inter_direct_assoc_min_observations_ = 2;
    double inter_direct_assoc_min_found_ratio_ = 0.25;

    int inter_direct_assoc_max_mps_per_submap_ = 1500;
    int inter_direct_assoc_max_jobs_per_tick_ = 120;
    int inter_direct_assoc_max_queue_size_ = 5000;
    int inter_direct_assoc_max_new_pairs_per_tick_ = 500;
    int inter_direct_assoc_max_sample_logs_ = 40;

    rclcpp::TimerBase::SharedPtr inter_direct_landmark_association_timer_;

    std::deque<InterDirectLandmarkAssociationJob> inter_direct_assoc_queue_;

    std::unordered_set<
        KeyFramePairKey,
        KeyFramePairKeyHash>
        inter_direct_assoc_queued_pairs_;

    std::unordered_set<
        KeyFramePairKey,
        KeyFramePairKeyHash>
        inter_direct_assoc_processed_pairs_;

    void InterDroneDirectLandmarkAssociationTimerCallback();

    void QueueInterDroneDirectLandmarkAssociations();

    void ProcessInterDroneDirectLandmarkAssociationQueue();

    bool ProcessInterDroneDirectLandmarkAssociationJob(
        const InterDirectLandmarkAssociationJob& job);

    uint64_t PickRepresentativeKeyFrameForMapPoint(
        const orbslam3_multi::ImportedMapPoint& mp) const;

    // ============================================================
    // FASE 5E-light:
    // Limpieza no destructiva de /global_sparse_map_fused.
    // Solo afecta a la nube publicada, no borra MapPoints del atlas.
    // ============================================================

    bool fused_cloud_enable_publication_filter_ = true;

    // ============================================================
    // HOTFIX-FUSED-OFF:
    // Permite desactivar COMPLETAMENTE el cálculo y publicación de
    // /global_sparse_map_fused.
    //
    // Importante:
    // - No solo apaga el publish.
    // - Evita llamar a BuildFusedLandmarksFromLoopEdges().
    // - Evita el coste grande que bloquea el ordenador.
    // ============================================================
    bool enable_fused_cloud_publication_ = true;

    // Fase 2 estable:
    // Evita reconstruir /global_sparse_map_fused en cada tick.
    // Si hay cache reciente, se republica la nube anterior.
    double fused_cloud_min_recompute_period_s_ = 8.0;
    bool has_fused_cloud_cache_ = false;
    rclcpp::Time last_fused_cloud_build_time_;
    std::vector<GlobalSparsePoint> fused_cloud_cached_points_;

    // ============================================================
    // FASE 1:
    // Modo nuevo de publicación fused.
    //
    // fused_cloud_mode_:
    //   "legacy" -> usa PublishFusedLandmarkCloud() antigua.
    //   "simple" -> usa PublishFusedLandmarkCloudSimple().
    //
    // La ruta simple NO usa la caché legacy como semántica principal.
    // Recalcula de forma ligera cuando el publish timer llama a fused.
    // ============================================================

    std::string fused_cloud_mode_ = "simple";

    double fused_simple_period_s_ = 2.0;

    // Radio legacy/base. Se mantiene por compatibilidad y como fallback.
    double fused_simple_merge_radius_m_ = 0.12;
    double fused_simple_max_group_spread_m_ = 0.30;
    int fused_simple_max_hamming_ = 70;

    // ============================================================
    // FASE 2A:
    // Umbrales diferenciados para fusionar landmarks.
    //
    // intra:
    //   mismo dron y mismo epoch. Debe ser conservador.
    //
    // cross_epoch:
    //   mismo dron pero distinto map_epoch. Algo más permisivo.
    //
    // inter:
    //   distinto dron. Más permisivo porque el error residual entre
    //   submapas puede ser mayor incluso con anchor correcto.
    // ============================================================

    double fused_simple_merge_radius_intra_m_ = 0.10;
    double fused_simple_merge_radius_cross_epoch_m_ = 0.16;
    double fused_simple_merge_radius_inter_m_ = 0.28;

    double fused_simple_max_group_spread_intra_m_ = 0.18;
    double fused_simple_max_group_spread_cross_epoch_m_ = 0.25;
    double fused_simple_max_group_spread_inter_m_ = 0.35;

    int fused_simple_max_hamming_intra_ = 65;
    int fused_simple_max_hamming_cross_epoch_ = 75;
    int fused_simple_max_hamming_inter_ = 80;

    // ============================================================
    // FASE 2B:
    // Supresión de singletons redundantes.
    //
    // Problema observado:
    //   - accepted_inter > 0, por tanto hay fusión inter-dron.
    //   - pero sobreviven muchos singleton_groups.
    //   - visualmente aparecen puntos ruidosos del dron 2 alrededor
    //     de zonas que ya tienen grupos fusionados.
    //
    // Política:
    //   - conservar singletons en zonas nuevas;
    //   - eliminar singletons cercanos a grupos merged/inter-merged
    //     si descriptor y distancia son compatibles.
    // ============================================================

    bool fused_simple_suppress_singletons_near_merged_ = true;

    double fused_simple_singleton_suppression_radius_m_ = 0.18;

    int fused_simple_singleton_suppression_max_hamming_ = 85;

    bool fused_simple_singleton_suppression_require_inter_merged_ = true;

     // ============================================================
    // FASE TRACKS-1:
    // Base persistente de landmarks fused con score.
    //
    // Si enabled=true, PublishFusedLandmarkCloudSimple() usa los grupos
    // simples como observaciones y actualiza tracks persistentes.
    // El mapa publicado deja de ser una nube reconstruida por tick y pasa
    // a ser un conjunto de landmarks con memoria temporal.
    // ============================================================

    bool fused_tracks_enabled_ = true;

    double fused_track_publish_score_threshold_ = 0.55;
    double fused_track_initial_score_singleton_ = 0.08;
    double fused_track_initial_score_merged_ = 0.42;

    double fused_track_seen_gain_ = 0.08;
    double fused_track_merged_bonus_ = 0.08;
    double fused_track_inter_drone_bonus_ = 0.10;
    double fused_track_confirmed_bonus_ = 0.08;
    double fused_track_provisional_penalty_ = 0.08;

    double fused_track_assoc_radius_intra_m_ = 0.12;
    double fused_track_assoc_radius_cross_epoch_m_ = 0.18;
    double fused_track_assoc_radius_inter_m_ = 0.30;

    int fused_track_assoc_hamming_intra_ = 70;
    int fused_track_assoc_hamming_cross_epoch_ = 80;
    int fused_track_assoc_hamming_inter_ = 90;

    double fused_track_max_update_step_weak_m_ = 0.12;
    double fused_track_max_update_step_strong_m_ = 0.04;
    double fused_track_strong_score_threshold_ = 0.70;

    size_t fused_track_max_descriptor_samples_ = 40;
    size_t fused_track_max_tracks_ = 20000;

    // TRACKS-1B:
    // Control de singletons persistentes.
    bool fused_track_use_singleton_observations_ = true;

    uint32_t fused_track_singleton_min_observations_ = 10;
    double fused_track_singleton_min_found_ratio_ = 0.55;

    // Un track sin soporte merged necesita verse varias veces antes de publicarse.
    uint32_t fused_track_singleton_min_reobservations_to_publish_ = 3;

    // Tamaño de celda para índice espacial de tracks.
    // Debe ser aproximadamente >= radio máximo de asociación.
    double fused_track_spatial_cell_size_m_ = 0.30;

    // ============================================================
    // TRACKS-PERF1A:
    // Control de carga por recompute.
    //
    // Objetivo:
    //   - no procesar todos los MapPoints históricos en cada tick;
    //   - priorizar candidatos buenos;
    //   - mantener publicación global de tracks persistentes;
    //   - conservar fused_cloud_min_recompute_period_s=4.0 para estabilidad.
    //
    // Esta fase NO cambia la semántica de anchors ni pose graph.
    // ============================================================

    bool fused_tracks_perf1_enabled_ = true;

    // Cap de candidatos antes del merge simple.
    // Reduce merge_ms.
    size_t fused_tracks_perf1_max_candidates_per_submap_ = 4500;
    size_t fused_tracks_perf1_max_total_candidates_ = 9000;

    // Cap de observaciones de tracks después del merge simple.
    // Reduce tracks_ms.
    size_t fused_tracks_perf1_max_observations_per_tick_ = 2000;

    // Si true, se ordenan candidatos/observaciones por calidad antes de capar.
    bool fused_tracks_perf1_prioritize_by_quality_ = true;

    // ============================================================
    // TRACKS-PERF1B:
    // Selección de candidatos por zona activa.
    //
    // Objetivo:
    //   - no reprocesar todo el edificio en cada recompute;
    //   - priorizar MapPoints cerca de los drones activos;
    //   - mantener publicación global de tracks ya existentes;
    //   - reducir coste de merge_ms/tracks_ms cuando el mapa crezca.
    //
    // Esta fase NO borra tracks y NO afecta a anchors/pose graph.
    // Solo decide cuántas observaciones nuevas entran al merge.
    // ============================================================

    bool fused_tracks_perf1b_active_zone_enabled_ = true;

    // Radio alrededor de cada dron activo desde el que se priorizan candidatos.
    double fused_tracks_perf1b_active_radius_m_ = 5.0;

    // Edad máxima de pose GT/local para considerarla centro activo.
    double fused_tracks_perf1b_max_pose_age_s_ = 3.0;

    // En Gazebo usamos GT como centro activo preferente.
    bool fused_tracks_perf1b_use_gt_pose_ = true;

    // Fallback a pose ORB local transformada a world si no hay GT reciente.
    bool fused_tracks_perf1b_use_orb_pose_fallback_ = true;

    // Caps por submapa dentro/fuera de zona activa.
    size_t fused_tracks_perf1b_max_active_candidates_per_submap_ = 3500;
    size_t fused_tracks_perf1b_max_background_candidates_per_submap_ = 600;

    // Cap total tras selección activa. PERF1A todavía puede aplicar su cap después.
    size_t fused_tracks_perf1b_max_total_candidates_ = 7500;

    // ============================================================
    // TRACKS-PERF1C-safe:
    // Filtro de artefactos cerca de la trayectoria/cuerpo del dron.
    //
    // Objetivo:
    //   - eliminar puntos espurios cerca de la trayectoria;
    //   - evitar puntos creados por auto-observación/dinámica;
    //   - no tocar landmarks estructurales ya fusionados;
    //   - no afectar anchors ni pose graph.
    // ============================================================

    bool fused_tracks_perf1c_trajectory_filter_enabled_ = true;

    // Radio XY alrededor de la trayectoria reciente.
    double fused_tracks_perf1c_trajectory_radius_xy_m_ = 0.35;

    // Tolerancia vertical. Evita eliminar paredes altas cercanas en XY
    // si no están realmente cerca de la altura del dron.
    double fused_tracks_perf1c_trajectory_radius_z_m_ = 0.45;

    // Cuánto historial de trayectoria guardar por dron.
    double fused_tracks_perf1c_trajectory_max_age_s_ = 12.0;

    // Separación mínima entre muestras guardadas.
    double fused_tracks_perf1c_trajectory_min_separation_m_ = 0.20;

    // Máximo de muestras por dron.
    size_t fused_tracks_perf1c_trajectory_max_points_per_drone_ = 80;

    // Conservador: solo filtrar candidatos débiles.
    bool fused_tracks_perf1c_filter_only_low_quality_ = true;

    uint32_t fused_tracks_perf1c_low_quality_max_observations_ = 12;
    double fused_tracks_perf1c_low_quality_max_found_ratio_ = 0.65;

    // ============================================================
    // ORB-TRAJ2 / FREE-SPACE1:
    // Filtro de puntos flotantes delante de la cámara corregida ORB.
    //
    // Objetivo:
    //   - eliminar puntos que aparecen entre el dron y la pared;
    //   - no usar ground truth;
    //   - no tocar anchors ni pose graph;
    //   - aplicar solo a candidatos débiles.
    // ============================================================

    bool fused_tracks_free_space_filter_enabled_ = true;

    // Profundidad mínima/máxima delante de cámara para considerar
    // que un candidato está en zona sospechosa de free-space.
    double fused_tracks_free_space_min_depth_m_ = 0.25;
    double fused_tracks_free_space_max_depth_m_ = 2.50;

    // Radio lateral alrededor del eje óptico.
    // Se usa:
    //   radius = base_radius + radius_per_depth * depth
    double fused_tracks_free_space_base_radius_m_ = 0.25;
    double fused_tracks_free_space_radius_per_depth_ = 0.12;

    // Edad máxima de una pose ORB corregida para usarla como referencia.
    double fused_tracks_free_space_max_pose_age_s_ = 8.0;

    // Solo filtrar candidatos débiles.
    bool fused_tracks_free_space_only_low_quality_ = true;

    uint32_t fused_tracks_free_space_low_quality_max_observations_ = 15;
    double fused_tracks_free_space_low_quality_max_found_ratio_ = 0.70;

    // ============================================================
    // TRACKS-STAB1:
    // Evitar que observaciones singleton arrastren landmarks fuertes.
    //
    // Los singletons son útiles para zonas nuevas, pero no deben mover
    // mucho landmarks ya merged/inter/fuertes.
    // ============================================================

    bool fused_tracks_stab_freeze_singleton_geometry_on_strong_tracks_ = true;

    double fused_tracks_stab_strong_track_score_ = 0.65;

    double fused_tracks_stab_singleton_weight_scale_ = 0.10;

    double fused_tracks_stab_singleton_max_update_step_m_ = 0.005;

    // ============================================================
    // TRACKS-MERGE1 / PERF1C-safe:
    // Consolidación track-to-track.
    //
    // Objetivo:
    //   - eliminar dobles paredes;
    //   - fusionar tracks persistentes duplicados;
    //   - reducir crecimiento del track store;
    //   - no tocar anchors ni pose graph.
    //
    // Se ejecuta después de actualizar tracks y antes de publicar.
    // ============================================================

    bool fused_tracks_merge_enabled_ = true;

    // Radios pequeños. No queremos colapsar toda una pared, solo duplicados.
    double fused_tracks_merge_radius_intra_m_ = 0.040;
    double fused_tracks_merge_radius_cross_epoch_m_ = 0.070;
    double fused_tracks_merge_radius_inter_m_ = 0.120;

    int fused_tracks_merge_hamming_intra_ = 55;
    int fused_tracks_merge_hamming_cross_epoch_ = 75;
    int fused_tracks_merge_hamming_inter_ = 90;

    // Solo consolidamos tracks con score mínimo razonable.
    double fused_tracks_merge_min_score_ = 0.20;

    // Límite de seguridad por recompute.
    size_t fused_tracks_merge_max_merges_per_tick_ = 300;

    // Tamaño de celda para buscar tracks vecinos.
    double fused_tracks_merge_cell_size_m_ = 0.12;

    // ============================================================
    // TRACKS2A:
    // Score negativo básico.
    //
    // No es todavía expected-visible completo por proyección a cámara.
    // Es una versión segura:
    //   - tracks no observados bajan score suavemente;
    //   - tracks cerca de observaciones actuales pero no asociados
    //     bajan más;
    //   - tracks débiles con score muy bajo se eliminan.
    // ============================================================

    bool fused_track_score_decay_enabled_ = true;

    // Decaimiento suave por tick para tracks no actualizados.
    double fused_track_stale_decay_merged_ = 0.015;
    double fused_track_stale_decay_singleton_ = 0.040;

    // Edad mínima desde última observación para aplicar decay.
    double fused_track_stale_decay_min_age_s_ = 3.0;

    // Proxy de expected-but-missed:
    // si hay observaciones actuales cerca de un track y el track no fue actualizado,
    // se considera missed.
    bool fused_track_expected_miss_proxy_enabled_ = true;
    double fused_track_expected_miss_radius_m_ = 0.35;
    int fused_track_expected_miss_min_nearby_observations_ = 3;
    double fused_track_expected_miss_penalty_ = 0.08;

    // Eliminación de tracks débiles.
    bool fused_track_remove_low_score_tracks_ = true;
    double fused_track_remove_score_threshold_ = 0.08;
    double fused_track_remove_min_age_s_ = 8.0;

    // Publicación conservadora de tracks jóvenes.
    uint32_t fused_track_merged_min_reobservations_to_publish_ = 2;
    uint32_t fused_track_inter_min_reobservations_to_publish_ = 1;

    // ============================================================
    // TRACKS2B:
    // Expected-visible real por proyección a KeyFrames recientes.
    //
    // En TRACKS2B-A se recomienda diag_only=true:
    //   - calcula cuántos tracks deberían ser visibles;
    //   - escribe logs;
    //   - NO modifica score.
    //
    // En TRACKS2B-B, si el diagnóstico tiene sentido:
    //   - diag_only=false;
    //   - los tracks expected-visible no actualizados bajan score.
    // ============================================================

    bool fused_track_expected_visible_enabled_ = true;
    bool fused_track_expected_visible_diag_only_ = true;

    int fused_track_expected_visible_max_kfs_per_drone_ = 8;
    int fused_track_expected_visible_min_kfs_ = 2;

    double fused_track_expected_visible_min_depth_m_ = 0.20;
    double fused_track_expected_visible_max_depth_m_ = 18.0;
    double fused_track_expected_visible_margin_px_ = 25.0;

    double fused_track_expected_visible_penalty_ = 0.04;

    // Solo analizamos tracks razonablemente vivos para no gastar tiempo
    // en basura de score casi cero.
    double fused_track_expected_visible_min_score_ = 0.20;

    // Protección de coste.
    size_t fused_track_expected_visible_max_tracks_per_tick_ = 1500;

    // Memoria persistente de landmarks fused.
    std::unordered_map<uint64_t, FusedLandmarkTrack> fused_tracks_;
    uint64_t next_fused_track_id_ = 1;

    uint32_t fused_simple_min_observations_ = 2;
    double fused_simple_min_found_ratio_ = 0.20;

    // Fase 1A:
    // Los candidatos pueden entrar con filtros suaves para permitir merges,
    // pero los singletons publicados deben ser más estrictos.
    uint32_t fused_simple_singleton_min_observations_ = 5;
    double fused_simple_singleton_min_found_ratio_ = 0.30;

    // Fase 1A.2:
    // Filtro de densidad más estricto para limpiar ruido aislado.
    bool fused_simple_density_filter_enabled_ = true;
    double fused_simple_density_radius_m_ = 0.28;
    int fused_simple_density_min_neighbors_ = 4;

    bool fused_simple_keep_merged_without_density_ = true;

    // Fase 1A.2:
    // Desactivado de momento. En la prueba anterior dejó pasar muchos
    // puntos aislados por ser "HQ" aunque no tuvieran densidad local.
    bool fused_simple_keep_high_quality_singletons_without_density_ = false;
    uint32_t fused_simple_hq_singleton_min_observations_ = 10;
    double fused_simple_hq_singleton_min_found_ratio_ = 0.50;

    // ============================================================
    // FASE 1B:
    // Publicación visual de submapas LOOP_OR_PROPAGATED + PROVISIONAL.
    //
    // Importante:
    // - Esto solo afecta a /global_sparse_map_fused.
    // - NO implica publicar MapCorrection definitiva.
    // - NO implica usar el anchor provisional en pose graph estable.
    // ============================================================

    bool fused_simple_publish_loop_provisional_ = true;

    // Filtro de entrada más estricto para puntos de submapas provisionales.
    uint32_t fused_simple_provisional_min_observations_ = 3;
    double fused_simple_provisional_min_found_ratio_ = 0.25;

    // Filtro de singletons todavía más estricto para provisionales.
    uint32_t fused_simple_provisional_singleton_min_observations_ = 7;
    double fused_simple_provisional_singleton_min_found_ratio_ = 0.40;

    // Densidad más estricta para singletons provisionales.
    int fused_simple_provisional_density_min_neighbors_ = 6;

    // Límite de seguridad para que un submapa provisional no inunde RViz2.
    size_t fused_simple_max_provisional_candidates_per_submap_ = 1200;

    // Logs explícitos de la política anchor -> fused / correction / graph.
    bool phase1b_anchor_policy_logs_enabled_ = true;

    bool fused_simple_voxel_enabled_ = true;
    double fused_simple_voxel_size_m_ = 0.06;

    size_t fused_simple_max_points_ = 3000;

    bool fused_simple_log_per_submap_ = true;

    rclcpp::Time last_fused_simple_build_time_;

    // Filtrado de singletons por calidad.
    bool fused_cloud_filter_singletons_by_quality_ = true;
    uint32_t fused_cloud_singleton_min_observations_ = 2;
    double fused_cloud_singleton_min_found_ratio_ = 0.20;

    // Filtrado de singletons aislados por densidad local.
    bool fused_cloud_filter_isolated_singletons_ = true;
    double fused_cloud_isolated_radius_m_ = 0.20;
    int fused_cloud_isolated_min_neighbors_ = 2;

    // ============================================================
    // Fase H:
    // Permitir singletons MUY estables aunque no tengan vecinos.
    // Esto evita que /global_sparse_map_fused se quede demasiado pobre
    // cuando los puntos son buenos pero sparse.
    // ============================================================

    bool fused_keep_high_quality_singletons_without_density_ = true;

    uint32_t fused_high_quality_singleton_min_observations_ = 6;
    uint32_t fused_high_quality_singleton_min_effective_kfs_ = 4;
    double fused_high_quality_singleton_min_found_ratio_ = 0.35;
    double fused_high_quality_singleton_min_score_ = 1.60;

    // Si true, los landmarks fusionados no se eliminan por el filtro de densidad.
    // Recomendado: true.
    bool fused_cloud_always_keep_merged_landmarks_ = true;

    // Si no hay ningún grupo fusionado todavía, mantenemos singletons.
    // Así no vaciamos la nube durante el arranque.
    bool fused_cloud_keep_singletons_if_no_merged_ = true;

    // Fase 2: nueva semántica de /global_sparse_map_fused.
    // Si está activo, el tópico fused solo publica landmarks confirmados,
    // es decir, grupos con al menos N MapPoints fuente.
    bool fused_cloud_publish_only_confirmed_ = true;

    // Número mínimo de MapPoints fuente para considerar confirmado un landmark.
    // Para Fase 2 usamos 2: cualquier singleton queda fuera.
    size_t fused_cloud_min_confirmed_sources_ = 2;

    // Si true, los grupos que fueron rechazados por spread y partidos a singletons
    // tampoco se publican en /global_sparse_map_fused.
    bool fused_cloud_reject_split_singletons_in_confirmed_mode_ = true;

    // Fase 3: score de confirmación para landmarks fusionados.
    // Sigue siendo un filtro de publicación: no borra nada del atlas.
    bool fused_cloud_use_confirmation_score_ = true;

    double fused_confirmed_min_score_ = 2.0;
    double fused_confirmed_max_group_spread_m_ = 0.30;

    uint32_t fused_confirmed_min_match_count_ = 1;
    uint32_t fused_confirmed_min_observing_kfs_ = 2;
    uint32_t fused_confirmed_min_total_observations_ = 2;

    // ============================================================
    // Fase 3B:
    // Reintroducción conservadora de singletons estables.
    //
    // Un singleton puede ser un MapPoint bueno de ORB-SLAM3 si ya fue
    // observado desde varios KeyFrames dentro del SLAM local. No necesita
    // estar fusionado con otro MapPoint para ser un landmark confirmado.
    // ============================================================

    bool fused_cloud_allow_stable_singletons_ = true;

    // Evidencia mínima local de ORB-SLAM3.
    uint32_t fused_stable_singleton_min_observations_ = 5;
    uint32_t fused_stable_singleton_min_effective_observing_kfs_ = 4;
    double fused_stable_singleton_min_found_ratio_ = 0.35;

    // Score opcional para singletons. Debe ser menos estricto que para merged,
    // porque confirmed_match_count será 0 para un singleton.
    bool fused_stable_singleton_use_score_ = true;
    double fused_stable_singleton_min_score_ = 1.50;

    // Si true, los singletons estables pasan después por el filtro espacial
    // de aislamiento ya existente.
    bool fused_stable_singleton_require_density_ = true;

    // Logs.
    bool fused_stable_singleton_log_samples_ = true;
    int fused_stable_singleton_max_sample_logs_ = 30;

    // ============================================================
    // Fase 4:
    // Evidencia negativa por expected/seen/missed views.
    // Primera versión: diagnóstico y score suave.
    // ============================================================

    bool fused_confirmed_enable_expected_view_score_ = true;

    // Primera prueba: no rechazar, solo medir y loguear.
    // Cuando validemos métricas, se podrá poner a false para aplicar rechazo.
    bool fused_confirmed_expected_view_diagnostic_only_ = true;

    // ============================================================
    // Fase 5:
    // Separación entre mapa fused final y mapa fused debug.
    //
    // /global_sparse_map_fused:
    //   mapa final filtrado.
    //
    // /global_sparse_map_fused_debug:
    //   nube de diagnóstico para ver qué se está eliminando.
    // ============================================================

    bool publish_fused_debug_cloud_ = true;

    // Opciones:
    //   "all_valid":
    //       publica todos los FusedLandmark válidos devueltos por
    //       BuildFusedLandmarksFromLoopEdges(), antes de confirmed/stable/density.
    //
    //   "confirmed_candidates":
    //       publica landmarks que pasan confirmed/stable, antes de densidad.
    //
    //   "pre_voxel":
    //       publica landmarks que pasan densidad, antes de voxel.
    //
    // Recomendación inicial: "all_valid".
    std::string fused_debug_cloud_mode_ = "all_valid";

    bool fused_debug_cloud_apply_voxel_filter_ = true;
    double fused_debug_cloud_voxel_size_m_ = 0.04;

    size_t fused_debug_cloud_max_points_ = 50000;

    bool fused_debug_cloud_log_summary_ = true;

    // ============================================================
    // Fase 8:
    // Política de publicación multi-dron.
    // Cuando hay al menos N drones anclados, /global_sparse_map_fused
    // debe priorizar landmarks con soporte inter-dron y endurecer singletons.
    // ============================================================

    bool fused_cloud_enable_multi_drone_policy_ = true;

    // A partir de cuántos drones anclados se activa la política multi-dron.
    int fused_cloud_multi_drone_min_anchored_drones_ = 2;

    // Si true, los singletons usan umbrales más duros cuando hay varios drones.
    bool fused_cloud_multi_drone_strict_singletons_ = true;

    uint32_t fused_multi_singleton_min_observations_ = 7;
    uint32_t fused_multi_singleton_min_effective_kfs_ = 5;
    double fused_multi_singleton_min_found_ratio_ = 0.40;
    double fused_multi_singleton_min_score_ = 1.80;

    // Después del filtro de densidad, limita cuántos singletons de un solo dron
    // pueden llegar al voxel final. Los merged y los inter-supported no se capan.
    int fused_multi_max_singletons_per_drone_after_density_ = 900;

    // Si true, los landmarks con soporte inter-dron se mantienen siempre que hayan
    // pasado score/spread, aunque la política de singletons sea dura.
    bool fused_multi_keep_inter_supported_landmarks_ = true;

    bool fused_multi_log_policy_samples_ = true;
    int fused_multi_max_policy_sample_logs_ = 30;

    // ============================================================
    // Fase 6:
    // Validación intra-dron con un solo dron.
    // No cambia la lógica de fusión por defecto: solo añade diagnóstico.
    // ============================================================

    bool fusion6_enable_intra_validation_logs_ = true;
    double fusion6_intra_summary_period_s_ = 1.0;

    // Queremos distinguir si DIRECT5D está aportando merges por
    // revisita real o por KFs consecutivos.
    bool fusion6_log_direct_reason_breakdown_ = true;

    // Logs de muestras de pares DIRECT5D aceptados/rechazados.
    bool fusion6_log_direct_pair_samples_ = true;
    int fusion6_max_direct_pair_sample_logs_ = 20;

    // Logs de loops intra opt-only, es decir, loops buenos para optimización
    // pero no suficientemente buenos para fusión.
    bool fusion6_log_intra_loop_opt_only_samples_ = true;
    int fusion6_max_intra_loop_sample_logs_ = 20;

    double fused_confirmed_expected_view_radius_m_ = 4.0;
    double fused_confirmed_projection_margin_px_ = 20.0;

    double fused_confirmed_min_projection_depth_m_ = 0.10;
    double fused_confirmed_max_projection_depth_m_ = 20.0;

    uint32_t fused_confirmed_expected_view_max_kfs_ = 80;

    double fused_confirmed_seen_view_weight_ = 0.35;
    double fused_confirmed_missed_view_weight_ = 0.15;

    uint32_t fused_confirmed_max_expected_misses_ = 8;
    uint32_t fused_confirmed_min_expected_views_to_reject_ = 4;

    bool fused_confirmed_reject_if_too_many_misses_ = false;

    bool fused_confirmed_log_expected_samples_ = true;
    int fused_confirmed_max_expected_sample_logs_ = 30;

    bool fused_confirmed_log_score_samples_ = true;
    int fused_confirmed_max_score_sample_logs_ = 30;

    // Voxel final opcional para reducir duplicados visuales.
    // Recomendado: activado con tamaño pequeño.
    bool fused_cloud_enable_voxel_filter_ = true;
    double fused_cloud_voxel_size_m_ = 0.04;

    // Logs de diagnóstico.
    bool fused_cloud_filter_log_samples_ = true;
    int fused_cloud_filter_max_sample_logs_ = 30;

    // ============================================================
    // Fase 2:
    // Profiling real de /global_sparse_map_fused.
    // Queremos saber si el cuello de botella está en:
    // - BuildFusedLandmarksFromLoopEdges()
    // - filtrado de candidatos
    // - filtro de densidad
    // - voxel
    // - BuildPointCloud()
    // - publish()
    // ============================================================

    bool fused_cloud_perf_logs_ = true;
    double fused_cloud_perf_warn_total_ms_ = 250.0;
    double fused_cloud_perf_warn_build_ms_ = 120.0;

    // Log agregado por razones de publicación/rechazo.
    // No imprime un log por punto, solo contadores por tick.
    bool fused_cloud_reason_summary_logs_ = true;

    // Permite que /global_sparse_map_fused use puntos de submapas
    // LOOP_PROVISIONAL si el anchor existe y pasó guard/consenso.
    // En Fase 2 lo mantenemos, porque en tu prueba ya acelera que drone_2 aparezca.
    bool fused_publish_loop_provisional_landmarks_ = true;
};

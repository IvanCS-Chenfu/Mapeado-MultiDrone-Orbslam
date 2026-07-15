# LEGACY F1B: copia congelada del launch monolitico anterior.
# No es el launch activo de la subfase 1B.
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue


def launch_setup(context, *args, **kwargs):
    n_drones = int(LaunchConfiguration('n_drones').perform(context))
    namespace_base = LaunchConfiguration('namespace_base').perform(context)

    actions = []

    actions.append(
        Node(
            package='orbslam3_server',
            executable='global_map_server',
            name='global_orb_map_server',
            output='screen',
            parameters=[
                {
                    'use_sim_time': ParameterValue(True, value_type=bool),

                    # Sistema global
                    'world_frame': ParameterValue('world', value_type=str),
                    'n_drones': ParameterValue(n_drones, value_type=int),
                    'namespace_base': ParameterValue(namespace_base, value_type=str),

                    # Sincronización de mapas
                    'full_snapshot_period_s': ParameterValue(120.0, value_type=float),
                    'global_publish_period_s': ParameterValue(2.0, value_type=float),
                    'bow_timer_period_s': ParameterValue(0.25, value_type=float),
                    'fiducial_check_period_s': ParameterValue(0.2, value_type=float),
                    'run_diagnostics_period_s': ParameterValue(2.0, value_type=float),
                    'subcloud_pipeline_diagnostics_enabled': ParameterValue(True, value_type=bool),
                    'subcloud_pipeline_diag_throttle_ms': ParameterValue(2000, value_type=int),

                    # Fase 4R:
                    # Las herramientas query/candidate/match existen, pero no se ejecutan
                    # automáticamente al llegar cada KeyFrame.
                    'subcloud_tool_diagnostics_enabled': ParameterValue(True, value_type=bool),
                    'subcloud_kf_ingest_diagnostics_enabled': ParameterValue(True, value_type=bool),
                    'subcloud_kf_ingest_throttle_ms': ParameterValue(2000, value_type=int),

                    'subcloud_query_cloud_diagnostics_enabled': ParameterValue(False, value_type=bool),
                    'subcloud_query_cloud_min_valid_points_warn': ParameterValue(15, value_type=int),

                    'subcloud_candidate_cloud_diagnostics_enabled': ParameterValue(False, value_type=bool),
                    'subcloud_candidate_cloud_min_points_warn': ParameterValue(30, value_type=int),
                    'subcloud_candidate_frustum_min_points': ParameterValue(30, value_type=int),
                    'subcloud_candidate_score_threshold': ParameterValue(0.60, value_type=float),
                    'subcloud_candidate_supported_score_threshold': ParameterValue(0.20, value_type=float),
                    'subcloud_candidate_include_supported_unpublished': ParameterValue(True, value_type=bool),
                    'subcloud_candidate_radius_m': ParameterValue(3.0, value_type=float),
                    'subcloud_candidate_max_points': ParameterValue(600, value_type=int),
                    'subcloud_candidate_frustum_min_depth_m': ParameterValue(0.30, value_type=float),
                    'subcloud_candidate_frustum_max_depth_m': ParameterValue(20.0, value_type=float),
                    'subcloud_candidate_frustum_margin_px': ParameterValue(20.0, value_type=float),

                    # Fase 6: matching query cloud ↔ candidate global subcloud.
                    # Diagnóstico only: no fusiona, no optimiza, no ancla.
                    'subcloud_match_diagnostics_enabled': ParameterValue(True, value_type=bool),
                    'subcloud_match_every_n_keyframes': ParameterValue(1, value_type=int),
                    'subcloud_match_min_query_points': ParameterValue(30, value_type=int),
                    'subcloud_match_min_candidate_points': ParameterValue(20, value_type=int),
                    'subcloud_match_max_query_points': ParameterValue(120, value_type=int),
                    'subcloud_match_max_candidate_points': ParameterValue(300, value_type=int),
                    'subcloud_match_max_matches': ParameterValue(80, value_type=int),
                    'subcloud_match_max_hamming': ParameterValue(85, value_type=int),
                    'subcloud_match_use_ratio_test': ParameterValue(True, value_type=bool),
                    'subcloud_match_ratio': ParameterValue(0.90, value_type=float),
                    'subcloud_match_use_spatial_gate': ParameterValue(True, value_type=bool),
                    'subcloud_match_max_spatial_distance_m': ParameterValue(1.50, value_type=float),
                    'subcloud_match_allow_query_without_world': ParameterValue(True, value_type=bool),
                    
                    # Fase 8R: LOOP-SUBCLOUD-ERROR con RANSAC 3D-3D.
                    # Diagnóstico only: no fusiona, no optimiza, no ancla.
                    'subcloud_error_diagnostics_enabled': ParameterValue(True, value_type=bool),
                    'subcloud_error_store_events': ParameterValue(True, value_type=bool),
                    'subcloud_error_ransac_iterations': ParameterValue(80, value_type=int),
                    'subcloud_error_min_matches': ParameterValue(20, value_type=int),
                    'subcloud_error_min_inliers': ParameterValue(15, value_type=int),
                    'subcloud_error_inlier_threshold_m': ParameterValue(0.35, value_type=float),
                    'subcloud_error_min_inlier_ratio': ParameterValue(0.25, value_type=float),
                    'subcloud_error_max_events': ParameterValue(1000, value_type=int),
                    
                    # Fase 9R: decisión única dry-run desde LOOP-SUBCLOUD-ERROR.
                    # No fusiona, no optimiza, no ancla, no cambia RViz.
                    'subcloud_decision_dry_run_enabled': ParameterValue(True, value_type=bool),

                    # Umbrales mínimos generales.
                    'subcloud_decision_min_inliers': ParameterValue(20, value_type=int),
                    'subcloud_decision_min_inlier_ratio': ParameterValue(0.30, value_type=float),
                    'subcloud_decision_min_coverage': ParameterValue(0.03, value_type=float),

                    # Fase 11R-A:
                    # Fusión más conservadora. Casos buenos con 15-25 cm pueden seguir entrando,
                    # pero eventos de banda media quedan más fácilmente en HOLD o LOCAL_OPT.
                    'subcloud_decision_fusion_max_error_t_m': ParameterValue(0.30, value_type=float),
                    'subcloud_decision_fusion_max_yaw_deg': ParameterValue(3.5, value_type=float),
                    'subcloud_decision_fusion_max_rot_deg': ParameterValue(5.0, value_type=float),
                    'subcloud_decision_fusion_max_mean_residual_m': ParameterValue(0.10, value_type=float),
                    'subcloud_decision_fusion_max_residual_m': ParameterValue(0.35, value_type=float),
                    'subcloud_decision_fusion_min_inliers': ParameterValue(25, value_type=int),
                    'subcloud_decision_fusion_min_inlier_ratio': ParameterValue(0.45, value_type=float),
                    'subcloud_decision_fusion_min_coverage': ParameterValue(0.03, value_type=float),

                    # LOCAL_LOOP_OPT_TASK dry-run:
                    # Solo cuando hay error grande, pero la geometría sigue siendo consistente.
                    'subcloud_decision_opt_min_error_t_m': ParameterValue(0.60, value_type=float),
                    'subcloud_decision_opt_min_yaw_deg': ParameterValue(8.0, value_type=float),
                    'subcloud_decision_opt_min_rot_deg': ParameterValue(10.0, value_type=float),
                    'subcloud_decision_opt_max_mean_residual_m': ParameterValue(0.15, value_type=float),
                    'subcloud_decision_opt_max_residual_m': ParameterValue(0.45, value_type=float),
                    'subcloud_decision_opt_min_inliers': ParameterValue(25, value_type=int),
                    'subcloud_decision_opt_min_inlier_ratio': ParameterValue(0.35, value_type=float),
                    'subcloud_decision_opt_min_coverage': ParameterValue(0.03, value_type=float),
                    
                    # Fase 11R-A:
                    # Regla hard para errores grandes con geometría razonable.
                    # Evita que errores tipo 3 m / 30 grados queden en HOLD por pocos inliers.
                    'subcloud_decision_hard_opt_enabled': ParameterValue(True, value_type=bool),
                    'subcloud_decision_hard_opt_min_error_t_m': ParameterValue(0.80, value_type=float),
                    'subcloud_decision_hard_opt_min_yaw_deg': ParameterValue(10.0, value_type=float),
                    'subcloud_decision_hard_opt_min_rot_deg': ParameterValue(12.0, value_type=float),
                    'subcloud_decision_hard_opt_min_inliers': ParameterValue(15, value_type=int),
                    'subcloud_decision_hard_opt_min_inlier_ratio': ParameterValue(0.25, value_type=float),
                    'subcloud_decision_hard_opt_min_coverage': ParameterValue(0.03, value_type=float),
                    'subcloud_decision_hard_opt_max_mean_residual_m': ParameterValue(0.18, value_type=float),
                    'subcloud_decision_hard_opt_max_residual_m': ParameterValue(0.50, value_type=float),
                    
                    # Fase 11R-C.2:
                    # Error extremo: crea LOCAL_LOOP_OPT_TASK aunque el residual medio
                    # sea algo mayor que el hard_opt normal. No aplica optimización real.
                    'subcloud_decision_very_hard_opt_enabled': ParameterValue(True, value_type=bool),
                    'subcloud_decision_very_hard_opt_min_error_t_m': ParameterValue(2.0, value_type=float),
                    'subcloud_decision_very_hard_opt_min_yaw_deg': ParameterValue(20.0, value_type=float),
                    'subcloud_decision_very_hard_opt_min_rot_deg': ParameterValue(20.0, value_type=float),
                    'subcloud_decision_very_hard_opt_min_inliers': ParameterValue(20, value_type=int),
                    'subcloud_decision_very_hard_opt_min_inlier_ratio': ParameterValue(0.35, value_type=float),
                    'subcloud_decision_very_hard_opt_min_coverage': ParameterValue(0.05, value_type=float),
                    'subcloud_decision_very_hard_opt_max_mean_residual_m': ParameterValue(0.22, value_type=float),
                    'subcloud_decision_very_hard_opt_max_residual_m': ParameterValue(0.40, value_type=float),

                    # Fase 11R-C.2:
                    # Error moderado con soporte fuerte: permite dry-run para deriva visible
                    # sin abrir la puerta a loops débiles.
                    'subcloud_decision_relaxed_opt_enabled': ParameterValue(True, value_type=bool),
                    'subcloud_decision_relaxed_opt_min_error_t_m': ParameterValue(0.60, value_type=float),
                    'subcloud_decision_relaxed_opt_min_yaw_deg': ParameterValue(8.0, value_type=float),
                    'subcloud_decision_relaxed_opt_min_rot_deg': ParameterValue(8.0, value_type=float),
                    'subcloud_decision_relaxed_opt_min_inliers': ParameterValue(30, value_type=int),
                    'subcloud_decision_relaxed_opt_min_inlier_ratio': ParameterValue(0.50, value_type=float),
                    'subcloud_decision_relaxed_opt_min_coverage': ParameterValue(0.08, value_type=float),
                    'subcloud_decision_relaxed_opt_max_mean_residual_m': ParameterValue(0.18, value_type=float),
                    'subcloud_decision_relaxed_opt_max_residual_m': ParameterValue(0.35, value_type=float),
                    
                    # Fase 11R-C3:
                    # Deriva lateral/traslacional: error_t medio, yaw/rot bajos,
                    # muchos inliers y residual medio más alto por doble pared lateral.
                    'subcloud_decision_lateral_opt_enabled': ParameterValue(True, value_type=bool),
                    'subcloud_decision_lateral_opt_min_error_t_m': ParameterValue(0.50, value_type=float),
                    'subcloud_decision_lateral_opt_max_yaw_deg': ParameterValue(5.0, value_type=float),
                    'subcloud_decision_lateral_opt_max_rot_deg': ParameterValue(6.0, value_type=float),
                    'subcloud_decision_lateral_opt_min_inliers': ParameterValue(35, value_type=int),
                    'subcloud_decision_lateral_opt_min_inlier_ratio': ParameterValue(0.45, value_type=float),
                    'subcloud_decision_lateral_opt_min_coverage': ParameterValue(0.07, value_type=float),
                    'subcloud_decision_lateral_opt_max_mean_residual_m': ParameterValue(0.27, value_type=float),
                    'subcloud_decision_lateral_opt_max_residual_m': ParameterValue(0.36, value_type=float),

                    # ANCHOR_CANDIDATE dry-run:
                    # Para query sin world pose; no se usa error_t/yaw.
                    'subcloud_decision_anchor_min_inliers': ParameterValue(25, value_type=int),
                    'subcloud_decision_anchor_min_inlier_ratio': ParameterValue(0.35, value_type=float),
                    'subcloud_decision_anchor_min_coverage': ParameterValue(0.03, value_type=float),
                    'subcloud_decision_anchor_max_mean_residual_m': ParameterValue(0.10, value_type=float),
                    'subcloud_decision_anchor_max_residual_m': ParameterValue(0.35, value_type=float),
                    
                    # Fase 10R-A: fusión real conservadora desde FUSION_EVENT subcloud.
                    # No optimiza, no ancla, no usa la fusión legacy KeyFrame-KeyFrame.
                    'subcloud_fusion_enabled': ParameterValue(True, value_type=bool),
                    'subcloud_fusion_dry_run': ParameterValue(False, value_type=bool),
                    'subcloud_fusion_process_event_once': ParameterValue(True, value_type=bool),
                    'subcloud_fusion_max_events_per_tick': ParameterValue(4, value_type=int),
                    'subcloud_fusion_max_pairs_per_event': ParameterValue(120, value_type=int),
                    'subcloud_fusion_min_accepted_pairs': ParameterValue(12, value_type=int),
                    'subcloud_fusion_max_pair_distance_m': ParameterValue(0.35, value_type=float),
                    'subcloud_fusion_max_residual_m': ParameterValue(0.35, value_type=float),
                    'subcloud_fusion_max_hamming': ParameterValue(85, value_type=int),
                    'subcloud_fusion_observation_weight_scale': ParameterValue(2.5, value_type=float),
                    'subcloud_fusion_initial_score': ParameterValue(0.70, value_type=float),
                    'subcloud_fusion_validate_region': ParameterValue(True, value_type=bool),
                    'subcloud_fusion_validated_region_radius_m': ParameterValue(1.50, value_type=float),
                    
                    # Durante la primera prueba de 10R-B activamos algunos logs de pares
                    # para confirmar que ya no todo acaba en reject_missing/reject_other.
                    'subcloud_fusion_log_pairs': ParameterValue(False, value_type=bool),
                    'subcloud_fusion_max_pair_logs': ParameterValue(20, value_type=int),

                    # Fase 10R-B: usar snapshots guardados en LoopPointMatch.
                    'subcloud_fusion_use_match_snapshots': ParameterValue(True, value_type=bool),
                    'subcloud_fusion_max_attempts_per_event': ParameterValue(2, value_type=int),

                    # Evidencia suave de score por inliers/outliers subcloud.
                    # No modifica ORB-SLAM3; solo afecta al score de fused observations/tracks.
                    'subcloud_mappoint_evidence_enabled': ParameterValue(True, value_type=bool),
                    'subcloud_evidence_inlier_gain': ParameterValue(0.025, value_type=float),
                    'subcloud_evidence_outlier_penalty': ParameterValue(0.008, value_type=float),
                    'subcloud_evidence_max_positive': ParameterValue(0.25, value_type=float),
                    'subcloud_evidence_max_negative': ParameterValue(-0.12, value_type=float),
                    'subcloud_evidence_apply_weight_scale': ParameterValue(1.0, value_type=float),

                    # Covisibilidad global de servidor entre KeyFrames.
                    # No modifica el grafo interno de ORB-SLAM3.
                    'subcloud_kf_covisibility_enabled': ParameterValue(True, value_type=bool),
                    'subcloud_kf_covisibility_min_accepted_pairs': ParameterValue(8, value_type=int),
                    'subcloud_kf_covisibility_log_edges': ParameterValue(False, value_type=bool),
                    
                    # Fase 11R-A: creación de LOCAL_LOOP_OPT_TASK dry-run,
                    # región pending, bloqueo de fusiones y keyframes pending.
                    'subcloud_local_loop_task_generation_enabled': ParameterValue(True, value_type=bool),
                    'subcloud_local_loop_task_dry_run': ParameterValue(True, value_type=bool),
                    'subcloud_local_loop_task_process_once': ParameterValue(True, value_type=bool),

                    # Publica busy=true para que el dron pueda detenerse.
                    'subcloud_local_loop_task_publish_busy': ParameterValue(True, value_type=bool),

                    # Si un submapa toca una región con optimización pendiente,
                    # no se fusionan sus FUSION_EVENT hasta terminar/recheck.
                    'subcloud_local_loop_task_block_fusion': ParameterValue(True, value_type=bool),

                    # KeyFrames nuevos durante pending opt:
                    # se guardan y se publican provisionalmente, pero no hacen BoW/subcloud.
                    'subcloud_local_loop_task_mark_new_kfs_pending': ParameterValue(True, value_type=bool),
                    'subcloud_local_loop_task_publish_new_kfs_provisional': ParameterValue(True, value_type=bool),

                    'subcloud_local_loop_task_max_per_tick': ParameterValue(4, value_type=int),
                    'subcloud_local_loop_task_max_attempts': ParameterValue(1, value_type=int),
                    'subcloud_local_loop_task_mark_state_dirty': ParameterValue(True, value_type=bool),
                    'subcloud_local_loop_task_log_existing': ParameterValue(True, value_type=bool),
                    'subcloud_local_loop_task_log_blocked_fusion': ParameterValue(True, value_type=bool),
                    
                    # Fase 11R-B:
                    # Construcción dry-run de ventana local de optimización.
                    # No mueve poses, no ejecuta g2o, no toca ORB-SLAM3 interno.
                    'subcloud_local_opt_window_enabled': ParameterValue(True, value_type=bool),
                    'subcloud_local_opt_window_build_once': ParameterValue(True, value_type=bool),
                    'subcloud_local_opt_window_max_per_tick': ParameterValue(2, value_type=int),

                    # Radio/vecindad local.
                    'subcloud_local_opt_window_query_radius_m': ParameterValue(2.0, value_type=float),
                    'subcloud_local_opt_window_candidate_radius_m': ParameterValue(2.0, value_type=float),
                    'subcloud_local_opt_window_boundary_radius_m': ParameterValue(3.0, value_type=float),
                    'subcloud_local_opt_window_query_temporal_span': ParameterValue(8, value_type=int),
                    'subcloud_local_opt_window_candidate_temporal_span': ParameterValue(6, value_type=int),

                    # Límites de tamaño.
                    'subcloud_local_opt_window_max_query_kfs': ParameterValue(18, value_type=int),
                    'subcloud_local_opt_window_max_candidate_seed_kfs': ParameterValue(8, value_type=int),
                    'subcloud_local_opt_window_max_candidate_kfs': ParameterValue(24, value_type=int),
                    'subcloud_local_opt_window_max_covisibility_kfs': ParameterValue(12, value_type=int),
                    'subcloud_local_opt_window_max_boundary_kfs': ParameterValue(16, value_type=int),
                    'subcloud_local_opt_window_max_total_kfs': ParameterValue(64, value_type=int),
                    'subcloud_local_opt_window_min_candidate_inlier_support': ParameterValue(3, value_type=int),

                    # Qué incluir.
                    'subcloud_local_opt_window_include_same_submap_temporal': ParameterValue(True, value_type=bool),
                    'subcloud_local_opt_window_include_covisibility': ParameterValue(True, value_type=bool),
                    'subcloud_local_opt_window_include_boundary': ParameterValue(True, value_type=bool),
                    'subcloud_local_opt_window_include_pending_as_excluded': ParameterValue(True, value_type=bool),

                    # Pesos diagnósticos. Todavía no se usan para optimizar,
                    # solo para comprobar qué lado tiraría más en Fase 11R-C/12R.
                    'subcloud_local_opt_window_weight_base': ParameterValue(1.0, value_type=float),
                    'subcloud_local_opt_window_weight_query_root': ParameterValue(2.0, value_type=float),
                    'subcloud_local_opt_window_weight_candidate_seed': ParameterValue(3.0, value_type=float),
                    'subcloud_local_opt_window_weight_anchor': ParameterValue(8.0, value_type=float),
                    'subcloud_local_opt_window_weight_fiducial_prior': ParameterValue(12.0, value_type=float),
                    # Fase 12R-B:
                    # Las zonas ya validadas/fusionadas deben moverse menos.
                    # Además, el grafo de restricciones añade peso extra por aristas KF-KF.
                    'subcloud_local_opt_window_weight_validated': ParameterValue(12.0, value_type=float),
                    'subcloud_local_opt_window_weight_fusion_done': ParameterValue(10.0, value_type=float),
                    'subcloud_local_opt_window_weight_covis_per_pair': ParameterValue(0.05, value_type=float),
                    'subcloud_local_opt_window_weight_covis_cap': ParameterValue(12.0, value_type=float),
                    'subcloud_local_opt_window_weight_boundary': ParameterValue(20.0, value_type=float),
                    'subcloud_local_opt_window_weight_pending_excluded': ParameterValue(0.05, value_type=float),

                    # Logs detallados de ventana.
                    'subcloud_local_opt_window_log_kfs': ParameterValue(True, value_type=bool),
                    'subcloud_local_opt_window_max_kf_logs': ParameterValue(40, value_type=int),
                    
                    # Fase 12R-B:
                    # Grafo persistente de restricciones KF-KF.
                    # No toca ORB-SLAM3 interno. Se usa para fijar fiduciales,
                    # proteger zonas fusionadas y reducir alpha de KFs muy conectados.
                    'local_opt_constraint_graph_enabled': ParameterValue(True, value_type=bool),
                    'local_opt_constraint_graph_log_edges': ParameterValue(False, value_type=bool),
                    'local_opt_constraint_graph_log_summary': ParameterValue(True, value_type=bool),

                    'local_opt_constraint_min_support_pairs': ParameterValue(3, value_type=int),
                    'local_opt_constraint_max_candidate_kfs_per_event': ParameterValue(12, value_type=int),

                    # Fuerzas base.
                    'local_opt_constraint_fusion_strength_base': ParameterValue(25.0, value_type=float),
                    'local_opt_constraint_covisibility_strength_base': ParameterValue(8.0, value_type=float),
                    'local_opt_constraint_local_apply_strength_base': ParameterValue(35.0, value_type=float),
                    # Good fiducial priors: solo si la pose actual del KF coincide
                    # con la pose real/medida por el fiducial.
                    'local_opt_constraint_fiducial_strength': ParameterValue(10000.0, value_type=float),

                    # Soft fiducial priors: error medio contra fiducial.
                    # Dan rigidez, pero no bloquean la ventana local.
                    'local_opt_constraint_soft_fiducial_strength': ParameterValue(120.0, value_type=float),

                    # Fiducial global debt: error alto contra fiducial.
                    # No debe actuar como ancla; solo marca deuda para optimización global futura.
                    'local_opt_constraint_fiducial_debt_strength': ParameterValue(5.0, value_type=float),

                    'local_opt_constraint_strength_max': ParameterValue(20000.0, value_type=float),

                    # Conversión de fuerza a pesos/alpha.
                    'local_opt_constraint_translation_weight_scale': ParameterValue(1.0, value_type=float),
                    'local_opt_constraint_rotation_weight_scale': ParameterValue(1.0, value_type=float),
                    'local_opt_constraint_alpha_strength_divisor': ParameterValue(35.0, value_type=float),
                    'local_opt_constraint_min_alpha_scale': ParameterValue(0.03, value_type=float),
                    'local_opt_constraint_max_alpha_scale': ParameterValue(1.0, value_type=float),
                    'local_opt_constraint_pose_prior_weight_scale': ParameterValue(1.0, value_type=float),

                    # Cap del bonus para KFs variables. Las aristas fuertes ya reducen alpha;
                    # no queremos que un KF variable reciba pose_prior_weight infinito.
                    'local_opt_constraint_pose_prior_bonus_cap': ParameterValue(300.0, value_type=float),

                    # Desde 12R-B2 queda como diagnóstico. No fija por max_strength general.
                    'local_opt_constraint_hard_fix_strength': ParameterValue(5000.0, value_type=float),
                    'local_opt_constraint_hard_fix_fiducials': ParameterValue(True, value_type=bool),
                    'local_opt_constraint_skip_hard_fixed_on_apply': ParameterValue(True, value_type=bool),
                    
                    # Fase 12R-B2:
                    # Clasificación de prior fiducial según error contra pose real/medida.
                    # Bajo error  -> FIDUCIAL_HARD_ANCHOR
                    # Error medio -> FIDUCIAL_SOFT_PRIOR
                    # Error alto  -> FIDUCIAL_GLOBAL_DEBT
                    'local_opt_fiducial_hard_max_error_t_m': ParameterValue(0.25, value_type=float),
                    'local_opt_fiducial_hard_max_yaw_deg': ParameterValue(4.0, value_type=float),
                    'local_opt_fiducial_hard_max_rot_deg': ParameterValue(6.0, value_type=float),

                    'local_opt_fiducial_soft_max_error_t_m': ParameterValue(0.80, value_type=float),
                    'local_opt_fiducial_soft_max_yaw_deg': ParameterValue(15.0, value_type=float),
                    'local_opt_fiducial_soft_max_rot_deg': ParameterValue(20.0, value_type=float),

                    # Si no hay pose mundo actual previa, el primer fiducial puede crear
                    # ancla inicial porque no hay contradicción medible.
                    'local_opt_fiducial_no_current_pose_as_hard_anchor': ParameterValue(True, value_type=bool),
                    
                    # Fase 12R-D2:
                    # Las deudas fiduciales NO entran al MultiDroneSystem como priors normales.
                    # Se registran para diagnóstico/futura optimización global, pero no fuerzan
                    # el optimizador legacy.
                    'local_opt_fiducial_debt_add_to_multisystem': ParameterValue(False, value_type=bool),
                    'local_opt_fiducial_debt_force_legacy_optimization': ParameterValue(False, value_type=bool),
                    'local_opt_fiducial_debt_count_as_ready_prior': ParameterValue(False, value_type=bool),
                    'local_opt_fiducial_debt_refresh_enabled': ParameterValue(False, value_type=bool),
                    'local_opt_fiducial_debt_max_new_per_window': ParameterValue(20, value_type=int),

                    # Tras una optimización aplicada, reforzamos la arista corregida.
                    'local_opt_constraint_post_apply_strength_gain': ParameterValue(25.0, value_type=float),
                    'local_opt_constraint_post_apply_min_improvement_ratio': ParameterValue(0.20, value_type=float),
                    
                    # Fase 12R-B3:
                    # No aplicar optimizaciones locales si los KFs móviles no tienen soporte
                    # real del grafo de restricciones. Esto evita mover zonas fiduciales buenas
                    # cuando constraint_strength=0.
                    'local_opt_apply_require_constraint_support': ParameterValue(True, value_type=bool),
                    'local_opt_apply_min_constrained_movable_kfs': ParameterValue(2, value_type=int),
                    'local_opt_apply_min_max_constraint_strength': ParameterValue(40.0, value_type=float),
                    'local_opt_apply_reject_all_zero_constraints': ParameterValue(True, value_type=bool),
                    'local_opt_apply_skip_fiducial_direct_without_constraint': ParameterValue(True, value_type=bool),
                    
                    # Fase 12R-C:
                    # Aristas objetivo de optimización local.
                    # Representan "reducir el error de este loop"; no son rigidez del grafo.
                    'local_opt_objective_edges_enabled': ParameterValue(True, value_type=bool),
                    'local_opt_objective_register_at_task_creation': ParameterValue(True, value_type=bool),
                    'local_opt_objective_strength_base': ParameterValue(80.0, value_type=float),
                    'local_opt_apply_min_objective_strength': ParameterValue(20.0, value_type=float),

                    # Precheck: se permite aplicar si suficientes KFs móviles tienen
                    # soporte de rigidez o soporte objetivo del loop.
                    'local_opt_apply_min_supported_movable_kfs': ParameterValue(2, value_type=int),
                    'local_opt_apply_min_supported_movable_ratio': ParameterValue(0.25, value_type=float),
                    
                    # Fase 12R-D:
                    # Optimizador local de pose graph.
                    # Sustituye el reparto por alpha por una minimización numérica local:
                    #   coste = objetivo loop + edges KF-KF + priors a pose actual.
                    'local_pose_graph_optimizer_enabled': ParameterValue(True, value_type=bool),
                    'local_pose_graph_log_solver': ParameterValue(True, value_type=bool),

                    # Iteraciones del solver local. Mantener bajo para no crear cuello de botella.
                    'local_pose_graph_iterations': ParameterValue(3, value_type=int),

                    # Levenberg-Marquardt / diferencias finitas.
                    'local_pose_graph_lm_lambda': ParameterValue(0.001, value_type=float),
                    'local_pose_graph_numeric_eps_t_m': ParameterValue(0.001, value_type=float),
                    'local_pose_graph_numeric_eps_rot_rad': ParameterValue(0.0001, value_type=float),

                    # Paso máximo por iteración para que el solver no salte de golpe.
                    'local_pose_graph_step_scale': ParameterValue(1.0, value_type=float),
                    'local_pose_graph_max_step_t_m': ParameterValue(0.15, value_type=float),
                    'local_pose_graph_max_step_rot_deg': ParameterValue(2.5, value_type=float),

                    # Peso del objetivo principal: acercar nubes query/candidate.
                    'local_pose_graph_objective_weight': ParameterValue(4.0, value_type=float),

                    # Peso de edges/prior. El strength del grafo se convierte a peso residual.
                    'local_pose_graph_edge_weight_scale': ParameterValue(0.02, value_type=float),
                    'local_pose_graph_prior_weight_scale': ParameterValue(0.02, value_type=float),
                    'local_pose_graph_max_edge_weight': ParameterValue(15.0, value_type=float),
                    'local_pose_graph_max_prior_weight': ParameterValue(6.0, value_type=float),

                    # Validación del solver.
                    'local_pose_graph_min_cost_reduction_ratio': ParameterValue(0.01, value_type=float),
                    'local_pose_graph_max_result_delta_t_m': ParameterValue(2.0, value_type=float),
                    'local_pose_graph_max_result_delta_rot_deg': ParameterValue(30.0, value_type=float),
                    
                    # Fase 12R-D3:
                    # La ventana puede contener muchos KFs, pero el solver solo optimiza
                    # un subconjunto seleccionado y distribuido espacialmente.
                    'local_pose_graph_max_variables': ParameterValue(12, value_type=int),
                    'local_pose_graph_max_objective_matches': ParameterValue(40, value_type=int),
                    'local_pose_graph_max_graph_edges': ParameterValue(80, value_type=int),

                    # Ya no rechazamos directamente si la ventana tiene 23/31 KFs:
                    # seleccionamos los mejores 12.
                    'local_pose_graph_reject_if_too_many_variables': ParameterValue(False, value_type=bool),
                    'local_pose_graph_variable_selection_enabled': ParameterValue(True, value_type=bool),
                    'local_pose_graph_log_variable_selection': ParameterValue(True, value_type=bool),

                    # Mantener representantes de ambos lados del loop.
                    'local_pose_graph_min_query_variables': ParameterValue(2, value_type=int),
                    'local_pose_graph_min_candidate_variables': ParameterValue(2, value_type=int),
                    'local_pose_graph_max_query_variables': ParameterValue(8, value_type=int),
                    'local_pose_graph_max_candidate_variables': ParameterValue(6, value_type=int),

                    # Scoring de selección.
                    'local_pose_graph_var_score_objective_support': ParameterValue(10000.0, value_type=float),
                    'local_pose_graph_var_score_objective_edge': ParameterValue(1500.0, value_type=float),
                    'local_pose_graph_var_score_inlier_support': ParameterValue(80.0, value_type=float),
                    'local_pose_graph_var_score_covis_support': ParameterValue(8.0, value_type=float),
                    'local_pose_graph_var_score_seed_bonus': ParameterValue(700.0, value_type=float),
                    'local_pose_graph_var_score_query_seed_bonus': ParameterValue(1200.0, value_type=float),
                    'local_pose_graph_var_score_distance_penalty': ParameterValue(0.20, value_type=float),

                    # Diversidad espacial: evita elegir todos los KFs en el mismo punto.
                    'local_pose_graph_var_score_spatial_diversity': ParameterValue(180.0, value_type=float),

                    # Masa representativa: KFs no seleccionados aumentan el prior de su representante.
                    'local_pose_graph_representation_radius_m': ParameterValue(2.5, value_type=float),
                    'local_pose_graph_represented_prior_weight_scale': ParameterValue(0.50, value_type=float),
                    'local_pose_graph_represented_prior_bonus_cap': ParameterValue(250.0, value_type=float),

                    # Propagación de delta a KFs no seleccionados dentro de la ventana.
                    'local_pose_graph_propagate_unselected_kfs': ParameterValue(True, value_type=bool),
                    'local_pose_graph_propagate_same_side_only': ParameterValue(True, value_type=bool),
                    'local_pose_graph_propagation_max_neighbors': ParameterValue(3, value_type=int),
                    'local_pose_graph_propagation_radius_m': ParameterValue(2.5, value_type=float),
                    'local_pose_graph_propagation_scale': ParameterValue(0.80, value_type=float),
                    'local_pose_graph_propagation_min_weight_sum': ParameterValue(1e-6, value_type=float),
                                        
                    # Fase 11R-C:
                    # Optimización numérica dry-run sobre copia.
                    # No mueve poses, no mueve MapPoints, no publica correcciones.
                    'subcloud_local_opt_dryrun_enabled': ParameterValue(True, value_type=bool),
                    'subcloud_local_opt_dryrun_run_once': ParameterValue(True, value_type=bool),
                    'subcloud_local_opt_dryrun_max_per_tick': ParameterValue(2, value_type=int),

                    # Para considerar que el dry-run sería útil.
                    'subcloud_local_opt_dryrun_min_improvement_m': ParameterValue(0.05, value_type=float),
                    'subcloud_local_opt_dryrun_min_improvement_ratio': ParameterValue(0.20, value_type=float),

                    # Reparto de corrección según pesos/movilidad.
                    'subcloud_local_opt_dryrun_weight_epsilon': ParameterValue(1e-6, value_type=float),
                    'subcloud_local_opt_dryrun_max_query_alpha': ParameterValue(0.95, value_type=float),
                    'subcloud_local_opt_dryrun_max_candidate_alpha': ParameterValue(0.95, value_type=float),
                    'subcloud_local_opt_dryrun_min_side_alpha': ParameterValue(0.05, value_type=float),

                    # Diagnóstico de deltas potencialmente peligrosos.
                    'subcloud_local_opt_dryrun_max_kf_delta_t_warn_m': ParameterValue(3.0, value_type=float),
                    'subcloud_local_opt_dryrun_max_kf_delta_yaw_warn_deg': ParameterValue(35.0, value_type=float),

                    # Logs detallados por KF.
                    'subcloud_local_opt_dryrun_log_kfs': ParameterValue(True, value_type=bool),
                    'subcloud_local_opt_dryrun_max_kf_logs': ParameterValue(40, value_type=int),
                    
                    # Fase 12R-A:
                    # Aplica al servidor las correcciones locales calculadas en dry-run.
                    # No toca ORB-SLAM3 interno. Solo escribe optimized_kf_poses_,
                    # reconstruye optimized_world_T_local_by_submap_ y republica mapas.
                    'subcloud_local_opt_apply_enabled': ParameterValue(True, value_type=bool),
                    'subcloud_local_opt_apply_once': ParameterValue(True, value_type=bool),

                    # Muy importante: aplicar solo una corrección por tick.
                    # Si hay dos tareas útiles en la misma región, la primera aplicada cierra
                    # las hermanas para evitar doble corrección.
                    'subcloud_local_opt_apply_max_per_tick': ParameterValue(1, value_type=int),

                    'subcloud_local_opt_apply_min_improvement_m': ParameterValue(0.05, value_type=float),
                    'subcloud_local_opt_apply_min_improvement_ratio': ParameterValue(0.20, value_type=float),

                    # Guard conservador: después de aplicar, el residual medio predicho debe
                    # quedar razonable. En tus dry-runs malos útiles estaba en torno a 0.23 m.
                    'subcloud_local_opt_apply_max_after_mean_residual_m': ParameterValue(0.30, value_type=float),

                    # Guard contra explosiones.
                    'subcloud_local_opt_apply_max_kf_delta_t_m': ParameterValue(2.0, value_type=float),
                    'subcloud_local_opt_apply_max_kf_delta_yaw_deg': ParameterValue(25.0, value_type=float),
                    'subcloud_local_opt_apply_max_kf_delta_rot_deg': ParameterValue(30.0, value_type=float),

                    # Evita escribir KFs con cambios despreciables.
                    'subcloud_local_opt_apply_min_kf_delta_t_m': ParameterValue(0.005, value_type=float),

                    'subcloud_local_opt_apply_publish_immediately': ParameterValue(True, value_type=bool),
                    'subcloud_local_opt_apply_requeue_pending_kfs': ParameterValue(True, value_type=bool),
                    'subcloud_local_opt_apply_clear_pending_region': ParameterValue(True, value_type=bool),
                    'subcloud_local_opt_apply_log_kfs': ParameterValue(True, value_type=bool),
                    'subcloud_local_opt_apply_max_kf_logs': ParameterValue(40, value_type=int),

                    # Fase 5R: estado por KeyFrame y cola de loop jobs.
                    # Solo agenda trabajos; no construye subnubes ni hace matching todavía.
                    'subcloud_loop_state_enabled': ParameterValue(True, value_type=bool),
                    'subcloud_loop_job_queue_enabled': ParameterValue(True, value_type=bool),
                    'subcloud_loop_schedule_from_bow_enabled': ParameterValue(True, value_type=bool),
                    'subcloud_loop_schedule_require_legacy_bow_queued': ParameterValue(True, value_type=bool),
                    'subcloud_loop_job_max_queue': ParameterValue(200, value_type=int),
                    'subcloud_loop_state_min_query_mappoints': ParameterValue(30, value_type=int),
                    'subcloud_loop_cooldown_no_candidate_s': ParameterValue(5.0, value_type=float),
                    'subcloud_loop_cooldown_no_matches_s': ParameterValue(8.0, value_type=float),
                    'subcloud_loop_cooldown_bad_geometry_s': ParameterValue(10.0, value_type=float),
                    
                    # Fase 6R.1: procesar la cola con presupuesto.
                    # Solo query cloud + candidate subcloud dentro del job.
                    # Sin matching, sin RANSAC, sin fusión, sin optimización.
                    'subcloud_loop_job_processing_enabled': ParameterValue(True, value_type=bool),
                    'subcloud_loop_candidate_build_enabled': ParameterValue(True, value_type=bool),
                    'subcloud_loop_max_jobs_per_tick': ParameterValue(1, value_type=int),
                    'subcloud_loop_drop_stale_jobs': ParameterValue(True, value_type=bool),
                    'subcloud_loop_max_job_age_s': ParameterValue(20.0, value_type=float),

                    # Fase 6R.2: candidate subcloud usando candidatos BoW como semillas.
                    # Sigue sin matching, sin RANSAC, sin fusión y sin optimización.
                    'subcloud_bow_seed_candidate_enabled': ParameterValue(True, value_type=bool),
                    'subcloud_bow_seed_fallback_to_frustum_for_revisit': ParameterValue(True, value_type=bool),
                    'subcloud_bow_seed_allow_anchor_candidate': ParameterValue(True, value_type=bool),
                    'subcloud_bow_seed_max_seed_kfs': ParameterValue(8, value_type=int),
                    'subcloud_bow_seed_max_points_per_seed_kf': ParameterValue(250, value_type=int),
                    
                    "fiducial_prior_max_kf_distance_m": 0.75,
                    "fiducial_prior_min_interval_same_fiducial_s": 1.0,

                    'min_time_between_forced_snapshots_s': ParameterValue(5.0, value_type=float),
                    'max_empty_delta_count': ParameterValue(20, value_type=int),
                    'request_snapshot_on_sequence_jump': ParameterValue(True, value_type=bool),
                    'request_snapshot_on_empty_deltas': ParameterValue(True, value_type=bool),

                    # Fiduciales simulados en world
                    'fiducials.ids': [1, 2],
                    'fiducials.x': [0.0, 0.0],
                    'fiducials.y': [9.0, -9.0],
                    'fiducials.z': [1.0, 1.0],
                    'fiducials.roll': [0.0, 0.0],
                    'fiducials.pitch': [0.0, 0.0],
                    'fiducials.yaw': [0.0, 0.0],
                    'fiducials.radius': [2.0, 2.0],

                    # Paso 3/4: candidatos BoW
                    'enable_bow_candidate_detection': ParameterValue(True, value_type=bool),
                    'enable_intra_drone_loop_detection': ParameterValue(True, value_type=bool),
                    'enable_inter_drone_loop_detection': ParameterValue(True, value_type=bool),

                    'min_intra_drone_loop_kf_gap': ParameterValue(15, value_type=int),
                    'max_bow_candidates_per_keyframe': ParameterValue(16, value_type=int),
                    'max_bow_queue_size': ParameterValue(500, value_type=int),
                    'max_bow_jobs_per_tick': ParameterValue(12, value_type=int),
                    'min_common_words': ParameterValue(4, value_type=int),
                    'min_bow_score': ParameterValue(0.004, value_type=float),
                    'max_bow_query_keyframes_per_tick': ParameterValue(3, value_type=int),
                    'max_pending_bow_query_keyframes': ParameterValue(700, value_type=int),

                    # Paso 5: SearchByBoW
                    'max_hamming_distance': ParameterValue(60, value_type=int),
                    'bow_ratio_test': ParameterValue(0.75, value_type=float),
                    'min_bow_matches': ParameterValue(12, value_type=int),

                    # Paso 6: geometría SE3 + SearchByProjection
                    'geometry_ransac_iterations': ParameterValue(30, value_type=int),  # 300
                    'geometry_ransac_threshold_m': ParameterValue(0.60, value_type=float),
                    'geometry_min_ransac_inliers': ParameterValue(4, value_type=int),
                    'geometry_min_inlier_ratio': ParameterValue(0.25, value_type=float),    # 0.05
                    'geometry_max_mean_error_m': ParameterValue(0.35, value_type=float),    # 0.60
                    'geometry_projection_radius_px': ParameterValue(10.0, value_type=float),    
                    'geometry_projection_descriptor_threshold': ParameterValue(70, value_type=int),
                    'geometry_min_projection_matches': ParameterValue(12, value_type=int),   # 8
                    'geometry_min_final_inliers': ParameterValue(30, value_type=int),    # 4

                    # Publicación raw/debug
                    'merged_cloud_voxel_size_m': ParameterValue(0.05, value_type=float),
                    'max_published_global_points': ParameterValue(200000, value_type=int),
                    'publish_raw_and_optimized_clouds': ParameterValue(False, value_type=bool),
                    'optimization_immediate_heavy_publish': ParameterValue(False, value_type=bool),
                    'optimization_immediate_light_publish': ParameterValue(True, value_type=bool),
                    
                    'fiducial_force_optimization_when_waiting': ParameterValue(False, value_type=bool),
                    'fiducial_force_optimization_min_interval_s': ParameterValue(1.0, value_type=float),
                    'fiducial_force_optimization_min_good_priors': ParameterValue(3, value_type=int),
                    'fiducial_force_optimization_min_total_priors': ParameterValue(5, value_type=int),
                    'fiducial_force_optimization_only_active_window': ParameterValue(True, value_type=bool),
                    'fused_simple_use_optimized_keyframe_points': ParameterValue(True, value_type=bool),
                    'fused_simple_optimized_point_max_observations': ParameterValue(4, value_type=int),
                    'fused_simple_optimized_point_max_camera_depth_m': ParameterValue(40.0, value_type=float),
                    'fused_simple_optimized_point_max_delta_from_rigid_m': ParameterValue(6.0, value_type=float),
                    'fused_simple_log_optimized_point_usage': ParameterValue(True, value_type=bool),
                    'fiducial_force_optimization_reject_backoff_s': ParameterValue(8.0, value_type=float),
                    
                    # Paso 8: construcción del pose graph glsobal
                    'enable_pose_graph_building': ParameterValue(False, value_type=bool),
                    'pose_graph_build_period_s': ParameterValue(1.0, value_type=float),
                    'min_covisibility_weight_for_pose_graph': ParameterValue(50, value_type=int),
                    'max_covisibility_edges_per_kf': ParameterValue(2, value_type=int),
                    
                    'enable_pose_graph_optimization': ParameterValue(True, value_type=bool),
                    'pose_graph_optimization_period_s': ParameterValue(2.0, value_type=float),
                    'pose_graph_optimization_iterations': ParameterValue(4, value_type=int),
                    
                    'min_loop_edges_for_optimization': ParameterValue(4, value_type=int),
                    'min_seconds_after_last_loop_for_optimization': ParameterValue(0.5, value_type=float),
                    'min_seconds_after_last_keyframe_for_optimization': ParameterValue(0.0, value_type=float),
                    
                    "optimize_on_new_fiducial_priors": False,
                    "optimize_on_force_even_without_loops": True,
                    "precheck_pose_graph_before_optimization": True,
                    "min_pose_graph_vertices_for_optimization": 2,
                    "min_pose_graph_fixed_vertices_for_optimization": 1,
                    
                    "optimized_cloud_use_nearest_kf_fallback": True,
                    "optimized_cloud_nearest_kf_max_distance_m": 8.0,
                    
                    "continue_optimization_if_not_converged": True,
                    "optimization_min_chi2_reduction_ratio": 0.35,
                    "optimization_large_final_chi2_threshold": 1000.0,
                    "optimization_large_final_chi2_per_edge_threshold": 3.0,
                    "optimization_max_continuation_passes": 1,
                    
                    'fiducial_window_timeout_s': ParameterValue(5.0, value_type=float),
                    'fiducial_window_prior_weight': ParameterValue(20.0, value_type=float),
                    'fiducial_window_max_new_kfs_per_tick': ParameterValue(20, value_type=int),
                    "fiducial_window_kf_time_margin_before_s": 0.5,
                    "fiducial_window_kf_time_margin_after_s": 1.0,
                    "fiducial_window_backfill_kfs": 20,
                    "reject_far_fiducial_priors": True,
                    "fiducial_prior_reject_distance_m": 4.0,
                    "fiducial_prior_strong_distance_m": 2.5,
                    "fiducial_window_max_constraints_per_window": 30,
                    
                    "pose_graph_optimization_max_step_m": 2.0,
                    "pose_graph_max_fiducial_prior_weight": 120.0,

                    "pose_graph_use_active_window_filter": True,
                    "pose_graph_active_window_local_kfs": 20,

                    "enable_fast_fiducial_anchor_guess": False,
                    "fast_anchor_only_if_no_other_fiducial_in_epoch": True,
                    "fast_fiducial_anchor_blend": 0.6,
                    "fast_fiducial_anchor_min_improvement_m": 0.2,
                    
                    "enable_local_fiducial_cache_warp": False,
                    "local_fiducial_cache_warp_blend": 0.8,
                    "local_fiducial_cache_warp_margin_kfs": 35,
                    "local_fiducial_cache_warp_max_distance_m": 10.0,
                    
                    "fiducial_lock_min_good_priors": 2,
                    "fiducial_lock_min_constrained_kfs": 2,
                    "fiducial_lock_good_prior_max_dist_m": 1.5,
                    "fiducial_lock_max_min_distance_m": 1.5,
                    "fiducial_lock_min_inside_duration_s": 1.0,
                    "fiducial_lock_timeout_s": 5.0,
                    "fiducial_lock_require_local_warp": False,
                    "fiducial_lock_require_graph_optimization": False,
                    "fiducial_lock_publish_period_s": 0.2,

                    # Política de "fiducial capturado / ready_to_leave".
                    # Dejamos un único valor para evitar ambigüedad.
                    # Usamos 50.0 porque el criterio realmente restrictivo es chi2_per_edge.
                    'fiducial_ready_min_window_kfs': 5,
                    'fiducial_ready_min_good_priors': 3,
                    'fiducial_ready_require_optimization': True,
                    'fiducial_ready_require_low_chi': True,
                    'fiducial_ready_allow_initial_anchor_without_optimization': False,
                    'fiducial_ready_min_inside_duration_s': 1.0,
                    'fiducial_ready_max_seconds_since_seen_s': 1.5,
                    'fiducial_ready_max_min_distance_m': 1.5,
                    'fiducial_ready_max_final_chi2': 50.0,
                    'fiducial_ready_max_chi2_per_edge': 0.50,
                    
                    'enable_inter_drone_anchor_assist': True,
                    'inter_drone_anchor_assist_distance_m': 1.50,
                    'inter_drone_anchor_assist_period_s': 2.0,
                    'inter_drone_anchor_assist_recent_kfs': 4,
                    'inter_drone_anchor_assist_min_kfs_per_drone': 1,
                    'inter_drone_anchor_assist_clear_tested_pairs': False,
                    'inter_drone_anchor_assist_boost_duration_s': 1.5,
                    'inter_drone_anchor_assist_max_bow_queries_per_tick': 5,
                    'inter_drone_anchor_assist_max_bow_jobs_per_tick': 12,
                    
                    # Fase 5B - validación de pares de landmarks antes de fusionar
                    'fusion5b_enable_pair_distance_filter': True,

                    # Para inter-dron empezamos permisivos pero seguros.
                    'fusion5b_max_inter_drone_pair_distance_m': 0.35,

                    # Para intra-dron, más estricto.
                    'fusion5b_max_intra_drone_pair_distance_m': 0.20,

                    # Mismo dron pero distinto epoch.
                    'fusion5b_max_cross_epoch_same_drone_pair_distance_m': 0.30,

                    # De momento solo warning, no rechazo de grupo completo.
                    'fusion5b_enable_group_spread_warning': True,
                    'fusion5b_warn_group_spread_m': 0.50,

                    # Para la primera prueba conviene true; si hay demasiado log, luego false.
                    'fusion5b_log_pair_validation_samples': True,
                    'fusion5b_max_pair_validation_logs': 30,

                    # Fase 5C
                    'fusion5c_enable_inter_drone_fusion': True,

                    'fusion5c_split_merged_group_if_spread_too_high': True,
                    'fusion5c_max_merged_group_spread_m': 0.35,
                    'fusion5c_publish_rejected_group_as_singletons': False,
                    
                    # Fase 5D
                    'fusion5d_enable_intra_drone_fusion': True,
                    'fusion5d_min_intra_drone_kf_id_gap_for_fusion': 15,
                    'fusion5d_reject_intra_fusion_for_nearby_keyframes': True,
                    'fusion5d_enable_cross_epoch_same_drone_fusion': True,
                    
                    'bow_debug_intra_candidates': False,
                    'bow_log_rejected_candidates': False,

                    'enable_intra_drone_loop_assist': True,
                    'intra_drone_loop_assist_period_s': 1.0,
                    'intra_drone_loop_assist_recent_kfs': 15,
                    'intra_drone_loop_assist_history_kfs': 30,
                    'intra_drone_loop_assist_history_stride': 4,
                    'intra_drone_loop_assist_only_anchored': False,
                    'intra_drone_loop_assist_clear_tested_pairs': False,
                    'intra_drone_loop_assist_max_requeue_per_drone': 50,
                    
                    # Fase 5D-B - asociación directa de landmarks intra-dron
                    'enable_direct_landmark_association': False,
                    'direct_landmark_association_period_s': 2.0,

                    # Pares locales a probar
                    'direct_assoc_use_consecutive_kfs': True,
                    'direct_assoc_use_revisit_kfs': True,
                    'direct_assoc_recent_kfs': 30,
                    'direct_assoc_history_kfs': 50,
                    'direct_assoc_history_stride': 2,

                    # Revisita local
                    'direct_assoc_revisit_local_distance_m': 1.5,
                    'direct_assoc_min_revisit_kf_gap': 10,

                    # Carga
                    'direct_assoc_max_jobs_per_tick': 8,
                    'direct_assoc_max_queue_size': 300,

                    # Validación
                    'direct_assoc_min_raw_matches': 8,
                    'direct_assoc_min_accepted_matches': 5,
                    'direct_assoc_max_local_mp_distance_m': 0.22,
                    'direct_assoc_max_descriptor_distance': 75,
                    
                    # Fase 5E-light: limpieza no destructiva de /global_sparse_map_fused
                    'fused_cloud_enable_publication_filter': True,
                    
                    # Fase 2: nueva semántica de /global_sparse_map_fused.
                    # El tópico fused solo debe mostrar landmarks confirmados por fusión.
                    'fused_cloud_publish_only_confirmed': True,
                    'fused_cloud_min_confirmed_sources': 2,
                    'fused_cloud_reject_split_singletons_in_confirmed_mode': True,
                    
                    # Fase 3B: reintroducción conservadora de singletons estables.
                    # Un singleton solo se publica si ORB-SLAM3 ya lo ha visto desde varias vistas.
                    'fused_cloud_allow_stable_singletons': True,
                    'fused_stable_singleton_min_observations': 4,
                    'fused_stable_singleton_min_effective_observing_kfs': 3,
                    'fused_stable_singleton_min_found_ratio': 0.25,
                    'fused_stable_singleton_use_score': True,
                    'fused_stable_singleton_min_score': 1.20,
                    'fused_stable_singleton_require_density': True,
                    'fused_stable_singleton_log_samples': True,
                    'fused_stable_singleton_max_sample_logs': 30,
                    
                    # Fase 3: score de confirmación de landmarks fusionados.
                    'fused_cloud_use_confirmation_score': True,
                    'fused_confirmed_min_score': 2.0,
                    'fused_confirmed_max_group_spread_m': 0.30,
                    'fused_confirmed_min_match_count': 1,
                    'fused_confirmed_min_observing_kfs': 2,
                    'fused_confirmed_min_total_observations': 2,
                    'fused_confirmed_log_score_samples': False,
                    'fused_confirmed_max_score_sample_logs': 0,

                    # Calidad mínima de singletons
                    'fused_cloud_filter_singletons_by_quality': True,
                    'fused_cloud_singleton_min_observations': 2,
                    'fused_cloud_singleton_min_found_ratio': 0.20,

                    # Filtro de puntos aislados
                    'fused_cloud_filter_isolated_singletons': True,
                    'fused_cloud_isolated_radius_m': 0.28,
                    'fused_cloud_isolated_min_neighbors': 3,
                    
                    # Fase H:
                    # Singletons muy buenos se publican aunque estén aislados.
                    # No es caché. Solo puntos que pasan calidad fuerte.
                    'fused_keep_high_quality_singletons_without_density': False,
                    'fused_high_quality_singleton_min_observations': 8,
                    'fused_high_quality_singleton_min_effective_kfs': 5,
                    'fused_high_quality_singleton_min_found_ratio': 0.45,
                    'fused_high_quality_singleton_min_score': 2.20,

                    # Mantener grupos fusionados siempre
                    'fused_cloud_always_keep_merged_landmarks': True,

                    # Importante para no vaciar el mapa al principio
                    'fused_cloud_keep_singletons_if_no_merged': False,

                    # Voxel final visual
                    'fused_cloud_enable_voxel_filter': True,
                    'fused_cloud_voxel_size_m': 0.06,

                    'fused_cloud_filter_log_samples': False,
                    'fused_cloud_filter_max_sample_logs': 0,
                    
                    'fused_cloud_perf_logs': False,
                    'fused_cloud_perf_warn_total_ms': 999999.0,
                    'fused_cloud_perf_warn_build_ms': 999999.0,
                    'fused_cloud_reason_summary_logs': False,
                    
                    # Fase 4: evidencia negativa expected/seen/missed.
                    # Primera prueba en modo diagnóstico: no rechaza todavía.
                    'fused_confirmed_enable_expected_view_score': True,
                    'fused_confirmed_expected_view_diagnostic_only': True,

                    'fused_confirmed_expected_view_radius_m': 4.0,
                    'fused_confirmed_projection_margin_px': 20.0,
                    'fused_confirmed_min_projection_depth_m': 0.10,
                    'fused_confirmed_max_projection_depth_m': 20.0,
                    'fused_confirmed_expected_view_max_kfs': 30,

                    'fused_confirmed_seen_view_weight': 0.35,
                    'fused_confirmed_missed_view_weight': 0.15,

                    'fused_confirmed_reject_if_too_many_misses': False,
                    'fused_confirmed_max_expected_misses': 8,
                    'fused_confirmed_min_expected_views_to_reject': 4,

                    'fused_confirmed_log_expected_samples': False,
                    'fused_confirmed_max_expected_sample_logs': 0,
                    
                    # Fase 5: tópico debug de fused.
                    'publish_fused_debug_cloud': False,

                    # Opciones:
                    #   all_valid              -> todos los landmarks válidos antes de filtros.
                    #   confirmed_candidates   -> merged + stable singletons antes de densidad.
                    #   pre_voxel              -> puntos tras densidad, antes de voxel.
                    'fused_debug_cloud_mode': 'all_valid',

                    'fused_debug_cloud_apply_voxel_filter': True,
                    'fused_debug_cloud_voxel_size_m': 0.04,
                    'fused_debug_cloud_max_points': 50000,
                    'fused_debug_cloud_log_summary': True,
                    
                    # ============================================================
                    # Fase 8: política de publicación multi-dron.
                    #
                    # IMPORTANTE:
                    # Estos parámetros pertenecen al global_map_server, no al
                    # global_pose_corrector. Controlan /global_sparse_map_fused.
                    # ============================================================

                    'fused_cloud_enable_multi_drone_policy': False,
                    'fused_cloud_multi_drone_min_anchored_drones': 2,
                    'fused_cloud_multi_drone_strict_singletons': False,

                    # Umbrales más duros para singletons cuando hay varios drones anclados.
                    'fused_multi_singleton_min_observations': 7,
                    'fused_multi_singleton_min_effective_kfs': 5,
                    'fused_multi_singleton_min_found_ratio': 0.40,
                    'fused_multi_singleton_min_score': 1.80,

                    # Cap de singletons por dron después del filtro de densidad.
                    # Si todavía hay mucho ruido, bajar a 600.
                    # Si queda demasiado pobre, subir a 1200.
                    'fused_multi_max_singletons_per_drone_after_density': 800,

                    'fused_multi_keep_inter_supported_landmarks': True,
                    'fused_multi_log_policy_samples': False,
                    'fused_multi_max_policy_sample_logs': 0,
                    
                    # ============================================================
                    # Fase B: diagnóstico inter-dron entre submapas confirmados.
                    #
                    # No fusiona nada. Solo mide si hay MapPoints de distintos
                    # drones que estén cerca en world y tengan descriptor parecido.
                    # ============================================================

                    'enable_inter_drone_fusion_diagnostics': False,
                    'inter_fusion_diag_max_world_distance_m': 0.15,
                    'inter_fusion_diag_max_descriptor_distance': 70,
                    'inter_fusion_diag_min_observations': 2,
                    'inter_fusion_diag_min_found_ratio': 0.25,
                    'inter_fusion_diag_max_mps_per_submap': 1500,
                    'inter_fusion_diag_max_pair_checks': 250000,
                    'inter_fusion_diag_max_sample_logs': 20,
                    
                    # ============================================================
                    # Fase C: asociación directa inter-dron de MapPoints.
                    #
                    # Crea asociaciones conservadoras entre MapPoints de submapas
                    # confirmados usando:
                    #   world distance + descriptor ORB + calidad.
                    #
                    # Estas asociaciones alimentan el union-find de landmarks.
                    # ============================================================

                    'enable_inter_drone_direct_landmark_association': False,
                    'inter_direct_assoc_period_s': 3.0,

                    # Un poco más permisivo en geometría porque dos ORB maps distintos
                    # rara vez dejan exactamente el mismo punto en world.
                    'inter_direct_assoc_max_world_distance_m': 0.18,

                    # Mantener descriptor bastante selectivo para no unir basura.
                    'inter_direct_assoc_max_descriptor_distance': 60,

                    # MapPoints mínimamente estables.
                    'inter_direct_assoc_min_observations': 2,
                    'inter_direct_assoc_min_found_ratio': 0.25,

                    # Más cobertura, porque antes muchos pares buenos ni llegaban a procesarse.
                    'inter_direct_assoc_max_mps_per_submap': 2500,
                    'inter_direct_assoc_max_jobs_per_tick': 0,
                    'inter_direct_assoc_max_queue_size': 0,
                    'inter_direct_assoc_max_new_pairs_per_tick': 0,
                    'inter_direct_assoc_max_sample_logs': 0,
                    
                    # Fase 6: validación intra-dron con un solo dron.
                    'fusion6_enable_intra_validation_logs': False,
                    'fusion6_intra_summary_period_s': 1.0,
                    'fusion6_log_direct_reason_breakdown': False,
                    'fusion6_log_direct_pair_samples': False,
                    'fusion6_max_direct_pair_sample_logs': 20,
                    'fusion6_log_intra_loop_opt_only_samples': False,
                    'fusion6_max_intra_loop_sample_logs': 20,
                    

                    # Experimental. Primera prueba: mantener False.
                    'fusion6_allow_intra_opt_only_loops_for_fusion_candidates': False,
                    
                    # Fase 7: anchor inter-dron robusto por loop visual.
                    'inter_drone_anchor_assist_enqueue_explicit_pairs': False,
                    'inter_drone_anchor_assist_explicit_recent_anchored_kfs': 3,
                    'inter_drone_anchor_assist_explicit_recent_floating_kfs': 3,
                    'inter_drone_anchor_assist_max_explicit_pairs_per_tick': 0,
                    'inter_drone_anchor_assist_clear_pairs_before_explicit': False,
                    'inter_drone_anchor_assist_log_explicit_pairs': False,
                    'inter_drone_anchor_assist_max_explicit_pair_logs': 0,
                    'phase7_log_anchor_state': False,

                    'phase7_enable_inter_drone_loop_anchor': False,
                    'phase7_loop_anchor_min_final_inliers': 30,
                    'phase7_loop_anchor_min_ransac_inliers': 20,
                    'phase7_loop_anchor_min_projection_matches': 20,
                    'phase7_loop_anchor_min_inlier_ratio': 0.25,
                    'phase7_loop_anchor_max_mean_error_m': 0.45,
                    'phase7_loop_anchor_max_error_m': 2.00,
                    'phase7_loop_anchor_min_final_inliers_relaxed_support': 40,
                    'phase7_loop_anchor_max_translation_norm_m': 1000.0,
                    'phase7_loop_anchor_log_rejections': False,
                    
                    # Fase 8C: pesos de loops según madurez del anchor.
                    'phase8_use_maturity_aware_loop_weights': True,

                    'phase8_loop_weight_intra_fusion': 80.0,
                    'phase8_loop_weight_intra_opt_only': 25.0,

                    'phase8_loop_weight_inter_fusion_confirmed': 80.0,
                    'phase8_loop_weight_inter_opt_only_confirmed': 20.0,

                    'phase8_loop_weight_inter_fusion_provisional': 25.0,
                    'phase8_loop_weight_inter_opt_only_provisional': 10.0,

                    'phase8_loop_weight_min': 1.0,
                    'phase8_loop_weight_max': 100.0,
                    
                    # Fase 8B: promoción de anchors loop provisionales a confirmados.
                    'anchor_maturity_enable_promotion': False,

                    'anchor_maturity_min_inter_opt_loops': 2,
                    'anchor_maturity_min_inter_fusion_loops': 1,

                    'anchor_maturity_min_inter_supported_groups': 50,
                    'anchor_maturity_min_inter_union_pairs': 150,
                    
                    # ============================================================
                    # Fase E: guardia física para anchors por loop inter-dron.
                    #
                    # Evita que un loop visual entre dos drones a distinta altura
                    # cree un anchor que colapse la diferencia Z.
                    # ============================================================

                    'phase_e_enable_loop_anchor_guard': True,
                    # REAL-SCOPE1:
                    # No usar GT para aceptar/rechazar anchors por loop.
                    # GT queda reservado para fiduciales simulados.
                    'phase_e_use_gt_current_pose_guard': False,
                    'phase_e_anchor_guard_max_position_error_m': 2.20,
                    'phase_e_anchor_guard_max_xy_error_m': 1.20,
                    'phase_e_anchor_guard_max_z_error_m': 1.85,
                    
                    # ============================================================
                    # Fase F: consenso de transformaciones para anchors inter-dron.
                    #
                    # Evita que un único loop fusion=1 cree un anchor provisional.
                    # Inicialmente pedimos 2 loops consistentes.
                    # Si sigue habiendo anchors malos, subir a 3.
                    # ============================================================

                    'phase_f_loop_anchor_require_consensus': True,
                    'phase_f_loop_anchor_consensus_min_cluster_size': 2,
                    'phase_f_loop_anchor_consensus_max_translation_spread_m': 0.60,
                    'phase_f_loop_anchor_consensus_max_yaw_spread_deg': 15.0,
                    'phase_f_loop_anchor_consensus_log_details': True,
                    
                    # ============================================================
                    # Fase G-fix:
                    # Los inter-loops sí pueden entrar al pose graph, pero solo
                    # cuando ambos submapas sean estables:
                    #   - FIDUCIAL_DIRECT
                    #   - LOOP_CONFIRMED
                    #
                    # Si uno de los dos es LOOP_PROVISIONAL, el loop se mantiene
                    # para anchoring/fusión, pero no deforma el grafo.
                    # ============================================================

                    'phase_g_use_inter_drone_loop_edges_for_pose_graph': True,
                    'phase_g_inter_loop_pose_graph_require_stable_anchors': True,

                    'anchor_maturity_max_chi2_per_edge': 2.5,
                    
                    # Fase 8B-fix: política dura de anchor inter-dron.
                    # No aceptar anchors por loops opt-only débiles.
                    'phase8_loop_anchor_require_fusion': True,

                    # Primera prueba: mantener False.
                    # Solo se activará más adelante si queremos permitir anchors opt-only
                    # extremadamente fuertes.
                    'phase8_loop_anchor_allow_strong_opt_only': False,

                    'phase8_loop_anchor_strong_opt_only_min_final_inliers': 50,
                    'phase8_loop_anchor_strong_opt_only_min_ransac_inliers': 25,
                    'phase8_loop_anchor_strong_opt_only_min_projection_matches': 80,
                    'phase8_loop_anchor_strong_opt_only_min_inlier_ratio': 0.45,
                    'phase8_loop_anchor_strong_opt_only_max_mean_error_m': 0.35,
                    'phase8_loop_anchor_strong_opt_only_max_error_m': 0.65,
                    
                    # Permitir que el mapa fused publique puntos de submapas
                    # anclados por loop provisional. No afecta a pose corrected.
                    'fused_publish_loop_provisional_landmarks': False,
                    
                    'enable_fused_cloud_publication': True,

                    # =====================================================
                    # FASE 1:
                    # Nueva ruta simple para /global_sparse_map_fused.
                    #
                    # "simple":
                    #   publica puntos fuertes de submapas anclados/publicables
                    #   y fusiona duplicados de forma barata.
                    #
                    # "legacy":
                    #   vuelve a la ruta antigua FUSION5*.
                    # =====================================================
                    'enable_fused_cloud_publication': True,
                    'fused_cloud_mode': 'simple',

                    # La ruta simple no usa la caché legacy como semántica
                    # principal. Este parámetro queda para legacy.
                    'fused_cloud_min_recompute_period_s': 4.0,

                    'fused_simple_period_s': 2.0,

                    # Fusión por proximidad + descriptor.
                    'fused_simple_merge_radius_m': 0.12,
                    'fused_simple_max_group_spread_m': 0.30,
                    'fused_simple_max_hamming': 70,
                    
                    # Fase 2A - radios diferenciados para fusionar landmarks.
                    # Mismo dron/mismo epoch: conservador.
                    # Mismo dron/distinto epoch: medio.
                    # Distinto dron: más permisivo para corregir offset residual.
                    'fused_simple_merge_radius_intra_m': 0.10,
                    'fused_simple_merge_radius_cross_epoch_m': 0.16,
                    'fused_simple_merge_radius_inter_m': 0.28,

                    'fused_simple_max_group_spread_intra_m': 0.18,
                    'fused_simple_max_group_spread_cross_epoch_m': 0.25,
                    'fused_simple_max_group_spread_inter_m': 0.35,

                    'fused_simple_max_hamming_intra': 65,
                    'fused_simple_max_hamming_cross_epoch': 75,
                    'fused_simple_max_hamming_inter': 80,
                    
                    # Fase 2B - eliminar singletons redundantes cerca de grupos fusionados.
                    # Esto reduce ruido cuando el segundo dron publica puntos en zonas ya vistas.
                    'fused_simple_suppress_singletons_near_merged': True,
                    'fused_simple_singleton_suppression_radius_m': 0.18,
                    'fused_simple_singleton_suppression_max_hamming': 85,

                    # True = solo usamos grupos merged inter-dron como referencia de supresión.
                    # Es más seguro: evita que un grupo intra-dron borre puntos nuevos del otro dron.
                    'fused_simple_singleton_suppression_require_inter_merged': True,
                    
                    # ========================================================
                    # TRACKS-1B:
                    # Tracks persistentes optimizados.
                    #
                    # Cambios respecto a TRACKS-1:
                    # - score inicial de singleton mucho más bajo;
                    # - threshold de publicación más alto;
                    # - score sube más despacio;
                    # - singletons necesitan calidad y varias reobservaciones;
                    # - búsqueda track-observación usa índice espacial.
                    # ========================================================
                    'fused_tracks_enabled': True,

                    'fused_track_publish_score_threshold': 0.60,

                    'fused_track_initial_score_singleton': 0.07,
                    'fused_track_initial_score_merged': 0.38,

                    'fused_track_seen_gain': 0.04,
                    'fused_track_merged_bonus': 0.04,
                    'fused_track_inter_drone_bonus': 0.08,
                    'fused_track_confirmed_bonus': 0.04,
                    'fused_track_provisional_penalty': 0.10,

                    # Asociación observación -> track.
                    'fused_track_assoc_radius_intra_m': 0.12,
                    'fused_track_assoc_radius_cross_epoch_m': 0.18,
                    'fused_track_assoc_radius_inter_m': 0.30,

                    'fused_track_assoc_hamming_intra': 70,
                    'fused_track_assoc_hamming_cross_epoch': 80,
                    'fused_track_assoc_hamming_inter': 90,

                    # Control de movimiento de landmarks.
                    'fused_track_max_update_step_weak_m': 0.06,
                    'fused_track_max_update_step_strong_m': 0.02,
                    'fused_track_strong_score_threshold': 0.70,

                    'fused_track_max_descriptor_samples': 40,
                    'fused_track_max_tracks': 20000,
                    
                    # Singletons persistentes.
                    # No todo singleton entra al mapa persistente.
                    'fused_track_use_singleton_observations': True,
                    'fused_track_singleton_min_observations': 12,
                    'fused_track_singleton_min_found_ratio': 0.60,
                    'fused_track_singleton_min_reobservations_to_publish': 4,

                    # Índice espacial de tracks.
                    'fused_track_spatial_cell_size_m': 0.30,
                    
                    # ========================================================
                    # TRACKS-PERF1A
                    #
                    # Mantiene recompute_period=4.0 para estabilidad, pero
                    # evita que cada recompute procese todos los puntos si el
                    # mapa crece mucho.
                    #
                    # No borra tracks persistentes. Solo limita observaciones
                    # nuevas por tick.
                    # ========================================================
                    'fused_tracks_perf1_enabled': True,

                    # Reduce merge_ms.
                    'fused_tracks_perf1_max_candidates_per_submap': 4500,
                    'fused_tracks_perf1_max_total_candidates': 9000,

                    # Reduce tracks_ms.
                    'fused_tracks_perf1_max_observations_per_tick': 2000,

                    # Prioriza puntos/grupos buenos antes de capar.
                    'fused_tracks_perf1_prioritize_by_quality': True,
                    
                    # ========================================================
                    # TRACKS-PERF1B
                    #
                    # Selección de candidatos por zona activa.
                    # Los tracks ya aceptados siguen publicándose globalmente,
                    # pero las observaciones nuevas se priorizan cerca de los
                    # drones activos. Esto ayuda a escalar cuando el edificio
                    # completo tenga muchos más puntos.
                    # ========================================================
                    'fused_tracks_perf1b_active_zone_enabled': True,
                    'fused_tracks_perf1b_active_radius_m': 5.0,
                    'fused_tracks_perf1b_max_pose_age_s': 3.0,

                    # No usar GT para fused/active-zone.
                    # La zona activa debe venir de ORB corregido.
                    'fused_tracks_perf1b_use_gt_pose': False,
                    'fused_tracks_perf1b_use_orb_pose_fallback': True,

                    # Caps de candidatos nuevos antes del merge.
                    'fused_tracks_perf1b_max_active_candidates_per_submap': 3500,
                    'fused_tracks_perf1b_max_background_candidates_per_submap': 600,
                    'fused_tracks_perf1b_max_total_candidates': 7500,
                    
                    # ========================================================
                    # TRACKS-PERF1C-safe
                    #
                    # Filtro conservador de artefactos cerca de la trayectoria.
                    # Busca eliminar puntos espurios que aparecen siguiendo el
                    # recorrido del dron, sin tocar landmarks estructurales.
                    # ========================================================
                    # Desactivado temporalmente:
                    # no queremos filtrar landmarks usando trayectoria GT.
                    'fused_tracks_perf1c_trajectory_filter_enabled': True,
                    'fused_tracks_perf1c_trajectory_radius_xy_m': 0.55,
                    'fused_tracks_perf1c_trajectory_radius_z_m': 0.75,
                    'fused_tracks_perf1c_trajectory_max_age_s': 18.0,
                    'fused_tracks_perf1c_trajectory_min_separation_m': 0.15,
                    'fused_tracks_perf1c_trajectory_max_points_per_drone': 140,

                    # Conservador: solo filtra puntos débiles.
                    'fused_tracks_perf1c_filter_only_low_quality': True,
                    'fused_tracks_perf1c_low_quality_max_observations': 20,
                    'fused_tracks_perf1c_low_quality_max_found_ratio': 0.80,
                    
                    # ========================================================
                    # ORB-TRAJ2 / FREE-SPACE1
                    #
                    # Filtra puntos débiles que aparecen delante de la cámara,
                    # entre el dron y la pared, usando solo pose ORB corregida.
                    # No usa ground truth.
                    # ========================================================
                    'fused_tracks_free_space_filter_enabled': True,
                    'fused_tracks_free_space_min_depth_m': 0.25,
                    'fused_tracks_free_space_max_depth_m': 3.25,
                    'fused_tracks_free_space_base_radius_m': 0.32,
                    'fused_tracks_free_space_radius_per_depth': 0.16,
                    'fused_tracks_free_space_max_pose_age_s': 12.0,

                    'fused_tracks_free_space_only_low_quality': True,
                    'fused_tracks_free_space_low_quality_max_observations': 25,
                    'fused_tracks_free_space_low_quality_max_found_ratio': 0.82,
                    
                    # ========================================================
                    # TRACKS-STAB1
                    #
                    # Evita que singletons muevan geometría de tracks fuertes,
                    # merged o inter-dron. Esto reduce giros/jitter cuando un
                    # dron se queda quieto y ORB-SLAM3 sigue refinando puntos.
                    # ========================================================
                    'fused_tracks_stab_freeze_singleton_geometry_on_strong_tracks': True,
                    'fused_tracks_stab_strong_track_score': 0.65,
                    'fused_tracks_stab_singleton_weight_scale': 0.10,
                    'fused_tracks_stab_singleton_max_update_step_m': 0.005,
                    
                    # ========================================================
                    # TRACKS-MERGE1 / PERF1C-safe
                    #
                    # Consolidación track-to-track para eliminar dobles paredes.
                    # No toca anchors ni pose graph. Fusiona landmarks persistentes
                    # cercanos y compatibles por descriptor.
                    # ========================================================
                    'fused_tracks_merge_enabled': True,

                    # Muy estricto intra; más permisivo inter-dron porque
                    # la doble pared suele venir de dos submapas/drones.
                    'fused_tracks_merge_radius_intra_m': 0.040,
                    'fused_tracks_merge_radius_cross_epoch_m': 0.070,
                    'fused_tracks_merge_radius_inter_m': 0.120,

                    'fused_tracks_merge_hamming_intra': 55,
                    'fused_tracks_merge_hamming_cross_epoch': 75,
                    'fused_tracks_merge_hamming_inter': 90,

                    'fused_tracks_merge_min_score': 0.20,
                    'fused_tracks_merge_max_merges_per_tick': 300,
                    'fused_tracks_merge_cell_size_m': 0.12,
                    
                    # ========================================================
                    # TRACKS2A - score negativo básico.
                    #
                    # Todavía no es expected-visible real por proyección a cámara.
                    # Es un proxy seguro:
                    # - si el track no se actualiza, decae suavemente;
                    # - si hay observaciones actuales cerca y no se asocia,
                    #   se penaliza más;
                    # - si baja mucho y es débil, se elimina.
                    # ========================================================
                    'fused_track_score_decay_enabled': False,

                    'fused_track_stale_decay_merged': 0.015,
                    'fused_track_stale_decay_singleton': 0.040,
                    'fused_track_stale_decay_min_age_s': 3.0,

                    'fused_track_expected_miss_proxy_enabled': True,
                    'fused_track_expected_miss_radius_m': 0.40,
                    'fused_track_expected_miss_min_nearby_observations': 3,
                    'fused_track_expected_miss_penalty': 0.08,
                    
                    # ========================================================
                    # TRACKS2B-A - expected-visible real por proyección.
                    #
                    # IMPORTANTE:
                    # diag_only=True en esta fase.
                    # Primero queremos comprobar si la proyección a KeyFrames
                    # recientes tiene sentido antes de usarla para bajar score.
                    # ========================================================
                    'fused_track_expected_visible_enabled': False,
                    'fused_track_expected_visible_diag_only': True,

                    'fused_track_expected_visible_max_kfs_per_drone': 8,
                    'fused_track_expected_visible_min_kfs': 2,

                    'fused_track_expected_visible_min_depth_m': 0.20,
                    'fused_track_expected_visible_max_depth_m': 18.0,
                    'fused_track_expected_visible_margin_px': 25.0,

                    'fused_track_expected_visible_penalty': 0.04,
                    'fused_track_expected_visible_min_score': 0.20,
                    'fused_track_expected_visible_max_tracks_per_tick': 1500,

                    'fused_track_remove_low_score_tracks': True,
                    'fused_track_remove_score_threshold': 0.08,
                    'fused_track_remove_min_age_s': 8.0,

                    # Publicación conservadora de tracks jóvenes.
                    'fused_track_merged_min_reobservations_to_publish': 2,
                    'fused_track_inter_min_reobservations_to_publish': 1,

                    # Filtro suave de entrada de candidatos.
                    # No subir demasiado aquí, porque los puntos débiles
                    # pueden servir para formar grupos merged.
                    'fused_simple_min_observations': 3,
                    'fused_simple_min_found_ratio': 0.25,

                    # Fase 1A.2:
                    # Endurecemos singletons porque la prueba anterior
                    # mostró mucho ruido visual.
                    'fused_simple_singleton_min_observations': 10,
                    'fused_simple_singleton_min_found_ratio': 0.45,

                    # Fase 1A.2:
                    # Densidad más estricta para eliminar puntos aislados.
                    'fused_simple_density_filter_enabled': True,
                    'fused_simple_density_radius_m': 0.32,
                    'fused_simple_density_min_neighbors': 6,

                    # Los grupos fusionados siguen publicándose aunque
                    # no pasen densidad, porque ya tienen soporte interno.
                    'fused_simple_keep_merged_without_density': True,

                    # Fase 1A.2:
                    # Desactivado de momento. En la prueba anterior dejó
                    # pasar puntos aislados de alta calidad aparente.
                    'fused_simple_keep_high_quality_singletons_without_density': False,
                    'fused_simple_hq_singleton_min_observations': 10,
                    'fused_simple_hq_singleton_min_found_ratio': 0.50,

                    # Fase 1A.2:
                    # Para validar un solo dron no publicamos todavía
                    # submapas LOOP_OR_PROPAGATED + PROVISIONAL.
                    # Esto se reactivará en Fase 1B.
                    # =====================================================
                    # FASE 1B:
                    # Permitimos que submapas LOOP_OR_PROPAGATED + PROVISIONAL
                    # publiquen en fused, pero con filtros más estrictos.
                    #
                    # Esto NO significa publicar MapCorrection definitiva.
                    # Esto NO significa usar el anchor provisional en pose graph.
                    # =====================================================
                    'fused_simple_publish_loop_provisional': False,

                    'fused_simple_provisional_min_observations': 3,
                    'fused_simple_provisional_min_found_ratio': 0.25,

                    'fused_simple_provisional_singleton_min_observations': 7,
                    'fused_simple_provisional_singleton_min_found_ratio': 0.40,
                    'fused_simple_provisional_density_min_neighbors': 6,

                    'fused_simple_max_provisional_candidates_per_submap': 1200,
                    'phase1b_anchor_policy_logs_enabled': True,

                    'fused_simple_voxel_enabled': True,
                    'fused_simple_voxel_size_m': 0.08,
                    'fused_simple_max_points': 3000,
                    'fused_simple_log_per_submap': True,
                    
                    'loop_fusion_log_events': ParameterValue(False, value_type=bool),
                    'loop_fusion_log_summary': ParameterValue(True, value_type=bool),
                    'loop_fusion_max_sample_logs': ParameterValue(5, value_type=int),
                    'loop_fusion_reinforcement_enabled': ParameterValue(False, value_type=bool),
                    
                    
                    # ============================================================
                    # FASE 5 original:
                    # Control iterativo de optimización por loop fuerte.
                    # ============================================================

                    'loop_opt_control_enabled': ParameterValue(True, value_type=bool),
                    'loop_opt_max_attempts_per_event': ParameterValue(3, value_type=int),
                    'loop_opt_min_abs_improvement_m': ParameterValue(0.03, value_type=float),
                    'loop_opt_min_relative_improvement': ParameterValue(0.10, value_type=float),
                    'loop_opt_retry_cooldown_s': ParameterValue(1.0, value_type=float),
                    'loop_opt_force_retry_optimization': ParameterValue(False, value_type=bool),
                    'loop_opt_log_events': ParameterValue(True, value_type=bool),
                    
                    # ============================================================
                    # FASE 5.1:
                    # Histéresis optimize/fuse para loops fuertes.
                    # ============================================================

                    'loop_error_hysteresis_enabled': ParameterValue(True, value_type=bool),
                    'loop_error_optimize_enter_t_m': ParameterValue(0.60, value_type=float),
                    'loop_error_optimize_enter_yaw_deg': ParameterValue(8.0, value_type=float),
                    'loop_error_fusion_exit_t_m': ParameterValue(0.40, value_type=float),
                    'loop_error_fusion_exit_yaw_deg': ParameterValue(5.0, value_type=float),
                    'loop_error_hysteresis_allow_middle_band_fusion': ParameterValue(True, value_type=bool),
                    'loop_error_hysteresis_log_events': ParameterValue(True, value_type=bool),
                    
                    # ============================================================
                    # FASE 5.2:
                    # Flag para pausar drones mientras el backend optimiza o está ocupado.
                    # ============================================================

                    'optimization_busy_flag_enabled': ParameterValue(True, value_type=bool),
                    'optimization_busy_publish_period_s': ParameterValue(0.20, value_type=float),
                    'optimization_busy_settle_time_s': ParameterValue(1.00, value_type=float),
                    'optimization_busy_fused_publish_ms_threshold': ParameterValue(450.0, value_type=float),
                    'optimization_busy_include_loop_pending': ParameterValue(True, value_type=bool),
                    'optimization_busy_include_fiducial_waiting': ParameterValue(False, value_type=bool),
                    'optimization_busy_include_slow_fused_publish': ParameterValue(True, value_type=bool),
                    
                    # ============================================================
                    # FASE 5.3:
                    # Busy robusto + anti-reoptimización innecesaria.
                    # ============================================================

                    'optimization_busy_background_thread_enabled': ParameterValue(True, value_type=bool),
                    'optimization_skip_stale_forced_requests': ParameterValue(True, value_type=bool),

                    'loop_opt_conflict_guard_enabled': ParameterValue(True, value_type=bool),
                    'loop_opt_conflict_error_t_m': ParameterValue(2.0, value_type=float),
                    'loop_opt_conflict_error_yaw_deg': ParameterValue(20.0, value_type=float),
                    'loop_opt_conflict_max_attempts': ParameterValue(1, value_type=int),
                    
                    # ============================================================
                    # FASE 6:
                    # Fusión explícita tras error bajo.
                    # ============================================================

                    'loop_fusion_score_boost_enabled': ParameterValue(True, value_type=bool),
                    'loop_fusion_score_gain_merged': ParameterValue(0.10, value_type=float),
                    'loop_fusion_score_gain_reobserve': ParameterValue(0.05, value_type=float),
                    'loop_fusion_source_track_merge_enabled': ParameterValue(True, value_type=bool),
                    'loop_fusion_log_score_updates': ParameterValue(True, value_type=bool),
                    'loop_fusion_max_score_update_logs_per_tick': ParameterValue(60, value_type=int),

                    # Reducir optimizaciones por pequeños lotes de priors fiduciales.
                    'fiducial_batch_new_priors_for_optimization_enabled': ParameterValue(True, value_type=bool),
                    'fiducial_min_new_priors_for_optimization': ParameterValue(5, value_type=int),
                    
                    # ============================================================
                    # FASE 6.1:
                    # Prioridad de deuda fiducial frente a batch skip.
                    # ============================================================

                    'fiducial_batch_skip_respects_optimization_debt': ParameterValue(True, value_type=bool),
                    'fiducial_debt_log_events': ParameterValue(True, value_type=bool),
                    
                    # ============================================================
                    # FASE 6.2:
                    # No optimizar fiducial si el error actual es bajo.
                    # Rechazar loops absurdamente conflictivos antes de optimizar.
                    # ============================================================

                    'fiducial_low_error_skip_optimization_enabled': ParameterValue(True, value_type=bool),
                    'fiducial_low_error_skip_max_t_m': ParameterValue(0.30, value_type=float),
                    'fiducial_low_error_skip_max_yaw_deg': ParameterValue(5.0, value_type=float),
                    'fiducial_low_error_skip_require_existing_world': ParameterValue(True, value_type=bool),
                    'fiducial_low_error_skip_log_events': ParameterValue(True, value_type=bool),

                    'loop_opt_pre_reject_conflicts_enabled': ParameterValue(True, value_type=bool),
                    'loop_opt_pre_reject_error_t_m': ParameterValue(2.0, value_type=float),
                    'loop_opt_pre_reject_error_yaw_deg': ParameterValue(45.0, value_type=float),
                    'loop_opt_pre_reject_error_rot_deg': ParameterValue(60.0, value_type=float),
                    'loop_opt_pre_reject_log_events': ParameterValue(True, value_type=bool),
                    
                    # ============================================================
                    # FASE 6.3:
                    # Optimizar solo si hay deuda real.
                    # ============================================================

                    'optimize_only_with_real_debt': ParameterValue(True, value_type=bool),
                    'advance_processed_counters_on_low_error_skip': ParameterValue(True, value_type=bool),
                    'loop_new_edges_require_optimization_debt': ParameterValue(True, value_type=bool),
                    'fiducial_new_priors_require_optimization_debt': ParameterValue(True, value_type=bool),
                    'real_debt_log_events': ParameterValue(True, value_type=bool),
                    
                    # ============================================================
                    # FASE A:
                    # Separar evento de acción.
                    # ============================================================

                    'phase_a_decision_gate_enabled': ParameterValue(True, value_type=bool),

                    # LoopDebt alto NO llama al optimizador global en esta fase.
                    # Solo se loguea como tarea local pendiente/diagnóstica.
                    'phase_a_loop_debt_diagnostic_only': ParameterValue(True, value_type=bool),

                    # El optimizador global solo puede arrancar por deuda fiducial explícita.
                    'phase_a_global_opt_only_for_fiducial_debt': ParameterValue(True, value_type=bool),

                    # Evita que new_loop_edges acumulados activen optimización global.
                    'phase_a_advance_loop_counters_when_local_task_deferred': ParameterValue(True, value_type=bool),

                    # Evita que new_fiducial_priors sin deuda acumulen optimizaciones inútiles.
                    'phase_a_advance_fiducial_counters_when_no_debt': ParameterValue(True, value_type=bool),

                    # Busy solo por tarea explícita, no por force residual.
                    'phase_a_busy_only_for_explicit_tasks': ParameterValue(True, value_type=bool),

                    'phase_a_log_decisions': ParameterValue(True, value_type=bool),
                    
                    # ============================================================
                    # FASE B:
                    # Diagnóstico de cantidad de loops.
                    # ============================================================

                    'phase_b_loop_throughput_enabled': ParameterValue(True, value_type=bool),
                    'phase_b_loop_throughput_period_s': ParameterValue(2.0, value_type=float),
                    'phase_b_log_kf_funnel': ParameterValue(True, value_type=bool),
                    'phase_b_max_kf_funnel_logs_per_period': ParameterValue(40, value_type=int),
                    
                    # ============================================================
                    # FASE C:
                    # Medición explícita del error vista actual ↔ mapa.
                    # ============================================================

                    'phase_c_viewmap_error_enabled': ParameterValue(True, value_type=bool),
                    'phase_c_log_viewmap_error': ParameterValue(True, value_type=bool),
                    'phase_c_log_stored_false_viewmap_error': ParameterValue(True, value_type=bool),
                    'phase_c_log_post_opt_viewmap_error': ParameterValue(True, value_type=bool),
                    

                }
            ]
        )
    )

    for drone_id in range(1, n_drones + 1):
        drone_name = f'{namespace_base}_{drone_id}'

        actions.append(
            GroupAction([
                PushRosNamespace(drone_name),

                Node(
                    package='orbslam3_server',
                    executable='global_pose_corrector',
                    name='global_pose_corrector',
                    output='screen',
                    parameters=[
                        {
                            'use_sim_time': ParameterValue(True, value_type=bool),
                            'drone_id': ParameterValue(drone_id, value_type=int),
                            'world_frame': ParameterValue('world', value_type=str),
                            # Fase 4A
                            'use_corrected_keyframes': True,
                            'max_nearest_kf_distance_m': 2.0,

                            'max_map_correction_age_s': 2.0,
                            'publish_pose_when_correction_stale': False,
                            'map_correction_stale_warn_age_s': 2.0,
                            'map_correction_hard_timeout_s': 30.0,
                            'publish_using_last_correction_when_stale': True,

                            'max_corrected_keyframes_age_s': 5.0,
                            'allow_rigid_fallback_when_corrected_kfs_stale': True,

                            'warn_output_jump_translation_m': 0.75,
                            'warn_output_jump_yaw_deg': 25.0,
                            
                            # Fase 4B
                            'enable_pose_smoothing': True,
                            'publish_smoothed_pose_on_main_topic': True,
                            'pose_smoothing_alpha': 0.25,
                            'max_smoothed_translation_step_m': 0.20,
                            'max_smoothed_yaw_step_deg': 8.0,
                            'reset_smoothing_if_raw_jump_m': 8.0,
                            'reset_smoothing_if_raw_jump_yaw_deg': 120.0,
                            'reset_smoothing_if_no_publish_s': 2.0,
                            
                            # Fase 4C
                            'publish_body_pose': True,

                            'body_T_camera_x': 0.10,
                            'body_T_camera_y': 0.03,
                            'body_T_camera_z': 0.03,
                            
                            'use_camera_optical_frame_convention': True,

                            # Provisional. Confirmar según frame exacto de ORB-SLAM/wrapper.
                            'body_T_camera_roll_deg': 0.0,
                            'body_T_camera_pitch_deg': -90.0,
                            'body_T_camera_yaw_deg': 90.0,
                        }
                    ]
                )
            ])
        )

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('n_drones', default_value='1'),
        DeclareLaunchArgument('namespace_base', default_value='dron'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        OpaqueFunction(function=launch_setup),
    ])

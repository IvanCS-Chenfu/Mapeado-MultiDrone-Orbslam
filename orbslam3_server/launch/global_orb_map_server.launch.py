from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # F1B: launch activo minimo del servidor global. Conserva el nombre publico
    # del launch para que `simulacion_dron/multi_dron.launch.py` no tenga que
    # cambiar, pero solo arranca el receptor de OrbMap.
    return LaunchDescription([
        # F1B: estos argumentos controlan la topologia ROS usada para construir
        # `/dron_X/orbslam/orb_map_delta` dentro del nodo.
        DeclareLaunchArgument('n_drones', default_value='1'),
        DeclareLaunchArgument('namespace_base', default_value='dron'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('rawdb_record_enabled', default_value='true'),
        DeclareLaunchArgument(
            'rawdb_record_path',
            default_value='src/codex/archivos_auxiliares/repeticiones/rawdb_prueba_1.record'),
        DeclareLaunchArgument('rawdb_replay_enabled', default_value='false'),
        DeclareLaunchArgument(
            'rawdb_replay_path',
            default_value='src/codex/archivos_auxiliares/repeticiones/rawdb_prueba_1.record'),
        DeclareLaunchArgument('rawdb_replay_period_sec', default_value='0.5'),
        DeclareLaunchArgument('pose_store_debug_enabled', default_value='false'),
        DeclareLaunchArgument('pose_store_debug_anchor_after_deltas', default_value='5'),
        DeclareLaunchArgument('pose_store_debug_anchor_drone_id', default_value='1'),
        DeclareLaunchArgument('pose_store_debug_anchor_epoch', default_value='0'),
        DeclareLaunchArgument('pose_store_debug_anchor_world_x', default_value='2.0'),
        DeclareLaunchArgument('pose_store_debug_anchor_world_y', default_value='0.0'),
        DeclareLaunchArgument('pose_store_debug_anchor_world_z', default_value='0.0'),
        DeclareLaunchArgument('pose_store_debug_anchor_yaw', default_value='0.0'),
        DeclareLaunchArgument('pose_store_debug_opt_enabled', default_value='false'),
        DeclareLaunchArgument('pose_store_debug_opt_after_deltas', default_value='10'),
        DeclareLaunchArgument('pose_store_debug_opt_kf_id', default_value='0'),
        DeclareLaunchArgument('pose_store_debug_opt_dx', default_value='0.15'),
        DeclareLaunchArgument('pose_store_debug_opt_dy', default_value='-0.03'),
        DeclareLaunchArgument('pose_store_debug_opt_dz', default_value='0.0'),
        DeclareLaunchArgument('pose_store_debug_opt_dyaw', default_value='0.05'),
        DeclareLaunchArgument('fiducial_sim_enabled', default_value='true'),
        DeclareLaunchArgument('fiducial_gt_max_dt_sec', default_value='1.0'),
        DeclareLaunchArgument('fiducial_gt_buffer_max_samples', default_value='250'),
        DeclareLaunchArgument('fiducial_revisit_error_threshold_m', default_value='0.35'),
        DeclareLaunchArgument('fiducial_revisit_yaw_threshold_rad', default_value='0.25'),
        DeclareLaunchArgument('fiducial_revisit_rot_threshold_rad', default_value='0.35'),
        DeclareLaunchArgument('body_T_camera_x', default_value='0.10'),
        DeclareLaunchArgument('body_T_camera_y', default_value='0.03'),
        DeclareLaunchArgument('body_T_camera_z', default_value='0.03'),
        DeclareLaunchArgument('body_T_camera_roll_deg', default_value='0.0'),
        DeclareLaunchArgument('body_T_camera_pitch_deg', default_value='-90.0'),
        DeclareLaunchArgument('body_T_camera_yaw_deg', default_value='90.0'),
        DeclareLaunchArgument('use_camera_optical_frame_convention', default_value='true'),
        DeclareLaunchArgument('global_sparse_cloud_topic', default_value='/global_sparse_cloud'),
        DeclareLaunchArgument('global_map_min_score_to_publish', default_value='0.0'),
        DeclareLaunchArgument('global_map_publish_period_sec', default_value='1.0'),
        DeclareLaunchArgument('f1g_full_snapshot_enabled', default_value='true'),
        DeclareLaunchArgument('f1g_full_snapshot_startup_delay_sec', default_value='35.0'),
        DeclareLaunchArgument('f1g_full_snapshot_period_sec', default_value='35.0'),
        DeclareLaunchArgument('f1g_debug_mark_optimized_kf', default_value='true'),
        DeclareLaunchArgument('pose_graph_max_vertices', default_value='24'),
        DeclareLaunchArgument('pose_graph_min_vertices', default_value='2'),
        DeclareLaunchArgument('pose_graph_max_path_length', default_value='80'),
        DeclareLaunchArgument('pose_graph_anchor_stop_enabled', default_value='true'),
        DeclareLaunchArgument('pose_graph_include_temporal_edges', default_value='true'),
        DeclareLaunchArgument('pose_graph_temporal_edge_weight', default_value='25.0'),
        DeclareLaunchArgument('pose_graph_temporal_edge_weight_sparse', default_value='10.0'),
        DeclareLaunchArgument('pose_graph_fiducial_hard_weight', default_value='1000000.0'),
        DeclareLaunchArgument(
            'pose_graph_fiducial_target_translation_weight',
            default_value='5000.0'),
        DeclareLaunchArgument(
            'pose_graph_fiducial_target_rotation_weight',
            default_value='2500.0'),
        DeclareLaunchArgument('pose_graph_current_pose_soft_weight', default_value='5.0'),
        DeclareLaunchArgument(
            'pose_graph_fiducial_neighborhood_radius_m',
            default_value='4.0'),
        DeclareLaunchArgument(
            'pose_graph_fiducial_neighborhood_radius_kfs',
            default_value='3'),
        DeclareLaunchArgument('pose_graph_max_temporal_edge_kf_gap', default_value='15'),
        DeclareLaunchArgument('pose_graph_max_temporal_edge_length_m', default_value='4.0'),
        DeclareLaunchArgument('pose_graph_corner_yaw_threshold_rad', default_value='0.5235987756'),
        DeclareLaunchArgument('f1i_debug_force_task_enabled', default_value='false'),
        DeclareLaunchArgument('f1i_debug_task_dx', default_value='0.75'),
        DeclareLaunchArgument('f1i_debug_task_dy', default_value='0.0'),
        DeclareLaunchArgument('f1i_debug_task_dz', default_value='0.0'),
        DeclareLaunchArgument('f1i_debug_task_dyaw', default_value='0.20'),
        DeclareLaunchArgument('f1j_dryrun_enabled', default_value='true'),
        DeclareLaunchArgument('f1j_dryrun_min_improvement_ratio', default_value='0.05'),
        DeclareLaunchArgument(
            'f1j_dryrun_partial_min_improvement_ratio',
            default_value='0.70'),
        DeclareLaunchArgument('f1j_dryrun_max_allowed_delta_t', default_value='30.0'),
        DeclareLaunchArgument(
            'f1j_dryrun_partial_max_allowed_delta_t',
            default_value='35.0'),
        DeclareLaunchArgument('f1j_dryrun_max_allowed_delta_yaw', default_value='1.2'),
        DeclareLaunchArgument('f1j_dryrun_max_final_error_t', default_value='0.35'),
        DeclareLaunchArgument('f1j_dryrun_require_cost_decrease', default_value='true'),
        DeclareLaunchArgument('f1j_solver_step_fraction', default_value='0.95'),
        DeclareLaunchArgument('f1j_export_debug_plot', default_value='false'),
        DeclareLaunchArgument(
            'f1j_debug_output_dir',
            default_value='src/codex/archivos_auxiliares'),
        DeclareLaunchArgument('f1k_apply_enabled', default_value='true'),
        DeclareLaunchArgument('f1l_validation_enabled', default_value='true'),
        DeclareLaunchArgument('f1l_partial_apply_enabled', default_value='true'),
        DeclareLaunchArgument('f1l_max_partial_retries', default_value='3'),
        DeclareLaunchArgument('f1l_debug_force_reject_once', default_value='false'),
        DeclareLaunchArgument('f1l_debug_force_reject_task_id', default_value='-1'),
        DeclareLaunchArgument(
            'f1l_post_apply_internal_broken_edge_t',
            default_value='2.5'),
        DeclareLaunchArgument(
            'f1l_post_apply_internal_max_growth_t',
            default_value='1.5'),
        DeclareLaunchArgument(
            'f1l_post_apply_fiducial_absurd_error_t',
            default_value='10.0'),
        DeclareLaunchArgument('f1l_gt_kf_debug_enabled', default_value='false'),
        DeclareLaunchArgument('f1l_gt_kf_debug_max_dt_sec', default_value='1.0'),
        DeclareLaunchArgument('f1l_debug_animation_enabled', default_value='false'),
        DeclareLaunchArgument(
            'f1l_debug_animation_output_dir',
            default_value='src/codex/archivos_auxiliares/html'),
        DeclareLaunchArgument('f1l_graph_dump_enabled', default_value='false'),
        DeclareLaunchArgument(
            'f1l_graph_dump_output_dir',
            default_value='src/codex/archivos_auxiliares/repeticiones'),

        # F1B: no se lanzan correctores de pose porque el servidor minimo aun no
        # publica `MapCorrection` ni `CorrectedKeyFrameArray`. Se reintroduciran
        # en una subfase posterior cuando exista backend global validado.
        Node(
            package='orbslam3_server',
            executable='global_map_server',
            name='global_orb_map_server',
            output='screen',
            parameters=[
                {
                    'use_sim_time': ParameterValue(
                        LaunchConfiguration('use_sim_time'),
                        value_type=bool),
                    'world_frame': ParameterValue('world', value_type=str),
                    'n_drones': ParameterValue(
                        LaunchConfiguration('n_drones'),
                        value_type=int),
                    'namespace_base': ParameterValue(
                        LaunchConfiguration('namespace_base'),
                        value_type=str),
                    'f1b_stats_period_s': ParameterValue(2.0, value_type=float),
                    'rawdb_record_enabled': ParameterValue(
                        LaunchConfiguration('rawdb_record_enabled'),
                        value_type=bool),
                    'rawdb_record_path': ParameterValue(
                        LaunchConfiguration('rawdb_record_path'),
                        value_type=str),
                    'rawdb_replay_enabled': ParameterValue(
                        LaunchConfiguration('rawdb_replay_enabled'),
                        value_type=bool),
                    'rawdb_replay_path': ParameterValue(
                        LaunchConfiguration('rawdb_replay_path'),
                        value_type=str),
                    'rawdb_replay_period_sec': ParameterValue(
                        LaunchConfiguration('rawdb_replay_period_sec'),
                        value_type=float),
                    # F1D: parametros de prueba controlada de GlobalPoseStore.
                    # Por defecto quedan apagados; 1E sera quien cree anchors
                    # reales mediante fiduciales.
                    'pose_store_debug_enabled': ParameterValue(
                        LaunchConfiguration('pose_store_debug_enabled'),
                        value_type=bool),
                    'pose_store_debug_anchor_after_deltas': ParameterValue(
                        LaunchConfiguration('pose_store_debug_anchor_after_deltas'),
                        value_type=int),
                    'pose_store_debug_anchor_drone_id': ParameterValue(
                        LaunchConfiguration('pose_store_debug_anchor_drone_id'),
                        value_type=int),
                    'pose_store_debug_anchor_epoch': ParameterValue(
                        LaunchConfiguration('pose_store_debug_anchor_epoch'),
                        value_type=int),
                    'pose_store_debug_anchor_world_x': ParameterValue(
                        LaunchConfiguration('pose_store_debug_anchor_world_x'),
                        value_type=float),
                    'pose_store_debug_anchor_world_y': ParameterValue(
                        LaunchConfiguration('pose_store_debug_anchor_world_y'),
                        value_type=float),
                    'pose_store_debug_anchor_world_z': ParameterValue(
                        LaunchConfiguration('pose_store_debug_anchor_world_z'),
                        value_type=float),
                    'pose_store_debug_anchor_yaw': ParameterValue(
                        LaunchConfiguration('pose_store_debug_anchor_yaw'),
                        value_type=float),
                    'pose_store_debug_opt_enabled': ParameterValue(
                        LaunchConfiguration('pose_store_debug_opt_enabled'),
                        value_type=bool),
                    'pose_store_debug_opt_after_deltas': ParameterValue(
                        LaunchConfiguration('pose_store_debug_opt_after_deltas'),
                        value_type=int),
                    'pose_store_debug_opt_kf_id': ParameterValue(
                        LaunchConfiguration('pose_store_debug_opt_kf_id'),
                        value_type=int),
                    'pose_store_debug_opt_dx': ParameterValue(
                        LaunchConfiguration('pose_store_debug_opt_dx'),
                        value_type=float),
                    'pose_store_debug_opt_dy': ParameterValue(
                        LaunchConfiguration('pose_store_debug_opt_dy'),
                        value_type=float),
                    'pose_store_debug_opt_dz': ParameterValue(
                        LaunchConfiguration('pose_store_debug_opt_dz'),
                        value_type=float),
                    'pose_store_debug_opt_dyaw': ParameterValue(
                        LaunchConfiguration('pose_store_debug_opt_dyaw'),
                        value_type=float),
                    # F1E/F1H: fiduciales simulados iniciales. El servidor usa
                    # GT solo para construir observaciones fiduciales, no para
                    # mapa, loops ni pose final. La prueba tipica larga usa el
                    # par legacy: fiducial 1 en (0, 9, 1) y fiducial 2 en
                    # (0, -9, 1).
                    'fiducial_sim_enabled': ParameterValue(
                        LaunchConfiguration('fiducial_sim_enabled'),
                        value_type=bool),
                    'fiducial_gt_max_dt_sec': ParameterValue(
                        LaunchConfiguration('fiducial_gt_max_dt_sec'),
                        value_type=float),
                    'fiducial_gt_buffer_max_samples': ParameterValue(
                        LaunchConfiguration('fiducial_gt_buffer_max_samples'),
                        value_type=int),
                    # F1H: umbrales para clasificar una segunda observacion
                    # fiducial como OK o como tarea pendiente. No activan solver.
                    'fiducial_revisit_error_threshold_m': ParameterValue(
                        LaunchConfiguration('fiducial_revisit_error_threshold_m'),
                        value_type=float),
                    'fiducial_revisit_yaw_threshold_rad': ParameterValue(
                        LaunchConfiguration('fiducial_revisit_yaw_threshold_rad'),
                        value_type=float),
                    'fiducial_revisit_rot_threshold_rad': ParameterValue(
                        LaunchConfiguration('fiducial_revisit_rot_threshold_rad'),
                        value_type=float),
                    # F1F-hotfix: extrinseca cuerpo-camara heredada del launch
                    # antiguo. El backend la usa para convertir GT body a pose
                    # camera antes de crear anchors fiduciales.
                    'body_T_camera_x': ParameterValue(
                        LaunchConfiguration('body_T_camera_x'),
                        value_type=float),
                    'body_T_camera_y': ParameterValue(
                        LaunchConfiguration('body_T_camera_y'),
                        value_type=float),
                    'body_T_camera_z': ParameterValue(
                        LaunchConfiguration('body_T_camera_z'),
                        value_type=float),
                    'body_T_camera_roll_deg': ParameterValue(
                        LaunchConfiguration('body_T_camera_roll_deg'),
                        value_type=float),
                    'body_T_camera_pitch_deg': ParameterValue(
                        LaunchConfiguration('body_T_camera_pitch_deg'),
                        value_type=float),
                    'body_T_camera_yaw_deg': ParameterValue(
                        LaunchConfiguration('body_T_camera_yaw_deg'),
                        value_type=float),
                    'use_camera_optical_frame_convention': ParameterValue(
                        LaunchConfiguration('use_camera_optical_frame_convention'),
                        value_type=bool),
                    # F1F: publicacion inicial de nube sparse global con score.
                    # El score lo calcula orbslam3_multi; este nodo solo publica
                    # PointCloud2 en el frame global.
                    'global_sparse_cloud_topic': ParameterValue(
                        LaunchConfiguration('global_sparse_cloud_topic'),
                        value_type=str),
                    'global_map_min_score_to_publish': ParameterValue(
                        LaunchConfiguration('global_map_min_score_to_publish'),
                        value_type=float),
                    'global_map_publish_period_sec': ParameterValue(
                        LaunchConfiguration('global_map_publish_period_sec'),
                        value_type=float),
                    # F1G: peticiones periodicas de full snapshot al wrapper.
                    # El snapshot resincroniza RawMapDatabase sin borrar estado
                    # global optimizado en GlobalPoseStore.
                    'f1g_full_snapshot_enabled': ParameterValue(
                        LaunchConfiguration('f1g_full_snapshot_enabled'),
                        value_type=bool),
                    'f1g_full_snapshot_startup_delay_sec': ParameterValue(
                        LaunchConfiguration('f1g_full_snapshot_startup_delay_sec'),
                        value_type=float),
                    'f1g_full_snapshot_period_sec': ParameterValue(
                        LaunchConfiguration('f1g_full_snapshot_period_sec'),
                        value_type=float),
                    'f1g_debug_mark_optimized_kf': ParameterValue(
                        LaunchConfiguration('f1g_debug_mark_optimized_kf'),
                        value_type=bool),
                    # F1I: parametros del PoseGraphBuilder temporal. No
                    # habilitan optimizacion ni aplicacion de poses; solo
                    # controlan tamano de ventana, aristas y pesos loggeados.
                    'pose_graph_max_vertices': ParameterValue(
                        LaunchConfiguration('pose_graph_max_vertices'),
                        value_type=int),
                    'pose_graph_min_vertices': ParameterValue(
                        LaunchConfiguration('pose_graph_min_vertices'),
                        value_type=int),
                    'pose_graph_max_path_length': ParameterValue(
                        LaunchConfiguration('pose_graph_max_path_length'),
                        value_type=int),
                    'pose_graph_anchor_stop_enabled': ParameterValue(
                        LaunchConfiguration('pose_graph_anchor_stop_enabled'),
                        value_type=bool),
                    'pose_graph_include_temporal_edges': ParameterValue(
                        LaunchConfiguration('pose_graph_include_temporal_edges'),
                        value_type=bool),
                    'pose_graph_temporal_edge_weight': ParameterValue(
                        LaunchConfiguration('pose_graph_temporal_edge_weight'),
                        value_type=float),
                    'pose_graph_temporal_edge_weight_sparse': ParameterValue(
                        LaunchConfiguration('pose_graph_temporal_edge_weight_sparse'),
                        value_type=float),
                    'pose_graph_fiducial_hard_weight': ParameterValue(
                        LaunchConfiguration('pose_graph_fiducial_hard_weight'),
                        value_type=float),
                    'pose_graph_fiducial_target_translation_weight': ParameterValue(
                        LaunchConfiguration(
                            'pose_graph_fiducial_target_translation_weight'),
                        value_type=float),
                    'pose_graph_fiducial_target_rotation_weight': ParameterValue(
                        LaunchConfiguration(
                            'pose_graph_fiducial_target_rotation_weight'),
                        value_type=float),
                    'pose_graph_current_pose_soft_weight': ParameterValue(
                        LaunchConfiguration('pose_graph_current_pose_soft_weight'),
                        value_type=float),
                    'pose_graph_fiducial_neighborhood_radius_m': ParameterValue(
                        LaunchConfiguration(
                            'pose_graph_fiducial_neighborhood_radius_m'),
                        value_type=float),
                    'pose_graph_fiducial_neighborhood_radius_kfs': ParameterValue(
                        LaunchConfiguration(
                            'pose_graph_fiducial_neighborhood_radius_kfs'),
                        value_type=int),
                    'pose_graph_max_temporal_edge_kf_gap': ParameterValue(
                        LaunchConfiguration('pose_graph_max_temporal_edge_kf_gap'),
                        value_type=int),
                    'pose_graph_max_temporal_edge_length_m': ParameterValue(
                        LaunchConfiguration('pose_graph_max_temporal_edge_length_m'),
                        value_type=float),
                    'pose_graph_corner_yaw_threshold_rad': ParameterValue(
                        LaunchConfiguration('pose_graph_corner_yaw_threshold_rad'),
                        value_type=float),
                    # F1I: modo de prueba replay cuando el record disponible no
                    # contiene un residual fiducial alto. Crea una tarea
                    # sintetica solo para construir/loggear el grafo.
                    'f1i_debug_force_task_enabled': ParameterValue(
                        LaunchConfiguration('f1i_debug_force_task_enabled'),
                        value_type=bool),
                    'f1i_debug_task_dx': ParameterValue(
                        LaunchConfiguration('f1i_debug_task_dx'),
                        value_type=float),
                    'f1i_debug_task_dy': ParameterValue(
                        LaunchConfiguration('f1i_debug_task_dy'),
                        value_type=float),
                    'f1i_debug_task_dz': ParameterValue(
                        LaunchConfiguration('f1i_debug_task_dz'),
                        value_type=float),
                    'f1i_debug_task_dyaw': ParameterValue(
                        LaunchConfiguration('f1i_debug_task_dyaw'),
                        value_type=float),
                    # F1J/F1K/F1L: dry-run, apply y validacion post-apply.
                    # 1L permite rollback controlado y apply parcial seguro.
                    'f1j_dryrun_enabled': ParameterValue(
                        LaunchConfiguration('f1j_dryrun_enabled'),
                        value_type=bool),
                    'f1j_dryrun_min_improvement_ratio': ParameterValue(
                        LaunchConfiguration('f1j_dryrun_min_improvement_ratio'),
                        value_type=float),
                    'f1j_dryrun_partial_min_improvement_ratio': ParameterValue(
                        LaunchConfiguration(
                            'f1j_dryrun_partial_min_improvement_ratio'),
                        value_type=float),
                    'f1j_dryrun_max_allowed_delta_t': ParameterValue(
                        LaunchConfiguration('f1j_dryrun_max_allowed_delta_t'),
                        value_type=float),
                    'f1j_dryrun_partial_max_allowed_delta_t': ParameterValue(
                        LaunchConfiguration('f1j_dryrun_partial_max_allowed_delta_t'),
                        value_type=float),
                    'f1j_dryrun_max_allowed_delta_yaw': ParameterValue(
                        LaunchConfiguration('f1j_dryrun_max_allowed_delta_yaw'),
                        value_type=float),
                    'f1j_dryrun_max_final_error_t': ParameterValue(
                        LaunchConfiguration('f1j_dryrun_max_final_error_t'),
                        value_type=float),
                    'f1j_dryrun_require_cost_decrease': ParameterValue(
                        LaunchConfiguration('f1j_dryrun_require_cost_decrease'),
                        value_type=bool),
                    'f1j_solver_step_fraction': ParameterValue(
                        LaunchConfiguration('f1j_solver_step_fraction'),
                        value_type=float),
                    'f1j_export_debug_plot': ParameterValue(
                        LaunchConfiguration('f1j_export_debug_plot'),
                        value_type=bool),
                    'f1j_debug_output_dir': ParameterValue(
                        LaunchConfiguration('f1j_debug_output_dir'),
                        value_type=str),
                    'f1k_apply_enabled': ParameterValue(
                        LaunchConfiguration('f1k_apply_enabled'),
                        value_type=bool),
                    'f1l_validation_enabled': ParameterValue(
                        LaunchConfiguration('f1l_validation_enabled'),
                        value_type=bool),
                    'f1l_partial_apply_enabled': ParameterValue(
                        LaunchConfiguration('f1l_partial_apply_enabled'),
                        value_type=bool),
                    'f1l_max_partial_retries': ParameterValue(
                        LaunchConfiguration('f1l_max_partial_retries'),
                        value_type=int),
                    'f1l_debug_force_reject_once': ParameterValue(
                        LaunchConfiguration('f1l_debug_force_reject_once'),
                        value_type=bool),
                    'f1l_debug_force_reject_task_id': ParameterValue(
                        LaunchConfiguration('f1l_debug_force_reject_task_id'),
                        value_type=int),
                    'f1l_post_apply_internal_broken_edge_t': ParameterValue(
                        LaunchConfiguration('f1l_post_apply_internal_broken_edge_t'),
                        value_type=float),
                    'f1l_post_apply_internal_max_growth_t': ParameterValue(
                        LaunchConfiguration('f1l_post_apply_internal_max_growth_t'),
                        value_type=float),
                    'f1l_post_apply_fiducial_absurd_error_t': ParameterValue(
                        LaunchConfiguration(
                            'f1l_post_apply_fiducial_absurd_error_t'),
                        value_type=float),
                    'f1l_gt_kf_debug_enabled': ParameterValue(
                        LaunchConfiguration('f1l_gt_kf_debug_enabled'),
                        value_type=bool),
                    'f1l_gt_kf_debug_max_dt_sec': ParameterValue(
                        LaunchConfiguration('f1l_gt_kf_debug_max_dt_sec'),
                        value_type=float),
                    'f1l_debug_animation_enabled': ParameterValue(
                        LaunchConfiguration('f1l_debug_animation_enabled'),
                        value_type=bool),
                    'f1l_debug_animation_output_dir': LaunchConfiguration(
                        'f1l_debug_animation_output_dir'),
                    'f1l_graph_dump_enabled': ParameterValue(
                        LaunchConfiguration('f1l_graph_dump_enabled'),
                        value_type=bool),
                    'f1l_graph_dump_output_dir': LaunchConfiguration(
                        'f1l_graph_dump_output_dir'),
                    'fiducials.ids': [1, 2],
                    'fiducials.x': [0.0, 0.0],
                    'fiducials.y': [9.0, -9.0],
                    'fiducials.z': [1.0, 1.0],
                    'fiducials.yaw': [0.0, 0.0],
                    'fiducials.radius': [2.0, 2.0],
                }
            ],
        ),
    ])

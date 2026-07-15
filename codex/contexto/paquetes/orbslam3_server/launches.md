# Launches de `orbslam3_server`

## Launch activo

```text
orbslam3_server/launch/global_orb_map_server.launch.py
```

Desde `1B`, este launch es mínimo y lanza solo:

```text
/global_orb_map_server
```

Ejecutable:

```text
global_map_server
```

El launch activo debe mantenerse comentado con trazabilidad por subfase. Desde `1C` también expone parámetros de grabación y replay raw, desde `1E` expone parámetros del fiducial simulado, desde `1F` expone parámetros de publicación de nube sparse global y de extrínseca `body_T_camera`, desde `1G` expone parámetros de full snapshots, desde `1H` expone umbrales de revisit fiducial, desde `1I` expone parámetros de `PoseGraphBuilder`, desde `1J` expone dry-run/dump/HTML diagnóstico, desde `1K` expone apply/publicación y desde `1L` expone diagnóstico post-apply. Tras `1H`, el launch activo recupera los dos fiduciales legacy para poder ejecutar la prueba tipica larga de rodeo del edificio.

## Argumentos

| Argumento | Default | Uso |
|---|---|---|
| `n_drones` | `1` | Número de drones esperados. |
| `namespace_base` | `dron` | Prefijo para construir namespaces `/dron_X`. |
| `use_sim_time` | `true` | Reloj de simulación. |
| `rawdb_record_enabled` | `true` | Graba `RawMapDatabase` durante simulación normal. |
| `rawdb_record_path` | `src/codex/archivos_auxiliares/repeticiones/rawdb_prueba_1.record` | Dataset raw de salida. |
| `rawdb_replay_enabled` | `false` | Activa replay sin subscribers a wrappers. |
| `rawdb_replay_path` | `src/codex/archivos_auxiliares/repeticiones/rawdb_prueba_1.record` | Dataset raw de entrada. |
| `rawdb_replay_period_sec` | `0.5` | Ritmo de reinyectado del journal. |
| `pose_store_debug_enabled` | `false` | Activa prueba controlada de `GlobalPoseStore` introducida en `1D`. |
| `pose_store_debug_anchor_after_deltas` | `5` | Umbral de entradas para anchor sintético. |
| `pose_store_debug_anchor_drone_id` | `1` | Dron del submapa anclado en modo debug. |
| `pose_store_debug_anchor_epoch` | `0` | Epoch del submapa anclado en modo debug. |
| `pose_store_debug_anchor_world_x/y/z/yaw` | `2.0/0.0/0.0/0.0` | Transformación `world_T_local` sintética. |
| `pose_store_debug_opt_enabled` | `false` | Activa registro de pose optimizada sintética. |
| `pose_store_debug_opt_after_deltas` | `10` | Umbral de entradas para corrección sintética. |
| `pose_store_debug_opt_kf_id` | `0` | KF local preferido para la prueba de optimización sintética. |
| `pose_store_debug_opt_dx/dy/dz/dyaw` | `0.15/-0.03/0.0/0.05` | Delta sintético usado para generar `correction_T_latest`. |
| `fiducial_sim_enabled` | `true` | Activa GT de Gazebo como detección fiducial simulada en live. |
| `fiducial_gt_max_dt_sec` | `1.0` | Ventana temporal máxima para asociar GT con timestamp de KF. |
| `fiducial_gt_buffer_max_samples` | `250` | Muestras GT retenidas por dron. |
| `fiducial_revisit_error_threshold_m` | `0.35` | Umbral traslacional para revisits fiduciales de `1H`. |
| `fiducial_revisit_yaw_threshold_rad` | `0.25` | Umbral yaw para revisits fiduciales de `1H`. |
| `fiducial_revisit_rot_threshold_rad` | `0.35` | Umbral rotacional para revisits fiduciales de `1H`. |
| `global_sparse_cloud_topic` | `/global_sparse_cloud` | Topic `PointCloud2` de nube sparse global. |
| `global_map_min_score_to_publish` | `0.0` | Umbral mínimo de score para publicación. |
| `global_map_publish_period_sec` | `1.0` | Periodo de publicación de nube sparse global. |
| `body_T_camera_x/y/z` | `0.10/0.03/0.03` | Traslación fija cuerpo-cámara usada para corregir anchors fiduciales. |
| `body_T_camera_roll/pitch/yaw_deg` | `0.0/-90.0/90.0` | RPY documentado para la extrínseca cuerpo-cámara. |
| `use_camera_optical_frame_convention` | `true` | Usa la convención óptica legacy al construir la rotación de `body_T_camera`. |
| `f1g_full_snapshot_enabled` | `true` | Activa requests de full snapshot a `/dron_X/orbslam/get_full_map` en modo live. |
| `f1g_full_snapshot_startup_delay_sec` | `35.0` | Retardo inicial antes de pedir snapshots. |
| `f1g_full_snapshot_period_sec` | `35.0` | Periodo de resincronización por full snapshot. |
| `f1g_debug_mark_optimized_kf` | `true` | Marca una pose optimizada debug para validar `KEEP_OPTIMIZED` en 1G. |
| `pose_graph_max_vertices` | `24` | Maximo de vertices del grafo temporal 1I/1L. |
| `pose_graph_min_vertices` | `2` | Minimo de vertices requerido por el builder. |
| `pose_graph_max_path_length` | `80` | Longitud maxima de ventana alrededor del KF objetivo. |
| `pose_graph_anchor_stop_enabled` | `true` | Permite detener ventana en KFs hard fiducial. |
| `pose_graph_include_temporal_edges` | `true` | Activa aristas temporales. |
| `pose_graph_max_temporal_edge_kf_gap` | `15` | Gap maximo deseado entre vertices de control consecutivos. |
| `pose_graph_max_temporal_edge_length_m` | `4.0` | Longitud maxima deseada de un tramo entre vertices de control. |
| `pose_graph_corner_yaw_threshold_rad` | `0.5235987756` | Umbral de giro para insertar vertices en esquinas/derivas angulares. |
| `pose_graph_temporal_edge_weight` | `25.0` | Peso de arista temporal cercana. |
| `pose_graph_temporal_edge_weight_sparse` | `10.0` | Peso de arista temporal entre muestras separadas. |
| `pose_graph_fiducial_hard_weight` | `1000000.0` | Peso de prior hard fiducial. |
| `pose_graph_fiducial_target_translation_weight` | `5000.0` | Peso traslacional del prior objetivo. |
| `pose_graph_fiducial_target_rotation_weight` | `2500.0` | Peso rotacional del prior objetivo. |
| `pose_graph_current_pose_soft_weight` | `5.0` | Peso de prior suave a pose actual. |
| `f1i_debug_force_task_enabled` | `false` | Activa tarea debug de grafo tras replay. |
| `f1i_debug_task_dx/dy/dz/dyaw` | `0.75/0.0/0.0/0.20` | Delta de tarea debug 1I. |
| `f1j_dryrun_enabled` | `true` | Activa dry-run de `OptimizationManager`. |
| `f1j_export_debug_plot` | `false` | Exporta SVG opcional de debug; no es canal normal de datos. |
| `f1k_apply_enabled` | `true` | Activa apply en `GlobalPoseStore`. |
| `f1l_validation_enabled` | `true` | Activa validación post-apply y backup/rollback. |
| `f1l_partial_apply_enabled` | `true` | Nombre legacy: permite apply controlado de candidatos; propiedad conceptual de `1K`. |
| `f1l_debug_force_reject_once` | `false` | Fuerza un rechazo controlado para probar rollback. |
| `f1l_debug_force_reject_task_id` | `-1` | Limita el rechazo debug a un `task_id` concreto si se configura. |
| `f1l_gt_kf_debug_enabled` | `false` | Activa metricas GT por KeyFrame para diagnostico de `1L`; solo logs, no solver ni publicacion. |
| `f1l_gt_kf_debug_max_dt_sec` | `1.0` | Maxima diferencia temporal para asociar GT a un KF en la base debug de `1L`. |
| `f1l_debug_animation_enabled` | `false` | Nombre legacy: activa export HTML animado del grafo; propiedad conceptual de `1J`. |
| `f1l_debug_animation_output_dir` | `src/codex/archivos_auxiliares/html` | Directorio donde se escriben `f1l_debug_animation_task_<task_id>.html`. |
| `f1l_graph_dump_enabled` | `false` | Nombre legacy: activa guardado del `PoseGraphProblem` para replay offline de `1J`. |
| `f1l_graph_dump_output_dir` | `src/codex/archivos_auxiliares/repeticiones` | Directorio donde se escriben `f1l_graph_task_<task_id>.tsv`. |

## Parámetros pasados al servidor activo

```text
use_sim_time
world_frame=world
n_drones
namespace_base
f1b_stats_period_s=2.0
rawdb_record_enabled
rawdb_record_path
rawdb_replay_enabled
rawdb_replay_path
rawdb_replay_period_sec
pose_store_debug_enabled
pose_store_debug_anchor_after_deltas
pose_store_debug_anchor_drone_id
pose_store_debug_anchor_epoch
pose_store_debug_anchor_world_x
pose_store_debug_anchor_world_y
pose_store_debug_anchor_world_z
pose_store_debug_anchor_yaw
pose_store_debug_opt_enabled
pose_store_debug_opt_after_deltas
pose_store_debug_opt_kf_id
pose_store_debug_opt_dx
pose_store_debug_opt_dy
pose_store_debug_opt_dz
pose_store_debug_opt_dyaw
fiducial_sim_enabled
fiducial_gt_max_dt_sec
fiducial_gt_buffer_max_samples
fiducial_revisit_error_threshold_m
fiducial_revisit_yaw_threshold_rad
fiducial_revisit_rot_threshold_rad
fiducials.ids=[1,2]
fiducials.x=[0.0,0.0]
fiducials.y=[9.0,-9.0]
fiducials.z=[1.0,1.0]
fiducials.yaw=[0.0,0.0]
fiducials.radius=[2.0,2.0]
global_sparse_cloud_topic
global_map_min_score_to_publish
global_map_publish_period_sec
body_T_camera_x
body_T_camera_y
body_T_camera_z
body_T_camera_roll_deg
body_T_camera_pitch_deg
body_T_camera_yaw_deg
use_camera_optical_frame_convention
f1g_full_snapshot_enabled
f1g_full_snapshot_startup_delay_sec
f1g_full_snapshot_period_sec
f1g_debug_mark_optimized_kf
pose_graph_max_vertices
pose_graph_min_vertices
pose_graph_max_path_length
pose_graph_anchor_stop_enabled
pose_graph_include_temporal_edges
pose_graph_max_temporal_edge_kf_gap
pose_graph_max_temporal_edge_length_m
pose_graph_corner_yaw_threshold_rad
pose_graph_temporal_edge_weight
pose_graph_temporal_edge_weight_sparse
pose_graph_fiducial_hard_weight
pose_graph_fiducial_target_translation_weight
pose_graph_fiducial_target_rotation_weight
pose_graph_current_pose_soft_weight
f1i_debug_force_task_enabled
f1i_debug_task_dx
f1i_debug_task_dy
f1i_debug_task_dz
f1i_debug_task_dyaw
f1j_dryrun_enabled
f1j_export_debug_plot
f1k_apply_enabled
f1l_validation_enabled
f1l_partial_apply_enabled
f1l_debug_force_reject_once
f1l_debug_force_reject_task_id
f1l_gt_kf_debug_enabled
f1l_gt_kf_debug_max_dt_sec
f1l_debug_animation_enabled
f1l_debug_animation_output_dir
f1l_graph_dump_enabled
f1l_graph_dump_output_dir
```

## Launch legacy

```text
orbslam3_server/launch/global_orb_map_server_antiguo.launch.py
```

Es una copia congelada del launch monolítico anterior. No es el launch activo de `1B`.

## Correctores de pose

`global_pose_corrector` sigue existiendo y compila, pero el launch activo de `1B` no lo lanza porque el servidor mínimo no publica todavía `MapCorrection` ni `CorrectedKeyFrameArray`.

Cuando fases posteriores recuperen publicación de correcciones, el launch deberá reintroducir los correctores de forma explícita y documentada.

## Relación con simulación

`simulacion_dron/launch/multi_dron.launch.py` sigue incluyendo:

```text
orbslam3_server/launch/global_orb_map_server.launch.py
```

Por tanto, la simulación oficial usa el servidor nuevo mínimo sin cambiar el nombre público del launch.

## Validación 1B

La validación por launch exige ver:

```text
[F1B-SERVER-INIT]
[F1B-SERVER-SUBSCRIBED]
[F1B-ORBMAP-RX]
```

No se espera nube global ni visualización RViz2 útil en `1B`.

## Validación 1C

La validación por launch exige ver:

```text
[F1C-RAWDB-INIT]
[F1C-RAWDB-SAVE]
[F1C-REPLAY-LOAD]
[F1C-REPLAY-DONE]
```

Replay validado:

```bash
ros2 launch orbslam3_server global_orb_map_server.launch.py n_drones:=2 namespace_base:=dron use_sim_time:=false rawdb_record_enabled:=false rawdb_replay_enabled:=true rawdb_replay_path:=src/codex/archivos_auxiliares/repeticiones/rawdb_prueba_1.record rawdb_replay_period_sec:=0.05
```

## Validación 1D

La validación por launch exige ver:

```text
[F1D-POSESTORE-INIT]
[F1D-POSESTORE-ANCHOR-SUMMARY]
[F1D-POSESTORE-OPT-POSE-SET]
[F1D-POSESTORE-CORRECTION-SET]
[F1D-POSESTORE-NEW-KF-CORRECTION-INHERIT]
[F1D-POSESTORE-STATS]
```

Replay validado:

```bash
ros2 launch orbslam3_server global_orb_map_server.launch.py n_drones:=2 namespace_base:=dron use_sim_time:=false rawdb_record_enabled:=false rawdb_replay_enabled:=true rawdb_replay_path:=src/codex/archivos_auxiliares/repeticiones/rawdb_prueba_1.record rawdb_replay_period_sec:=0.02 pose_store_debug_enabled:=true pose_store_debug_anchor_after_deltas:=5 pose_store_debug_anchor_drone_id:=1 pose_store_debug_anchor_epoch:=0 pose_store_debug_anchor_world_x:=2.0 pose_store_debug_anchor_world_y:=0.0 pose_store_debug_anchor_world_z:=0.0 pose_store_debug_anchor_yaw:=0.0 pose_store_debug_opt_enabled:=true pose_store_debug_opt_after_deltas:=10 pose_store_debug_opt_kf_id:=0 pose_store_debug_opt_dx:=0.15 pose_store_debug_opt_dy:=-0.03 pose_store_debug_opt_dz:=0.0 pose_store_debug_opt_dyaw:=0.05
```

## Validación 1E

La validación live usa el launch oficial de simulación, que incluye este launch:

```bash
ros2 launch simulacion_dron multi_dron.launch.py
```

La validación replay de `1E` desactiva GT fiducial vivo y reproduce el journal persistido:

```bash
ros2 launch orbslam3_server global_orb_map_server.launch.py n_drones:=2 namespace_base:=dron use_sim_time:=false rawdb_record_enabled:=false rawdb_replay_enabled:=true rawdb_replay_path:=src/codex/archivos_auxiliares/repeticiones/rawdb_prueba_1.record rawdb_replay_period_sec:=0.02 fiducial_sim_enabled:=false pose_store_debug_enabled:=false
```

Marcadores obligatorios:

```text
[F1E-FID-KF-ASSOC]
[F1E-FID-FIRST-ANCHOR]
[F1E-FID-KF-HARD]
[F1E-FID-JOURNAL-SAVE]
[F1E-FID-REPLAY-OBS]
[F1E-FID-STATS]
```

## Validación 1F

La validación live usa el launch oficial de simulación:

```bash
ros2 launch simulacion_dron multi_dron.launch.py
```

La validación replay lenta de `1F` reprodujo el `.record` a `1.0` s por delta. Tras el hotfix de `body_T_camera`, el dataset diferencial se llamó temporalmente `prueba_diff_anclaje.record`, pero se borró al iniciar `1G` a petición del usuario. El comando siguiente queda como evidencia histórica:

```bash
ros2 launch orbslam3_server global_orb_map_server.launch.py n_drones:=2 namespace_base:=dron use_sim_time:=false rawdb_record_enabled:=false rawdb_replay_enabled:=true rawdb_replay_path:=src/codex/archivos_auxiliares/repeticiones/prueba_diff_anclaje.record rawdb_replay_period_sec:=1.0 fiducial_sim_enabled:=false pose_store_debug_enabled:=false global_map_publish_period_sec:=1.0
```

## Validación 1H

La validación live usa el launch oficial de simulación:

```bash
ros2 launch simulacion_dron multi_dron.launch.py
```

La validación replay final de `1H` usa el record vigente con observaciones fiduciales y snapshots:

```bash
ros2 launch orbslam3_server global_orb_map_server.launch.py n_drones:=2 namespace_base:=dron use_sim_time:=false rawdb_record_enabled:=false rawdb_replay_enabled:=true rawdb_replay_path:=src/codex/archivos_auxiliares/repeticiones/rawdb_prueba_1.record rawdb_replay_period_sec:=0.05 fiducial_sim_enabled:=false pose_store_debug_enabled:=false f1g_full_snapshot_enabled:=false f1g_debug_mark_optimized_kf:=false global_map_publish_period_sec:=1.0
```

Marcadores obligatorios:

```text
[F1H-FID-REVISIT-CONFIG]
[F1H-FID-REVISIT]
[F1H-FID-POSE-ERROR]
[F1H-FID-OK]
[F1H-FID-TASK-STATS]
```

Si se valida visualmente `/global_sparse_cloud` en RViz2, comprobar primero que no haya dos servidores vivos:

```bash
ros2 topic info /global_sparse_cloud -v
```

En `1H` se encontro un replay antiguo que mantenia `Publisher count: 2`; al cerrarlo desaparecio la alternancia entre mapa nuevo y mapa antiguo.

Prueba tipica larga posterior a `1H`:

```bash
ros2 launch simulacion_dron multi_dron.launch.py rawdb_record_enabled:=false
```

Usada por `run_simulation.sh --prueba 3` con `tray_prueba_3.yaml`. La prueba completo el rodeo del edificio y valido que el launch activo con fiduciales legacy `[1,2]` detecta el fiducial 1. En el log completo `prueba_3.log` aparecen `[F1E-FID-KF-ASSOC] fid=1` y `[F1H-FID-TASK-CREATED]`; al volver al fiducial 2 aparecen revisits `[F1H-FID-OK]` en nuevos epochs. Esta prueba deja un caso util para subfases posteriores porque termina con `total=41 pending=41 confirmed_ok=65 high_error=41`.

Marcadores obligatorios:

```text
[F1F-BODY-CAMERA-CONFIG]
[F1F-BODY-CAMERA-APPLY]
[F1F-SCORE-UPDATE-ORBSLAM]
[F1F-SCORE-STATS]
[F1F-GLOBALMAP-BUILD]
[F1F-GLOBALMAP-POINT-STATS]
[F1F-GLOBALMAP-PUBLISH]
```

## Validación 1G

La validación live usa el launch oficial de simulación, que incluye este launch y deja activos los full snapshots:

```bash
ros2 launch simulacion_dron multi_dron.launch.py
```

La validación replay de `1G` reproduce el `.record` nuevo con snapshots ya guardados y desactiva nuevos requests de servicio:

```bash
ros2 launch orbslam3_server global_orb_map_server.launch.py n_drones:=2 namespace_base:=dron use_sim_time:=false rawdb_record_enabled:=false rawdb_replay_enabled:=true rawdb_replay_path:=src/codex/archivos_auxiliares/repeticiones/rawdb_prueba_1.record rawdb_replay_period_sec:=0.05 fiducial_sim_enabled:=false pose_store_debug_enabled:=false f1g_full_snapshot_enabled:=false f1g_debug_mark_optimized_kf:=true global_map_publish_period_sec:=1.0
```

Marcadores obligatorios:

```text
[F1G-FULL-SNAPSHOT-CLIENT-READY]
[F1G-FULL-SNAPSHOT-REQUEST]
[F1G-FULL-SNAPSHOT-RX]
[F1G-FULL-SNAPSHOT-ARRIVAL]
[F1G-RAWDB-INSERT-FULL]
[F1G-SNAPSHOT-RECONCILE]
[F1G-POSESTORE-RECONCILE-SUMMARY]
[F1G-REPLAY-FULL-SNAPSHOT]
```

La ejecución validada terminó con `SIM-DONE prueba=1 success=true`, `SIM-DONE prueba=2 success=true` y `[F1C-REPLAY-DONE] entries=368 journal=368 deltas=356 full=12 fiducial_observations=74 submaps=4 kfs=225 mps=26165`.

## Parametros 1L vigentes

`global_orb_map_server.launch.py` expone:

```text
pose_graph_fiducial_neighborhood_radius_kfs=3
f1l_max_partial_retries=3
f1l_post_apply_fiducial_absurd_error_t=10.0
```

La exportacion SVG sigue desactivada por defecto y no forma parte del transporte de resultados al servidor.

# `global_map_server.cpp`

## Rol actual

`orbslam3_server/src/global_map_server.cpp` es desde `1B` el servidor global activo mínimo, desde `1C` el adaptador que alimenta `orbslam3_multi::RawMapDatabase`, desde `1D` el punto de prueba controlada de `orbslam3_multi::GlobalPoseStore`, desde `1E` el adaptador ROS que construye observaciones fiduciales simuladas para `orbslam3_multi::FiducialAnchorManager`, desde `1F` el publicador de la primera nube sparse global, desde `1G` el cliente de full snapshots de ORB-SLAM3, desde `1H` el logger/configurador de revisits fiduciales, desde `1I` el invocador de `orbslam3_multi::PoseGraphBuilder`, desde `1J` el invocador de `orbslam3_multi::OptimizationManager` en modo dry-run y exportación diagnóstica, desde `1K` el invocador del apply seguro sobre `GlobalPoseStore`, propagación y publicación coherente, y desde `1L` el emisor de diagnóstico post-apply.

Estado de diseño 2026-07-12:

```text
`global_map_server` puede coordinar backup/apply/validacion, pero no debe
tratar como cerrada una optimizacion fiducial grande si el backend solo reparte
heuristicamente la correccion del fiducial objetivo. Para cerrar 1L, el servidor
debe recibir evidencia de solver real: coste minimizado, edges/priors usadas,
residuales internos acotados y pesos suaves por distancia/confianza a
fiduciales.
```

Desde el hotfix de `1F`, también pasa al backend la extrínseca `body_T_camera` para que los anchors fiduciales se calculen con la pose world de la cámara/KF y no directamente con la pose world del cuerpo del dron.

Su única responsabilidad funcional actual es:

```text
wrapper ORB-SLAM3
  -> /dron_X/orbslam/orb_map_delta
  -> global_map_server
  -> RawMapDatabase
  -> FiducialAnchorManager si hay KF asociado a fiducial
  -> LandmarkScoreManager + GlobalMapBuilder
  -> full snapshots /dron_X/orbslam/get_full_map
  -> reconciliacion RawMapDatabase + GlobalPoseStore
  -> medicion de revisits fiduciales y tareas pendientes 1H
  -> construccion/log de PoseGraphProblem temporal 1I
  -> dump opcional del PoseGraphProblem para replay offline 1J
  -> dry-run OptimizationManager 1J sin apply
  -> apply 1K de resultados useful=true en GlobalPoseStore
  -> apply/publicación 1K
  -> diagnóstico post-apply 1L
  -> logs F1B/F1C/F1E, guardado raw/fiducial y replay
  -> /global_sparse_cloud
  -> modo debug F1D opcional de GlobalPoseStore
```

No contiene la lógica monolítica anterior. Esa lógica quedó congelada en:

```text
orbslam3_server/src/global_map_server_antiguo.cpp
```

## Nodo

```text
global_orb_map_server
```

## Clase activa

El archivo define internamente:

```text
GlobalMapServer : rclcpp::Node
```

No incluye `include/orbslam3_server/global_map_server.hpp`. Ese header pertenece al servidor heredado y se conserva solo como referencia hasta que una subfase posterior decida limpiarlo o crear una interfaz nueva.

## Parámetros

| Parámetro | Default | Uso |
|---|---|---|
| `use_sim_time` | `false` | Lee el override del launch para trabajar con reloj simulado. |
| `world_frame` | `world` | Se conserva como contrato de frame para fases posteriores. |
| `namespace_base` | `dron` | Construye `/dron_X/orbslam/orb_map_delta`. |
| `n_drones` | `1` | Número de drones esperados. |
| `f1b_stats_period_s` | `2.0` | Periodo de estadísticas por log. |
| `rawdb_record_enabled` | `true` | Activa guardado periódico y final de `RawMapDatabase`. |
| `rawdb_record_path` | `src/codex/archivos_auxiliares/repeticiones/rawdb_prueba_1.record` | Ruta del dataset raw generado. |
| `rawdb_replay_enabled` | `false` | Arranca en modo replay y no crea subscribers a wrappers. |
| `rawdb_replay_path` | `src/codex/archivos_auxiliares/repeticiones/rawdb_prueba_1.record` | Dataset raw a cargar. |
| `rawdb_replay_period_sec` | `0.5` | Periodo del timer que reinyecta entradas del journal. |
| `pose_store_debug_enabled` | `false` | Activa el modo debug de `1D` para validar `GlobalPoseStore`; no es anclaje real. |
| `pose_store_debug_anchor_after_deltas` | `5` | Número de entradas de journal tras las que se intenta anchor sintético. |
| `pose_store_debug_anchor_drone_id` | `1` | Dron del submapa que se ancla sintéticamente. |
| `pose_store_debug_anchor_epoch` | `0` | Epoch del submapa que se ancla sintéticamente. |
| `pose_store_debug_anchor_world_x/y/z/yaw` | `2.0/0.0/0.0/0.0` | Transformación `world_T_local` sintética de prueba. |
| `pose_store_debug_opt_enabled` | `false` | Activa registro de pose optimizada sintética. |
| `pose_store_debug_opt_after_deltas` | `10` | Entrada de journal tras la que se registra la optimización sintética. |
| `pose_store_debug_opt_kf_id` | `0` | KF local preferido para registrar la pose optimizada sintética. |
| `pose_store_debug_opt_dx/dy/dz/dyaw` | `0.15/-0.03/0.0/0.05` | Delta sintético aplicado a la pose world del KF para calcular corrección heredable. |
| `fiducial_sim_enabled` | `true` | Activa la asociación simulada con fiduciales usando GT de Gazebo en modo live. En replay 1E se usa `false`. |
| `fiducial_gt_max_dt_sec` | `1.0` | Máxima diferencia temporal entre timestamp de KF y muestra GT candidata. |
| `fiducial_gt_buffer_max_samples` | `250` | Tamaño máximo del buffer temporal de GT por dron. |
| `fiducials.ids` | `[2]` | IDs de fiduciales simulados. |
| `fiducials.x/y/z/yaw/radius` | `[0.0]/[-9.0]/[1.0]/[0.0]/[2.0]` | Pose y radio de aceptación del fiducial simulado 2. |
| `fiducial_revisit_error_threshold_m` | `0.35` | Umbral traslacional para clasificar una segunda observacion fiducial en `1H`. |
| `fiducial_revisit_yaw_threshold_rad` | `0.25` | Umbral yaw para revisits fiduciales en `1H`. |
| `fiducial_revisit_rot_threshold_rad` | `0.35` | Umbral rotacional 3D para revisits fiduciales en `1H`. |
| `global_sparse_cloud_topic` | `/global_sparse_cloud` | Topic `PointCloud2` publicado por `1F`. |
| `global_map_min_score_to_publish` | `0.0` | Umbral de score mínimo aplicado por `GlobalMapBuilder`. |
| `global_map_publish_period_sec` | `1.0` | Periodo del timer de publicación de nube sparse global. |
| `body_T_camera_x/y/z` | `0.10/0.03/0.03` | Traslación fija del frame cuerpo al frame cámara usada para anclaje fiducial. |
| `body_T_camera_roll/pitch/yaw_deg` | `0.0/-90.0/90.0` | RPY documentado de la extrínseca; con convención óptica activa se conserva para trazabilidad del launch. |
| `use_camera_optical_frame_convention` | `true` | Replica la convención óptica del corrector legacy al construir `body_T_camera`. |
| `f1g_full_snapshot_enabled` | `true` | Activa clientes y timers de full snapshot en modo live. |
| `f1g_full_snapshot_startup_delay_sec` | `35.0` | Retardo antes de pedir el primer snapshot completo. |
| `f1g_full_snapshot_period_sec` | `35.0` | Periodo de resincronización por full snapshot. |
| `f1g_debug_mark_optimized_kf` | `true` | Marca una pose optimizada debug una sola vez para validar `KEEP_OPTIMIZED` en 1G. |
| `pose_graph_max_vertices` | `24` | Máximo nominal de vertices seleccionados por `PoseGraphBuilder` en `1I`; la cobertura puede ampliarlo de forma controlada. |
| `pose_graph_min_vertices` | `2` | Mínimo de vertices para aceptar un grafo 1I. |
| `pose_graph_max_path_length` | `80` | Longitud topológica máxima alrededor del KF objetivo. |
| `pose_graph_anchor_stop_enabled` | `true` | Permite cortar ventana en KFs hard fiducial. |
| `pose_graph_fiducial_connectivity_enabled` | `true` | Activa metadatos de conectividad fiducial auxiliares para preservar anclajes previos en `1I`. |
| `pose_graph_branch_selection_enabled` | `true` | Activa seleccion conservadora de anclajes frontera por rama temporal/fiducial. |
| `pose_graph_fiducial_neighborhood_radius_m` | `4.0` | Radio métrico de `1I` para seleccionar vecinos de cada fiducial; `1J` los pincha por pose inducida durante el solve. |
| `pose_graph_subdivision_candidate_radius_m` | `2.0` | Radio informativo para clasificacion conservadora de aristas fiduciales subdivisibles. |
| `pose_graph_max_temporal_edge_kf_gap` | `15` | Gap máximo deseado entre vertices de control consecutivos. |
| `pose_graph_max_temporal_edge_length_m` | `4.0` | Longitud máxima deseada de un tramo entre vertices de control. |
| `pose_graph_corner_yaw_threshold_rad` | `0.5235987756` | Umbral de yaw para insertar vertices de esquina. |
| `pose_graph_include_temporal_edges` | `true` | Activa aristas temporales iniciales. |
| `pose_graph_temporal_edge_weight` | `25.0` | Peso de aristas temporales cercanas. |
| `pose_graph_temporal_edge_weight_sparse` | `10.0` | Peso de aristas temporales entre muestras separadas. |
| `pose_graph_fiducial_hard_weight` | `1000000.0` | Peso de prior hard fiducial. |
| `pose_graph_fiducial_target_translation_weight` | `5000.0` | Peso traslacional del prior objetivo fiducial. |
| `pose_graph_fiducial_target_rotation_weight` | `2500.0` | Peso rotacional del prior objetivo fiducial. |
| `pose_graph_current_pose_soft_weight` | `5.0` | Peso de prior suave a pose actual. |
| `f1i_debug_force_task_enabled` | `false` | Crea una tarea debug de grafo tras replay si se activa. No optimiza ni aplica. |
| `f1i_debug_task_dx/dy/dz/dyaw` | `0.75/0.0/0.0/0.20` | Delta sintético de la tarea debug 1I. |
| `f1j_dryrun_enabled` | `true` | Activa `OptimizationManager::RunDryRun` tras construir un `PoseGraphProblem`. |
| `f1j_dryrun_min_improvement_ratio` | `0.05` | Mejora relativa minima para marcar `useful=true`. |
| `f1j_dryrun_partial_min_improvement_ratio` | `0.70` | Mejora relativa minima para marcar `partial_candidate=true` cuando el error final sigue sobre umbral. |
| `f1j_dryrun_max_allowed_delta_t` | `30.0` | Delta traslacional maximo permitido en el dry-run. |
| `f1j_dryrun_partial_max_allowed_delta_t` | `35.0` | Delta traslacional maximo para conservar un candidato parcial sin aceptarlo como `useful`. |
| `f1j_dryrun_max_allowed_delta_yaw` | `1.2` | Delta yaw maximo permitido en el dry-run. |
| `f1j_dryrun_max_final_error_t` | `0.35` | Error final traslacional maximo recomendado para aceptar utilidad. |
| `f1j_dryrun_require_cost_decrease` | `true` | Exige `final_cost < initial_cost`. |
| `f1j_solver_step_fraction` | `0.95` | Fraccion de la correccion temporal aplicada por el solver inicial de 1J. |
| `f1j_export_debug_plot` | `false` | Exporta SVG de debug solo si el usuario lo pide; no es canal normal de datos. |
| `f1j_debug_output_dir` | `src/codex/archivos_auxiliares` | Directorio donde se guardarian SVGs opcionales `f1j_dryrun_task_<id>.svg`. |
| `f1k_apply_enabled` | `true` | Activa apply real de dry-runs `useful=true` en `GlobalPoseStore`; desde `1L` los parciales pueden pasar por validacion controlada. |
| `f1l_validation_enabled` | `true` | Activa backup, validacion post-apply y decision `ACCEPT/PARTIAL/REJECT`. |
| `f1l_partial_apply_enabled` | `true` | Permite aplicar `partial_candidate=true` bajo backup obligatorio y validacion inmediata. |
| `f1l_debug_force_reject_once` | `false` | Fuerza un rechazo controlado una vez para probar rollback. |
| `f1l_debug_force_reject_task_id` | `-1` | Si es mayor o igual que cero, limita el rechazo debug a ese `task_id`. |
| `f1l_post_apply_internal_broken_edge_t` | `2.50` | Umbral de arista interna rota tras apply. |
| `f1l_post_apply_internal_max_growth_t` | `1.50` | Crecimiento maximo tolerado de error interno tras apply. |
| `f1l_post_apply_fiducial_absurd_error_t` | `10.0` | Umbral para tratar un residual fiducial como absurdo y activar politica de correccion progresiva/parcial. |
| `f1l_gt_kf_debug_enabled` | `false` | Activa metricas GT por KeyFrame solo para debug/test de `1L`; no alimenta el solver ni la publicacion. |
| `f1l_gt_kf_debug_max_dt_sec` | `1.0` | Maxima diferencia temporal permitida para asociar una muestra GT a un KF en la base debug. |
| `f1l_debug_animation_enabled` | `false` | Nombre legacy: exporta HTML diagnóstico del grafo, propiedad conceptual de `1J`. |
| `f1l_debug_animation_output_dir` | `src/codex/archivos_auxiliares/html` | Directorio de salida para `f1l_debug_animation_task_<task_id>.html`. |
| `f1l_graph_dump_enabled` | `false` | Nombre legacy: guarda el `PoseGraphProblem` exacto antes del dry-run para reproducir el solver offline de `1J`. |
| `f1l_graph_dump_output_dir` | `src/codex/archivos_auxiliares/repeticiones` | Directorio de salida para `f1l_graph_task_<task_id>.tsv`. |

## Funciones principales

| Función | Responsabilidad |
|---|---|
| `DeclareOrReadUseSimTime` | Lee o declara `use_sim_time` sin romper si ROS 2 ya lo declaró. |
| `CreateOrbMapSubscriptions` | Crea un subscriber por dron a `/dron_X/orbslam/orb_map_delta`. |
| `OnOrbMapDelta` | Callback ROS de `OrbMap`; solo registra metadatos y actualiza contadores. |
| `LogKeyFrameRange` | Loggea conteo y rango de IDs locales de KFs del delta. |
| `LogMapPointRange` | Loggea conteo y rango de IDs locales de MPs del delta. |
| `PublishStatsLog` | Emite acumulados por dron y por epoch. |
| `SaveRawDatabase` | Guarda el journal raw en `rawdb_record_path` durante ejecución y shutdown. |
| `LoadReplayDataset` | Carga un dataset raw y prepara replay por timer. |
| `ReplayNextEntry` | Reinyecta una entrada del journal en una `RawMapDatabase` nueva y emite logs de replay. |
| `HandlePoseStoreDebugAfterInsert` | En modo debug `1D`, conecta cada delta/replay ya insertado con `GlobalPoseStore`. |
| `TryPoseStoreDebugAnchor` | Aplica un anchor sintético `DEBUG_TEST` sobre un submapa existente del replay. |
| `TryPoseStoreDebugOptimization` | Registra una pose optimizada sintética `DEBUG_TEST_OPT` para probar corrección heredable. |
| `RegisterPoseStoreKeyFramesFromMap` | Registra KFs nuevos del delta si el submapa ya está anclado. |
| `LogPoseStoreStats` | Emite estadísticas `[F1D-POSESTORE-STATS]` y `[F1D-SERVER-POSESTORE-STATS]`. |
| `LoadFiducialConfig` | Lee parámetros de fiduciales simulados. |
| `ConfigureFiducialRevisit` | Lee umbrales de `1H` y configura `FiducialAnchorManager`. |
| `ConfigureBodyCameraTransform` | Lee `body_T_camera_*`, configura `GlobalPoseStore` y emite `[F1F-BODY-CAMERA-CONFIG]`. |
| `CreateGroundTruthSubscriptions` | Crea subscribers a `/dron_X/sensor/GT/pose` solo en modo live con `fiducial_sim_enabled=true`. |
| `OnGroundTruthPose` | Mantiene buffer temporal acotado de GT para asociación fiducial simulada. |
| `ProcessFiducialsForDelta` | Busca KFs del delta cerca del fiducial y construye observaciones para el backend usando `world_T_body_fiducial`. |
| `HandleFiducialObservation` | Llama a `FiducialAnchorManager`, guarda journal fiducial, loggea `body_t`/`camera_t` y desde `1H` loggea revisits, errores y tareas. |
| `ProcessReplayFiducials` | Reproduce observaciones fiduciales persistidas para un `arrival_id` sin consultar GT. |
| `UpdateScoresFromMap` | Alimenta `LandmarkScoreManager` desde MapPoints raw recibidos o reproducidos. |
| `LogScoreStats` | Emite estadísticas `[F1F-SCORE-STATS]`. |
| `BuildPointCloud2` | Convierte `GlobalSparsePoint` del backend a `sensor_msgs/msg/PointCloud2`. |
| `PublishGlobalSparseCloud` | Pide puntos a `GlobalMapBuilder` y publica `/global_sparse_cloud` para deltas/snapshots/replay; desde el hotfix de `1H`, `reason=timer` se delega a republicación cacheada. |
| `RepublishLastGlobalSparseCloud` | Republica la ultima nube cacheada para RViz2 sin reconstruir desde raw, evitando alternar estados antiguos con nuevos. |
| `CreateFullSnapshotClients` | Crea clientes `GetOrbMap` por dron para `/dron_X/orbslam/get_full_map`. |
| `RequestFullSnapshots` | Lanza una ronda de requests de snapshot completo con motivo trazable. |
| `RequestFullSnapshot` | Envía una llamada asíncrona a un wrapper concreto y controla que no haya request pendiente duplicada. |
| `OnFullSnapshotResponse` | Valida respuesta, asigna `arrival_id`, inserta snapshot en `RawMapDatabase` y dispara reconciliación. |
| `ProcessFullSnapshotAfterInsert` | Loggea resultado de `InsertFullSnapshot`, llama a `GlobalPoseStore` y reconstruye/publica mapa si procede. |
| `MaybeSetF1GDebugOptimizedKeyFrame` | Marca una pose optimizada debug para validar que un full snapshot no la pisa. |
| `ConfigurePoseGraphBuilder` | Configura `PoseGraphBuilder` con parámetros `pose_graph_*`, incluyendo conectividad fiducial/seleccion de ramas y radio métrico de vecindad fiducial de `1I`; conserva `[F1L-FIDUCIAL-NEIGHBORHOOD-CONFIG]` como marcador legacy. |
| `BuildPoseGraphsForPendingTasks` | Recorre tareas pendientes de `FiducialAnchorManager` y construye grafos F1I no repetidos. |
| `BuildAndLogPoseGraphForTask` | Llama a `PoseGraphBuilder::BuildForFiducialTask`, emite marcadores `[F1I-*]` y desde `1J` lanza el dry-run si procede. |
| `LogPoseGraphProblem` | Loggea ventana, vertices, fijos, variables, aristas, soporte/densidad por arista, priors, pesos, `PropagationPlan` y, desde la reapertura de `1I`, preservacion de anclajes fiduciales previos. |
| `ConfigureOptimizationManager` | Lee parametros `f1j_*` y `f1l_*`, configura `OptimizationManager` y deja `OptimizationDebugExporter` opt-in. |
| `PopulatePoseGraphDebugKeyFrames` | Añade al dump offline poses mapa y GT externo por KF de ventana. Es solo diagnóstico; el solver ignora `debug_keyframes`. |
| `DumpPoseGraphProblemForOffline` | Si `f1l_graph_dump_enabled=true`, guarda `f1l_graph_task_<task_id>.tsv` con el grafo que se iba a optimizar, soporte de aristas y `debug_keyframes`; emite `[F1L-GRAPH-DUMP]`. |
| `RunAndLogOptimizationDryRun` | Ejecuta `OptimizationManager::RunDryRun`, loggea resultado, clasifica `useful` o `partial_candidate`, y desde la revalidacion de `1J` loggea preservacion de anclajes previos con `[F1J-OPT-ANCHOR-PRESERVATION]`; también puede guardar el grafo offline/HTML diagnóstico, aunque algunos parámetros conserven prefijo `f1l_*`. |
| `CollectApplyAffectedKeyFrames` | Reune KFs de vertices y propagacion para crear backup antes del apply de `1K`. |
| `ShouldForceF1LReject` | Aplica el rechazo debug de `1L` para validar rollback de forma controlada. |
| `AssociateGtDebugForDelta` | Desde `1L`, asocia KFs recibidos con la muestra GT temporal mas cercana del mismo dron para metricas debug `[F1L-GT-KF-ASSOC]`; no modifica mapa ni solver. |
| `LogGtWindowErrors` | Desde `1L`, calcula y loggea errores GT por KF de una ventana antes/despues del apply con `[F1L-GT-KF-ERROR-BEFORE]`, `[F1L-GT-KF-ERROR-AFTER]` y `[F1L-GT-WINDOW-STATS]`. |
| `LogGtWindowComparison` | Resume mejora/empeoramiento de la ventana usando metricas GT debug externas. |
| `CollectGtCollateralKeyFrames` | Selecciona KFs de vecindad fiducial previa o KFs fijos del problema para medir dano colateral con GT debug. |
| `LogGtCollateralCheck` | Emite `[F1L-GT-COLLATERAL-CHECK]` para comparar la vecindad de fiduciales previos antes/despues del apply. |
| `ExportF1LDebugAnimation` | Nombre legacy: si `f1l_debug_animation_enabled=true`, crea un HTML animado de diagnóstico del dry-run/grafo, propiedad conceptual de `1J`. |
| `ApplyAndLogOptimizationResult` | Ejecuta el apply de `1K`, reconstruye/publica la nube correspondiente y emite diagnósticos post-apply `F1L` como `[F1L-GLOBALMAP-KF-PROJECTION]`, métricas GT y checks de propagación. |
| `CreateF1IDebugTaskFromCurrentState` | Crea una tarea debug equivalente desde un KF anclado para replay, sin insertarla en `FiducialAnchorManager`. |
| `TryF1IDebugBuildAfterReplay` | Lanza la tarea debug al terminar replay si `f1i_debug_force_task_enabled=true`. |

Cada función o bloque importante debe conservar comentarios con trazabilidad `F1B` o `F1C` según la subfase que introdujo la lógica.

## GT debug por KeyFrame implementado para 1L

El servidor ya recibe `/dron_X/sensor/GT/pose` para simular fiduciales. Para
diagnosticar los fallos actuales de `1L`, esa ruta se extiende con una base
separada de metricas GT por KeyFrame cuando `f1l_gt_kf_debug_enabled=true`:

```text
DebugGroundTruthKeyFrameStore
key=(drone_id,map_epoch,local_kf_id)
world_T_kf_gt
kf_stamp
gt_stamp
association_dt
association_quality
```

Responsabilidad del servidor:

- asociar cada KF recibido con la muestra GT temporal mas cercana del mismo
  dron;
- convertir `world_T_body_gt` a `world_T_camera_gt` usando `body_T_camera`;
- marcar la asociacion como invalida si `association_dt` supera el umbral;
- exponer solo logs de diagnostico `[F1L-GT-*]`;
- no pasar esas poses GT a `OptimizationManager` como entrada de coste, pesos o
  decision.

Logs implementados:

```text
[F1L-GT-KF-ASSOC]
[F1L-GT-KF-ERROR-BEFORE]
[F1L-GT-KF-ERROR-AFTER]
[F1L-GT-WINDOW-STATS]
[F1L-GT-COLLATERAL-CHECK]
```

Uso previsto:

```text
Comparar donde esta cada KF en el mapa global contra donde deberia estar
aproximadamente en simulacion, antes y despues de una optimizacion. Esto ayuda
a saber si la correccion mejora toda la ventana, si solo arregla el fiducial
nuevo, o si esta desplazando la vecindad del fiducial anterior.
```

Restricciones:

- GT por KF no construye mapa;
- GT por KF no decide loops;
- GT por KF no selecciona vertices;
- GT por KF no modifica pesos;
- GT por KF no corrige poses;
- GT por KF no debe existir como dependencia en ejecucion real sin simulacion.

Resultado de la primera ejecucion:

```text
El fiducial objetivo puede corregirse de forma excelente
(`19.776323 -> 0.013640 -> 0.000019 m`), pero `[F1L-GT-WINDOW-STATS]`
mostro que la ventana completa seguia con `mean_after ~4.27 m` y
`max_after ~10.76 m`. Por tanto, estos logs pasan a ser diagnostico clave para
no confundir exito local del fiducial con exito geometrico de la ventana.
```

Si los logs GT son incoherentes, revisar primero `association_dt`,
`map_epoch` y `body_T_camera` antes de repetir simulaciones largas.

## Animacion HTML debug del grafo

Propiedad conceptual: `1J`.

Los nombres `f1l_debug_animation_*` son legacy de parámetros/archivos creados
durante la reapertura de `1L`; no significan que `1L` sea dueña del HTML.

`global_map_server` puede exportar una animacion HTML autocontenida cuando:

```text
f1l_debug_animation_enabled:=true
```

Salida:

```text
src/codex/archivos_auxiliares/html/f1l_debug_animation_task_<task_id>.html
```

Contenido:

- frame `Inicial`: KFs de la ventana en rojo y GT debug en negro;
- frame `Grafo`: vertices del grafo agrandados, mapa en morado/naranja y GT
  de vertices en gris, con aristas del `PoseGraphProblem`;
- frame `Optimizado`: propuestas del dry-run en azul para KFs propagados y
  verde para vertices.

Uso previsto:

```text
Si los vertices ya se separan o quedan lejos del GT, se sospecha de
construccion de grafo/solver. Si los vertices parecen razonables pero los KFs
no vertices azules se dispersan, se sospecha de la propagacion.
```

Restricciones:

- no abre ventanas desde el proceso ROS;
- no se lee de vuelta desde el servidor;
- no usa GT como entrada del solver;
- no decide `ACCEPT`, `PARTIAL` ni `REJECT`;
- no sustituye la inspeccion RViz2 ni los logs `[F1L-GT-WINDOW-STATS]`.

## Logs de validación

```text
[F1B-SERVER-INIT]
[F1B-SERVER-PARAMS]
[F1B-SERVER-SUBSCRIBED]
[F1B-ORBMAP-RX]
[F1B-ORBMAP-RX-KFS]
[F1B-ORBMAP-RX-MPS]
[F1B-SERVER-STATS]
[F1C-RAWDB-INIT]
[F1C-RAWDB-DELTA-RX]
[F1C-RAWDB-INSERT-DELTA]
[F1C-RAWDB-NEW-SUBMAP]
[F1C-RAWDB-STATS]
[F1C-RAWDB-SAVE]
[F1C-REPLAY-LOAD]
[F1C-REPLAY-DELTA]
[F1C-REPLAY-DONE]
[F1D-POSESTORE-INIT]
[F1D-SERVER-DEBUG-ANCHOR]
[F1D-SERVER-DEBUG-OPT]
[F1D-SERVER-POSESTORE-STATS]
[F1D-POSESTORE-ANCHOR-SUMMARY]
[F1D-POSESTORE-OPT-POSE-SET]
[F1D-POSESTORE-CORRECTION-SET]
[F1D-POSESTORE-NEW-KF-CORRECTION-INHERIT]
[F1D-POSESTORE-STATS]
[F1E-FID-INIT]
[F1E-FID-CONFIG]
[F1E-FID-GT-SUBSCRIBED]
[F1E-FID-CANDIDATE-GT]
[F1E-FID-KF-ASSOC]
[F1E-FID-OBS]
[F1E-FID-WORLD-T-LOCAL]
[F1E-FID-FIRST-ANCHOR]
[F1E-FID-KF-HARD]
[F1E-FID-JOURNAL-SAVE]
[F1E-FID-REPLAY-OBS]
[F1E-FID-STATS]
[F1F-BODY-CAMERA-CONFIG]
[F1F-BODY-CAMERA-APPLY]
[F1F-SCORE-INIT]
[F1F-SCORE-UPDATE-ORBSLAM]
[F1F-SCORE-STATS]
[F1F-GLOBALMAP-BUILD]
[F1F-GLOBALMAP-SKIP-UNANCHORED]
[F1F-GLOBALMAP-POINT-STATS]
[F1F-GLOBALMAP-PUBLISH]
[F1G-FULL-SNAPSHOT-CLIENT-READY]
[F1G-FULL-SNAPSHOT-REQUEST]
[F1G-FULL-SNAPSHOT-RX]
[F1G-FULL-SNAPSHOT-ARRIVAL]
[F1G-FULL-SNAPSHOT-TIMEOUT]
[F1G-RAWDB-INSERT-FULL]
[F1G-SNAPSHOT-RECONCILE]
[F1G-RAW-POSE-CHANGED]
[F1G-POSESTORE-REBASE-ANCHOR]
[F1G-POSESTORE-KEEP-OPTIMIZED]
[F1G-SNAPSHOT-AFFECTS-OPTIMIZED-KFS]
[F1G-POSESTORE-RECONCILE-SUMMARY]
[F1G-GLOBALMAP-REBUILD-AFTER-SNAPSHOT]
[F1G-REPLAY-FULL-SNAPSHOT]
[F1H-FID-REVISIT-CONFIG]
[F1H-FID-REVISIT]
[F1H-FID-POSE-ERROR]
[F1H-FID-OK]
[F1H-FID-TASK-CREATED]
[F1H-FID-TASK-SKIP-DUPLICATE]
[F1H-FID-TASK-STATS]
[F1I-GRAPH-BUILDER-CONFIG]
[F1I-TASK-RX]
[F1I-GRAPH-BUILD-START]
[F1I-GRAPH-WINDOW]
[F1I-GRAPH-VERTEX-SELECT]
[F1I-GRAPH-FIXED-KF]
[F1I-GRAPH-VARIABLE-KF]
[F1I-GRAPH-EDGES]
[F1I-GRAPH-PRIORS]
[F1I-GRAPH-WEIGHTS]
[F1I-GRAPH-PROPAGATION-PLAN]
[F1I-GRAPH-PROBLEM-CREATED]
[F1I-GRAPH-BUILD-SUMMARY]
[F1I-FID-CONNECTIVITY-BRANCHES]
[F1I-FID-CONNECTIVITY-EDGE]
[F1I-FID-CONNECTIVITY-SUBDIVISION]
[F1I-GRAPH-ANCHOR-PRESERVATION]
[F1I-GRAPH-PREVIOUS-FIDUCIAL-ANCHOR]
[F1I-GRAPH-VERTEX-COVERAGE]
[F1I-GRAPH-CORNER-VERTEX]
[F1I-GRAPH-LONG-EDGE-SPLIT]
[F1I-GRAPH-PROPAGATION-SEGMENT]
[F1I-DEBUG-TASK-CREATED]
[F1I-GRAPH-DEBUG-BUILD-ONLY]
[F1J-OPT-CONFIG]
[F1J-SERVER-DRYRUN-REQUEST]
[F1J-OPT-TASK-RX]
[F1J-OPT-GRAPH-RX]
[F1J-OPT-PRECHECK]
[F1J-OPT-PRECHECK-FAIL]
[F1J-OPT-POSE-COPY]
[F1J-OPT-CONSTRAINTS]
[F1J-OPT-DRYRUN-START]
[F1J-OPT-SOLVER-SUMMARY]
[F1J-OPT-ANCHOR-PRESERVATION]
[F1J-OPT-PROPAGATION-DRYRUN]
[F1J-OPT-DRYRUN-RESULT]
[F1J-OPT-DRYRUN-DECISION]
[F1J-OPT-NO-STATE-MUTATION]
[F1J-OPT-DEBUG-EXPORT]
[F1J-OPT-DEBUG-EXPORT-FAIL]
[F1J-SERVER-DRYRUN-DONE]
[F1K-OPT-APPLY-CONFIG]
[F1K-OPT-PARTIAL-PENDING]
[F1K-OPT-APPLY-ANCHOR-PRESERVATION-PRECHECK]
[F1K-OPT-APPLY-PRECHECK]
[F1K-OPT-APPLY-PRECHECK-FAIL]
[F1K-OPT-APPLY-KF]
[F1K-POSESTORE-OPTIMIZED-POSE-SET]
[F1K-OPT-APPLY-PROPAGATED-KF]
[F1K-POSESTORE-PROPAGATED-POSE-SET]
[F1K-POSESTORE-CORRECTION-SET]
[F1K-POSESTORE-LAST-CORRECTION-SET]
[F1K-RAWDB-NOT-MODIFIED]
[F1K-OPT-APPLY-SUMMARY]
[F1K-OPT-APPLY-RESULT]
[F1K-GLOBALMAP-REBUILD-AFTER-APPLY]
[F1K-GLOBALMAP-PUBLISH-AFTER-APPLY]
[F1K-POSESTORE-NEW-KF-INHERIT-CORRECTION]
[F1K-POSESTORE-KEEP-SERVER-OPTIMIZED]
[F1K-POSESTORE-RECOMPUTE-CORRECTION-AFTER-RAW-CHANGE]
[F1F-GLOBALMAP-REPUBLISH]
[F1F-GLOBALMAP-REPUBLISH-SKIP]
[F1L-POST-APPLY-CONFIG]
[F1L-POSESTORE-BACKUP-CREATED]
[F1L-PARTIAL-ABSURD-ERROR-POLICY]
[F1L-INTERNAL-EDGE-CLASSIFY]
[F1L-POST-APPLY-START]
[F1L-POST-APPLY-ERROR]
[F1L-POST-APPLY-INTERNAL-ERROR]
[F1L-POST-APPLY-FIXED-CHECK]
[F1L-ANCHOR-PRESERVATION-CHECK]
[F1L-ANCHOR-PRESERVATION-FAIL]
[F1L-POST-APPLY-PROPAGATION-CHECK]
[F1L-POST-APPLY-GLOBALMAP-CHECK]
[F1L-POST-APPLY-ACCEPT]
[F1L-POST-APPLY-PARTIAL]
[F1L-POST-APPLY-REJECT]
[F1L-PARTIAL-CHECKPOINT]
[F1L-PARTIAL-PROPAGATION-SKIP]
[F1L-PARTIAL-PENDING]
[F1L-POSESTORE-COMMIT-CONFIRMED]
[F1L-POSESTORE-ROLLBACK]
[F1L-GLOBALMAP-REBUILD-AFTER-ROLLBACK]
[F1L-GLOBALMAP-PUBLISH-AFTER-ROLLBACK]
[F1L-ROLLBACK-DIAGNOSTIC]
[F1L-RETRY-SUGGESTION]
```

Desde la revalidacion de `1J`, `[F1J-OPT-DRYRUN-DECISION]` incluye tambien:

```text
partial_candidate
partial_reason
before_t
after_t
improvement_ratio
```

Ejemplo:

```text
[F1J-OPT-DRYRUN-DECISION] task_id=69 useful=false partial_candidate=true reason=large_error_reduced_but_above_threshold before_t=12.038334 after_t=0.710850 improvement_ratio=0.940951
```

`[F1B-ORBMAP-RX]` incluye:

- `subscribed_drone_id`;
- `drone_id`;
- `drone_name`;
- `map_epoch`;
- `map_sequence`;
- `frame_id`;
- `map_frame`;
- número de `keyframes`;
- número de `mappoints`.

## Restricciones vigentes

En `1C` este archivo no debe:

- pedir snapshots completos;
- usar ground truth;
- anclar fiduciales;
- publicar `MapCorrection`;
- publicar `CorrectedKeyFrameArray`;
- publicar nube sparse;
- detectar loops;
- fusionar landmarks;
- optimizar poses.

La persistencia permitida en `1C` es solo raw/journal mediante `RawMapDatabase`.

En `1D`, el modo `pose_store_debug_*`:

- solo existe para validar `GlobalPoseStore` con replay;
- no usa ground truth;
- no detecta fiduciales;
- no publica mapa global;
- no debe confundirse con anclaje real. El anclaje real empieza en `1E`.

En `1E`, el servidor:

- puede leer GT de Gazebo solo para simular la detección fiducial;
- no usa GT para loops, fusión, score, mapa global ni pose final;
- persiste cada observación fiducial asociada al `arrival_id` para replay;
- en replay debe poder funcionar con `fiducial_sim_enabled:=false`.

En `1F`, el servidor:

- no calcula score por su cuenta;
- no decide qué submapas son publicables;
- publica solo puntos devueltos por `GlobalMapBuilder`;
- usa `PointCloud2` para exponer `score`, `drone_id` y `map_epoch`;
- configura `body_T_camera` y persiste/reproduce observaciones fiduciales con pose de cuerpo para que el backend derive la pose de cámara;
- no fusiona landmarks ni elimina duplicados raw;
- deja la inspección visual de `/global_sparse_cloud` lista para RViz2.

En `1G`, el servidor:

- pide snapshots completos solo en modo live si `f1g_full_snapshot_enabled=true`;
- en replay no pide servicios y reproduce las entradas `FullSnapshot` del `.record`;
- asigna `arrival_id` monotónico a cada snapshot recibido;
- ignora snapshots vacíos/no inicializados para no contaminar el journal;
- no trata un snapshot como loop, fusión ni optimización;
- reconstruye/publica la nube global tras reconciliar para comprobar que la salida de `1F` no desaparece.

En `1H`, el servidor:

- configura umbrales de revisit fiducial y los pasa al backend;
- no decide optimizacion, solo reporta la decision devuelta por `FiducialAnchorManager`;
- acepta tanto `OK` como `TASK-CREATED` como resultados validos segun residual;
- no debe lanzar solver ni mover KFs;
- publica `/global_sparse_cloud` con QoS `KeepLast(1)`;
- el timer no reconstruye mapa, solo republica la ultima nube cacheada.

Antes de juzgar visualmente RViz2, comprobar que el topic no tenga publicadores duplicados:

```bash
ros2 topic info /global_sparse_cloud -v
```

En la validacion de `1H` se detecto parpadeo porque quedaba vivo un replay antiguo con el mismo nodo `global_orb_map_server`; el topic mostraba `Publisher count: 2`. Tras cerrar ese proceso quedo `Publisher count: 1`.

En `1J`, el servidor:

- configura `OptimizationManager` con parametros `f1j_*`;
- ejecuta dry-run solo si `f1j_dryrun_enabled=true`;
- pasa `PoseGraphProblem`, `RawMapDatabase` y `GlobalPoseStore` como entradas de solo lectura;
- emite `[F1J-OPT-ANCHOR-PRESERVATION]` para comprobar que los anclajes fiduciales previos seleccionados por `1I` no se mueven en la propuesta;
- compara estadisticas de raw y pose store antes/despues para loggear `[F1J-OPT-NO-STATE-MUTATION]`;
- no llama a `SetOptimizedKeyFramePose`;
- no publica `map_correction` ni `corrected_keyframes`;
- mantiene `f1j_export_debug_plot=false` por defecto y solo exporta SVG bajo peticion explicita;
- no lee CSV/SVG para consumir resultados.

En `1K`, el servidor:

- ejecuta apply solo para `OptimizationDryRunResult useful=true`;
- si `1L` no esta habilitada, no aplica `partial_candidate=true/useful=false` y lo loggea como `[F1K-OPT-PARTIAL-PENDING] ... required_next=F1L_DECISION`;
- si `1L` esta habilitada, un parcial puede entrar en `ApplyAndLogOptimizationResult`, pero no se trata como apply normal ni como correccion heredable del submapa;
- antes de crear backup o escribir en `GlobalPoseStore`, emite `[F1K-OPT-APPLY-ANCHOR-PRESERVATION-PRECHECK]` y bloquea el apply si una tarea que requiere preservacion no trae `previous_fiducial_fixed_count>=1`, `fixed_kfs>=1` y `anchor_preservation_ok=true`;
- compara estadisticas raw antes/despues y exige `[F1K-RAWDB-NOT-MODIFIED] ... ok=true`;
- loggea KFs optimizados, KFs propagados y correcciones heredables;
- reconstruye y publica `/global_sparse_cloud` tras apply;
- no publica `MapCorrection` ni `CorrectedKeyFrameArray` todavia.

En `1L`, el servidor:

- crea backup en `GlobalPoseStore` antes de apply;
- permite parciales solo si `f1l_partial_apply_enabled=true`;
- valida el resultado real despues de aplicar, no solo el dry-run;
- valida explicitamente que los anclajes fiduciales previos seleccionados por `1I` no se hayan movido, mediante `[F1L-ANCHOR-PRESERVATION-CHECK]`;
- debe distinguir aristas internas fuertes de aristas deformables/sospechosas antes de convertir `internal_edges_broken` en rollback;
- para errores fiduciales absurdos, si la mejora parcial reduce mucho el error y solo rompe aristas deformables, debe conservar checkpoint parcial y pedir reconstruccion de grafo;
- para parciales, no debe publicar una propagacion amplia que desplace la vecindad del fiducial previo; si se omite esa propagacion, debe loggear `[F1L-PARTIAL-PROPAGATION-SKIP]`;
- un parcial no debe dejar `last_correction_set=true` ni correccion heredable de submapa;
- confirma backup si la decision es `ACCEPT` o `PARTIAL_KEEP_FOR_NEXT_PASS`;
- restaura backup, reconstruye y publica nube restaurada si la decision es `REJECT_ROLLBACK`;
- si se conserva un parcial, debe loggear deuda de reintento/reconstruccion con `[F1L-PARTIAL-PENDING]`;
- emite sugerencia de reintento pero no relanza optimizacion automaticamente;
- sigue sin modificar `RawMapDatabase`;
- no fusiona landmarks ni publica `MapCorrection`/`CorrectedKeyFrameArray` todavia.

## Validación 1G

El 2026-07-08 se validó:

- build de `orbslam3_server` con `BUILD-EXIT-CODE 0`;
- live `prueba_1` con `SIM-DONE prueba=1 success=true`;
- clientes creados para `/dron_1/orbslam/get_full_map` y `/dron_2/orbslam/get_full_map`;
- `8` snapshots recibidos en live y `8` inserciones `[F1G-RAWDB-INSERT-FULL]`;
- `F1G-POSESTORE-REBASE-ANCHOR` y `F1G-POSESTORE-KEEP-OPTIMIZED` aparecen en live;
- replay `prueba_2` con `SIM-DONE prueba=2 success=true`;
- replay con `[F1C-REPLAY-LOAD] ... entries=368 deltas=356 full=12 ...`;
- `12` `[F1G-REPLAY-FULL-SNAPSHOT]`;
- `[F1C-REPLAY-DONE] entries=368 journal=368 deltas=356 full=12 fiducial_observations=74 submaps=4 kfs=225 mps=26165`.

## Validación 1H

El 2026-07-09 se validó:

- build de `orbslam3_server` y `orbslam3_multi` con `BUILD-EXIT-CODE 0`;
- live `prueba_1` con `SIM-DONE prueba=1 success=true`;
- replay `prueba_2` con `SIM-DONE prueba=2 success=true`;
- `[F1H-FID-REVISIT-CONFIG] threshold_t=0.350 threshold_yaw=0.250 threshold_rot=0.350`;
- revisits live con `source=SIM_GT_TEMPORAL_ASSOCIATION`;
- revisits replay con `source=REPLAY_RECORDED_FIDUCIAL`;
- `[F1H-FID-POSE-ERROR]` y `[F1H-FID-OK]` aparecen en ambas pruebas;
- `[F1H-FID-TASK-STATS]` muestra `high_error=0` y `pending=0` porque el residual quedo bajo umbral;
- `[F1F-GLOBALMAP-REPUBLISH]` confirma que el timer republica la nube cacheada;
- el record vigente quedo como `codex/archivos_auxiliares/repeticiones/rawdb_prueba_1.record`, alrededor de `586M`.

Nota visual:

Durante la prueba se detecto que un replay antiguo seguia publicando `/global_sparse_cloud`. El diagnostico se hizo con:

```bash
ros2 topic info /global_sparse_cloud -v
pgrep -af 'global_map_server|global_orb_map_server|ros2 launch orbslam3_server|ros2 launch simulacion_dron'
```

Tras cerrar los PIDs del replay antiguo, el topic quedo con un unico publisher. Si vuelve a aparecer parpadeo entre mapas, esta es la primera comprobacion recomendada.

## Validación 1J

El 2026-07-09 se validó:

- build final de `orbslam3_multi`, `orbslam3_server` y `simulacion_dron` con `BUILD-EXIT-CODE 0`;
- live `prueba_1` con `SCENARIO-RUNNER` y goals `success=true`, `SIM-DONE prueba=1 success=true` y `SIM-EXIT-CODE 0`;
- replay `prueba_2` con `SCENARIO-RUNNER-DONE ... success=true`, `SIM-DONE prueba=2 success=true` y `SIM-EXIT-CODE 0`;
- `prueba_1.reduced.log`: `19` resultados `[F1J-OPT-DRYRUN-RESULT] ... success=true`;
- `prueba_2.reduced.log`: `17` resultados `[F1J-OPT-DRYRUN-RESULT] ... success=true`, incluyendo `task_id=9000000001`;
- todos los resultados validados tienen `[F1J-OPT-DRYRUN-DECISION] ... useful=true reason=error_reduced`;
- `[F1J-OPT-NO-STATE-MUTATION] ... ok=true raw_unchanged=true pose_store_unchanged=true`;
- `[F1J-OPT-DEBUG-EXPORT] ... enabled=false reason=disabled_by_default`, coherente con la politica opt-in;
- no aparecen eventos reales `OPT-APPLY`, `OPTIMIZATION-APPLIED`, `SET_OPTIMIZED_POSE`, `MAP_CORRECTION_PUBLISH` ni `CORRECTED_KEYFRAMES_PUBLISH`.

Nota: `prueba_1.reduced.log` conserva `gazebo ... exit code 255` durante cleanup despues de `SIM-DONE success=true`; se mantiene como no bloqueante.

## Revalidación 1J tras reparar 1I

El 2026-07-10 se revalidó `1J` con grafos que ya incluyen anclaje fiducial previo fijo:

- build final de `orbslam3_multi`, `orbslam3_server` y `simulacion_dron` con `BUILD-EXIT-CODE 0`;
- `prueba_1`: `24` tareas/dry-runs, todos `success=true/useful=true`;
- `prueba_2`: `14` tareas/dry-runs, todos `success=true/partial_candidate=true`;
- todas las tareas muestran `[F1J-OPT-ANCHOR-PRESERVATION] ok=true required=true graph_satisfied=true previous_fiducial_fixed_count>=1 fixed_kfs>=1 hard_fixed_moved=false max_previous_fiducial_delta_t=0.000000000`;
- la `prueba_2` reprodujo errores grandes `before_t=23-26 m`, reducidos a `after_t=2.09-2.31 m`, clasificados como `partial_candidate=true` y dejados sin apply;
- no aparecen `SET_OPTIMIZED_POSE`, `MAP_CORRECTION_PUBLISH`, `CORRECTED_KEYFRAMES_PUBLISH` ni summaries de apply real.

Las pruebas se ejecutaron con `f1k_apply_enabled=false` y `f1l_validation_enabled=false`, por lo que `[F1K-OPT-APPLY-SKIP] reason=disabled` y `[F1K-OPT-PARTIAL-PENDING] ... required_next=F1L_DECISION` son marcadores esperados, no aplicación real.

## Validación 1K histórica

El 2026-07-10 se validó:

- build final de `orbslam3_multi`, `orbslam3_server` y `simulacion_dron` con `BUILD-EXIT-CODE 0`;
- live largo `prueba_3` con ambos drones rodeando el edificio en sentidos opuestos: `14` goals enviados, `14` resultados `success=true`, `SIM-DONE prueba=3 success=true` y `SIM-EXIT-CODE 0`;
- replay/debug `prueba_2` con `SCENARIO-RUNNER-DONE ... success=true`, `SIM-DONE prueba=2 success=true` y `SIM-EXIT-CODE 0`;
- `prueba_3.log`: `5` `[F1K-OPT-APPLY-SUMMARY] ... applied=true`;
- `prueba_2.log`: `3` `[F1K-OPT-APPLY-SUMMARY] ... applied=true`;
- `prueba_3.log`: `309` `[F1K-POSESTORE-NEW-KF-INHERIT-CORRECTION]`;
- ambas pruebas tienen `[F1K-RAWDB-NOT-MODIFIED] ... ok=true` en todos los applies;
- ambas pruebas tienen `[F1K-GLOBALMAP-PUBLISH-AFTER-APPLY]` y `server_corrected_points>0`;
- no aparecen `RAWDB-POSE-OVERWRITE-BY-OPT`, `HARD-FIDUCIAL-MOVED`, `APPLY-USEFUL-FALSE`, prechecks fallidos ni `raw_db_modified=true`.

No se observo RViz2 manualmente. La evidencia automatica confirma publicación de nube corregida, pero la inspección visual humana queda recomendada antes de cerrar decisiones de calidad visual.

Tras la reapertura por preservacion de anclajes, esta validacion queda como evidencia historica.

## Revalidación 1K tras reparar 1I/1J

El 2026-07-11 se revalidó:

- build final de `orbslam3_multi`, `orbslam3_server` y `simulacion_dron` con `BUILD-EXIT-CODE 0`;
- `prueba_1`: `SCENARIO-RUNNER-DONE scenario='prueba_tipica_rodeo_edificio_dos_fiduciales' success=true`, `14` goals `success=true`, `SIM-DONE success=true`, `SIM-EXIT-CODE 0`;
- `prueba_2`: `SCENARIO-RUNNER-DONE scenario='prueba_1k_rodeo_edificio_apply_revalidacion' success=true`, `14` goals `success=true`, `SIM-DONE success=true`, `SIM-EXIT-CODE 0`;
- `prueba_1`: `22` tareas/grafos/dry-runs, `7` applies, `15` parciales pendientes para `1L`;
- `prueba_2`: `44` tareas/grafos/dry-runs, `10` applies, `15` parciales pendientes para `1L`;
- todos los applies tienen `[F1K-OPT-APPLY-ANCHOR-PRESERVATION-PRECHECK] ... ok=true`, `previous_fiducial_fixed_count>=1`, `fixed_kfs>=1`, `hard_fixed_moved=false` y delta nulo del fiducial previo;
- todos los applies tienen `[F1K-RAWDB-NOT-MODIFIED] ... ok=true`, `[F1K-OPT-APPLY-SUMMARY] ... raw_db_modified=false` y `[F1K-GLOBALMAP-PUBLISH-AFTER-APPLY]`;
- `prueba_1`: `27` poses optimizadas, `157` propagadas, `184` correcciones por KF y `402` herencias de correccion en KFs nuevos;
- `prueba_2`: `40` poses optimizadas, `439` propagadas, `479` correcciones por KF y `399` herencias de correccion en KFs nuevos;
- no aparecen `[F1K-OPT-APPLY-ANCHOR-PRESERVATION-PRECHECK] ok=false`, `fixed_kfs=0`, `hard_fixed_moved=true`, `raw_db_modified=true`, `RAWDB-POSE-OVERWRITE-BY-OPT`, `HARD-FIDUCIAL-MOVED`, `APPLY-USEFUL-FALSE`, `FATAL`, `Segmentation fault` ni `Killed`.

No se observó RViz2 manualmente. El cierre `gazebo ... exit code 255` aparece después de `SIM-DONE success=true` y se mantiene como cleanup no bloqueante.

## Validación 1L

El 2026-07-10 se validó:

- build final de `orbslam3_multi`, `orbslam3_server` y `simulacion_dron` con `BUILD-EXIT-CODE 0`;
- `prueba_1`: `SCENARIO-RUNNER-DONE ... success=true`, `14` goals `success=true`, `7` `ACCEPT`, `1` `PARTIAL_KEEP_FOR_NEXT_PASS`;
- `prueba_2`: `SCENARIO-RUNNER-DONE scenario='prueba_1l_rodeo_edificio_rollback_forzado' success=true`, `14` goals `success=true`, `4` `ACCEPT`, `1` `PARTIAL_KEEP_FOR_NEXT_PASS`, `3` `REJECT_ROLLBACK`;
- `prueba_3`: `SCENARIO-RUNNER-DONE scenario='prueba_tipica_rodeo_edificio_dos_fiduciales' success=true`, `14` goals `success=true`, `3` `ACCEPT`, `1` `PARTIAL_KEEP_FOR_NEXT_PASS`;
- los logs reducidos contienen los marcadores obligatorios `[F1L-POST-APPLY-ERROR]`, `[F1L-POST-APPLY-INTERNAL-ERROR]`, `[F1L-POST-APPLY-FIXED-CHECK]`, `[F1L-POST-APPLY-PROPAGATION-CHECK]` y `[F1L-POST-APPLY-GLOBALMAP-CHECK]`;
- `prueba_2` contiene rollback forzado y rechazos naturales por `internal_edges_broken`, todos con `[F1L-POSESTORE-ROLLBACK] ok=true`;
- no aparecen `ROLLBACK_FAILED`, `POST_APPLY_ACCEPT_WITH_ERROR_WORSE`, `HARD-FIDUCIAL-MOVED_ACCEPTED`, `RAWDB-POSE-OVERWRITE-BY-OPT`, `FATAL`, `Segmentation fault` ni `Killed`.

Nota: la prueba replay recomendada se sustituyo por una tercera prueba live porque el dataset `rawdb_prueba_1.record` tuvo que borrarse para liberar espacio tras fallos operativos de build/log por workspace/disco.

## Revalidación 1L tras reparar 1I-1K

El 2026-07-11 se probó `1L`. Los marcadores de backup, rollback y preservacion de anclajes aparecen correctamente, pero la conclusion vigente es `PARCIAL` porque RViz2 muestra que el error grande del dron 2 al fiducial 1 no queda corregido:

- build final de `orbslam3_multi`, `orbslam3_server` y `simulacion_dron` con salida `0`;
- `prueba_1`: `SCENARIO-RUNNER-DONE scenario='prueba_tipica_rodeo_edificio_dos_fiduciales' success=true`, `14` goals `success=true`, `SIM-DONE success=true`, `SIM-EXIT-CODE 0`;
- `prueba_2`: `SCENARIO-RUNNER-DONE scenario='prueba_1l_rodeo_edificio_rollback_forzado' success=true`, `14` goals `success=true`, `SIM-DONE success=true`, `SIM-EXIT-CODE 0`;
- `prueba_3`: `SCENARIO-RUNNER-DONE scenario='prueba_1l_rodeo_edificio_live_parciales' success=true`, `14` goals `success=true`, `SIM-DONE success=true`, `SIM-EXIT-CODE 0`;
- `prueba_1`: `13` validaciones post-apply, `2` `ACCEPT`, `11` `REJECT_ROLLBACK`;
- `prueba_2`: `20` validaciones post-apply, `3` `ACCEPT`, `17` `REJECT_ROLLBACK`, incluyendo rechazo debug forzado;
- `prueba_3`: `19` validaciones post-apply, `3` `ACCEPT`, `16` `REJECT_ROLLBACK`;
- todos los rechazos tienen `[F1L-POSESTORE-ROLLBACK] ok=true` y publicacion posterior con `[F1L-GLOBALMAP-PUBLISH-AFTER-ROLLBACK]`;
- todos los checks tienen `[F1L-ANCHOR-PRESERVATION-CHECK] ok=true required=true previous_fiducial_fixed_count=1 checked_branch_anchors=1 max_previous_fiducial_delta_t=0.000000000`;
- hubo `partial_candidate=true` tratados por `1L`, pero ninguno quedo como `PARTIAL_KEEP_FOR_NEXT_PASS`; se rechazaron con rollback si rompian aristas internas;
- no aparecen `F1L-ANCHOR-PRESERVATION-FAIL`, `ROLLBACK_FAILED`, `POST_APPLY_ACCEPT_WITH_ERROR_WORSE`, `RAWDB-POSE-OVERWRITE-BY-OPT`, `HARD-FIDUCIAL-MOVED_ACCEPTED`, `FATAL`, `Segmentation fault` ni `Killed`;
- ejemplos del fallo pendiente:
  - `prueba_2 task_id=4`: `20.122146 m -> 1.878937 m`, luego `REJECT_ROLLBACK reason=internal_edges_broken`;
  - `prueba_1 task_id=13`: `29.931767 m -> 3.321213 m`, luego `REJECT_ROLLBACK reason=internal_edges_broken`;
  - `prueba_3 task_id=4`: `26.624750 m -> 3.123809 m`, luego `REJECT_ROLLBACK reason=internal_edges_broken`.

Politica pendiente:

- no dejar el fiducial en errores absurdos si una primera correccion baja mucho el residual;
- clasificar la arista interna rota como fuerte o deformable;
- si es deformable, publicar `PARTIAL_KEEP_FOR_NEXT_PASS`, guardar `[F1L-PARTIAL-CHECKPOINT]` y emitir `[F1L-PARTIAL-PENDING] graph_rebuild_required=true`;
- reconstruir un grafo nuevo desde poses parciales antes de la siguiente optimizacion.

## Aislamiento de `dron_2` ejecutado durante 1L

`global_map_server.cpp` coordina ahora checkpoints parciales por `RawSubmapId`, limita reintentos con `f1l_max_partial_retries` y reconstruye inmediatamente el mismo `FiducialOptimizationTask` desde `GlobalPoseStore`. Emite:

```text
[F1L-PARTIAL-RETRY-START]
[F1I-GRAPH-REBUILD-FROM-PARTIAL]
[F1I-GRAPH-WINDOW] rebuilt_from_partial=true
[F1L-PARTIAL-RETRY-LIMIT]
```

La prueba aislada completa el escenario y llega a umbral, pero la primera pasada publica residuales internos de `49-60 m`. La coordinación de retry/checkpoint pertenece al apply de `1K` y a la decisión de `1L`; la barrera geometrica que faltaba corresponde a `1I/1J`.

## Reejecucion 2026-07-12 y reclasificación

Tras reforzar `PoseGraphBuilder` y `PropagationPlan`, cambios propiedad de
`1I`, el servidor:

- configura `pose_graph_max_vertices=24`;
- pasa a `PoseGraphBuilderConfig` los limites
  `pose_graph_max_temporal_edge_kf_gap`,
  `pose_graph_max_temporal_edge_length_m` y
  `pose_graph_corner_yaw_threshold_rad`;
- loggea cobertura y tramos con `[F1I-GRAPH-VERTEX-COVERAGE]` y
  `[F1I-GRAPH-PROPAGATION-SEGMENT]`;
- loggea rechazo temprano con `[F1I-GRAPH-REJECT] reason=bad_window_coverage`
  cuando el builder no puede cubrir la ventana;
- exporta animaciones HTML con `[F1L-DEBUG-ANIMATION-EXPORT]`, nombre legacy de
  una herramienta conceptual de `1J`;
- aplica checkpoints parciales sin propagacion amplia y con
  `last_correction_set=false`.

La prueba aislada de `dron_2` antihorario termino con simulacion OK y genero
HTMLs para `task_id=1..8`, pero todos los casos grandes del fiducial 1 fueron
rechazados por `deformable_edges_broken_without_safe_partial`. Por tanto el
servidor no debe considerar cerrada `1L`: hay que validar/corregir cobertura de
grafo antes del solver, o aumentar vertices de forma adaptativa si la ventana
queda mal cubierta.

La reejecucion posterior implemento ese guard de cobertura. En la prueba
`dron_2` antihoraria, la segunda pasada desde checkpoint parcial fue rechazada
antes del solver con `uncovered_long_segments=3`. El resultado sigue siendo
`PARCIAL` porque el primer candidato redujo el fiducial objetivo pero dejo la
ventana GT completa con errores de varios metros.
# Actualización 1N provisional

El servidor despacha cada `RawInsertResult::new_keyframe_ids` hacia
`LoopDetector` después de la ingesta raw, tanto live como replay. Mientras el
contrato BoW no esté disponible, solo emite `[F1N-LOOP-NEW-KF-DISPATCH]` y
`[F1N-LOOP-KF-QUERY]` con el motivo conservador de ausencia de datos BoW; no
crea candidatos ni modifica estado de loops.

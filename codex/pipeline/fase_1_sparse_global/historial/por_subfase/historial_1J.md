# Historial 1J

> Extraido mecanicamente de `historial_fase_1.md`. Leer este archivo antes de abrir otros historiales de subfase.

## Nota de reclasificación 2026-07-14

Los experimentos de rigidez, pesos, solver SO(3), replay offline,
`RunDryRunGraphOnly`, dumps TSV y HTML diagnóstico que aparecen en historiales
`1L` pertenecen conceptualmente a `1J`. Se conservaron allí por orden temporal
de trabajo y por nombres legacy `f1l_graph_*` / `f1l_debug_animation_*`, pero no
son lógica post-apply.

## 2026-07-09 — Subfase 1J — OptimizationManager dry-run unificado

- objetivo intentado:
  - crear `OptimizationManager` en `orbslam3_multi`;
  - ejecutar dry-run sobre `PoseGraphProblem`;
  - calcular coste/error before-after y decision `useful`;
  - calcular propagacion propuesta sin aplicar;
  - confirmar que `RawMapDatabase` y `GlobalPoseStore` no cambian;
  - mantener `OptimizationDebugExporter` como salida opcional bajo peticion explicita, no como canal normal de datos.
- archivos modificados:
  - `orbslam3_multi/include/orbslam3_multi/optimization_result.hpp`;
  - `orbslam3_multi/include/orbslam3_multi/optimization_manager.hpp`;
  - `orbslam3_multi/src/optimization_manager.cpp`;
  - `orbslam3_multi/include/orbslam3_multi/optimization_debug_exporter.hpp`;
  - `orbslam3_multi/src/optimization_debug_exporter.cpp`;
  - `orbslam3_multi/CMakeLists.txt`;
  - `orbslam3_server/src/global_map_server.cpp`;
  - documentacion de contexto, paquetes, subfase, pipeline e historial.
- cambios realizados:
  - `OptimizationDryRunResult` contiene `success`, `useful`, coste inicial/final, error before/after, propuestas de poses, propagacion propuesta y ruta debug opcional;
  - `OptimizationManager::RunDryRun` trabaja sobre copias temporales de poses;
  - la correccion temporal se calcula como `target_world_T_kf * inverse(current_target_world_T_kf)` y se interpola con `solver_step_fraction`;
  - se detecto y corrigio un bug inicial donde la correccion rotaba la ventana alrededor del origen y empeoraba `after_t`;
  - `global_map_server` emite marcadores `[F1J-*]` y compara estadisticas antes/despues para asegurar no mutacion;
  - `OptimizationDebugExporter` queda desactivado por defecto mediante `f1j_export_debug_plot=false`; el servidor no lee CSV/SVG.
- paquetes compilados:
  ```bash
  ./codex/herramientas/build_selected_packages.sh orbslam3_multi
  ./codex/herramientas/build_selected_packages.sh orbslam3_server
  ./codex/herramientas/build_selected_packages.sh simulacion_dron
  ```
- resultado de build:
  - `orbslam3_multi`: `BUILD-EXIT-CODE 0`;
  - `orbslam3_server`: `BUILD-EXIT-CODE 0`;
  - `simulacion_dron`: `BUILD-EXIT-CODE 0`;
  - no hizo falta ejecutar `reduce_build_log.sh` en el build final.
- pruebas Gazebo/replay ejecutadas:
  - prueba `1`: live larga con:
    ```bash
    ./codex/herramientas/run_simulation.sh --prueba 1 --launch "ros2 launch simulacion_dron multi_dron.launch.py rawdb_record_enabled:=false f1g_debug_mark_optimized_kf:=false f1j_export_debug_plot:=false" --post-scenario-wait-sec 30 --startup-wait-sec 20 --timeout-sec 1200 --max-gazebo-retries 1
    ```
  - prueba `2`: replay/debug con:
    ```bash
    ./codex/herramientas/run_simulation.sh --prueba 2 --launch "ros2 launch orbslam3_server global_orb_map_server.launch.py n_drones:=2 namespace_base:=dron use_sim_time:=false rawdb_record_enabled:=false rawdb_replay_enabled:=true rawdb_replay_path:=src/codex/archivos_auxiliares/repeticiones/rawdb_prueba_1.record rawdb_replay_period_sec:=0.02 fiducial_sim_enabled:=false pose_store_debug_enabled:=false f1g_full_snapshot_enabled:=false f1g_debug_mark_optimized_kf:=false global_map_publish_period_sec:=1.0 f1i_debug_force_task_enabled:=true f1j_dryrun_enabled:=true f1j_export_debug_plot:=false" --post-scenario-wait-sec 10 --startup-wait-sec 5 --timeout-sec 240 --max-gazebo-retries 0
    ```
- resultado de simulacion:
  - `prueba_1`: `SCENARIO-RUNNER` ejecuto goals con `success=true`, `SIM-DONE prueba=1 success=true`, `SIM-EXIT-CODE 0`;
  - `prueba_2`: `SCENARIO-RUNNER-DONE scenario='prueba_1i_pose_graph_replay_debug' success=true`, `SIM-DONE prueba=2 success=true`, `SIM-EXIT-CODE 0`.
- patrones usados para reducir logs:
  - prueba 1:
    ```text
    SCENARIO-RUNNER|SIM-DONE|SIM-EXIT-CODE|GOAL|RESULT|success|F1J-|F1I-GRAPH|F1H-FID-TASK-CREATED|F1E-FID|F1D-POSESTORE|ERROR|FATAL|Segmentation fault|Killed|OPT-APPLY|OPTIMIZATION-APPLIED|SET_OPTIMIZED_POSE|MAP_CORRECTION_PUBLISH|CORRECTED_KEYFRAMES_PUBLISH
    ```
  - prueba 2:
    ```text
    SCENARIO-RUNNER|SIM-DONE|SIM-EXIT-CODE|GOAL|RESULT|success|F1J-|F1I-GRAPH|F1H-FID-TASK-CREATED|F1C-REPLAY|F1D-POSESTORE|ERROR|FATAL|Segmentation fault|Killed|OPT-APPLY|OPTIMIZATION-APPLIED|SET_OPTIMIZED_POSE|MAP_CORRECTION_PUBLISH|CORRECTED_KEYFRAMES_PUBLISH
    ```
- logs reducidos:
  - `codex/archivos_auxiliares/logs/prueba_1.reduced.log`: `2151` lineas;
  - `codex/archivos_auxiliares/logs/prueba_2.reduced.log`: `1633` lineas.
- evidencia positiva encontrada:
  - `prueba_1.reduced.log`: `19` lineas `[F1J-OPT-DRYRUN-RESULT] ... success=true`;
  - `prueba_2.reduced.log`: `17` lineas `[F1J-OPT-DRYRUN-RESULT] ... success=true`, incluyendo `task_id=9000000001`;
  - todos los dry-runs validados tienen `[F1J-OPT-DRYRUN-DECISION] ... useful=true reason=error_reduced`;
  - ejemplo live: `before_t=0.403252 after_t=0.020480 improvement_ratio=0.949214`;
  - ejemplo replay: `before_t=2.693890 after_t=0.163607 improvement_ratio=0.939267`;
  - tarea debug replay: `before_t=2.670325 after_t=0.134376 improvement_ratio=0.949678`;
  - aparecen `[F1J-OPT-TASK-RX]`, `[F1J-OPT-GRAPH-RX]`, `[F1J-OPT-PRECHECK]`, `[F1J-OPT-POSE-COPY]`, `[F1J-OPT-DRYRUN-START]`, `[F1J-OPT-SOLVER-SUMMARY]`, `[F1J-OPT-PROPAGATION-DRYRUN]`, `[F1J-OPT-DRYRUN-RESULT]`, `[F1J-OPT-DRYRUN-DECISION]` y `[F1J-OPT-NO-STATE-MUTATION]`;
  - `[F1J-OPT-NO-STATE-MUTATION] ... ok=true raw_unchanged=true pose_store_unchanged=true`;
  - `[F1J-OPT-DEBUG-EXPORT] ... enabled=false reason=disabled_by_default`, correcto porque la exportacion es opt-in;
  - no aparecieron eventos reales `OPT-APPLY`, `OPTIMIZATION-APPLIED`, `SET_OPTIMIZED_POSE`, `MAP_CORRECTION_PUBLISH` ni `CORRECTED_KEYFRAMES_PUBLISH`.
- evidencia negativa o ausente:
  - `prueba_1.reduced.log` contiene `[F1G-FULL-SNAPSHOT-RX] ... success=false ... reason=startup_resync`; es un snapshot vacio de arranque y no bloquea `1J`;
  - `prueba_1.reduced.log` contiene `gazebo ... exit code 255` despues de `SIM-DONE success=true`, durante cleanup; patron no bloqueante ya observado;
  - no se comprobo RViz2 manualmente; el criterio visual aplicable es que la nube oficial no se mueve por dry-run, y los logs demuestran que no hubo apply.
- documentacion actualizada:
  - `codex/contexto/07_ULTIMA_SESION.md`;
  - `codex/contexto/01_ESTADO_ACTUAL.md`;
  - `codex/contexto/paquetes/orbslam3_multi/orbslam3_multi.md`;
  - `codex/contexto/paquetes/orbslam3_multi/optimization_manager.md`;
  - `codex/contexto/paquetes/orbslam3_multi/optimization_debug_exporter.md`;
  - `codex/contexto/paquetes/orbslam3_multi/pose_graph_builder.md`;
  - `codex/contexto/paquetes/orbslam3_multi/pose_graph_problem.md`;
  - `codex/contexto/paquetes/orbslam3_multi/global_pose_store.md`;
  - `codex/contexto/paquetes/orbslam3_server/global_map_server.md`;
  - `codex/pipeline/fase_1_sparse_global/pipeline_fase_1.md`;
  - `codex/pipeline/fase_1_sparse_global/subfases/subfase_1J.md`;
  - `codex/pipeline/fase_1_sparse_global/historial/historial_fase_1.md`.
- conclusion: `CONSEGUIDA`.
- siguiente paso recomendado:
  - ejecutar `subfase_1K.md`: apply seguro en `GlobalPoseStore` para dry-runs `useful=true`, sin tocar raw, sin mover hard fiducials y preparando rollback.

## 2026-07-09 — Subfase 1J reabierta — Clasificacion `partial_candidate`

- objetivo intentado:
  - modificar `OptimizationDryRunResult` y `OptimizationManager` para distinguir `useful=true`, `partial_candidate=true` y rechazo;
  - conservar como candidato parcial una optimizacion que reduce mucho el error pero queda por encima de `f1j_dryrun_max_final_error_t`;
  - verificar que `1J` sigue sin aplicar poses ni publicar correcciones.
- archivos modificados:
  - `orbslam3_multi/include/orbslam3_multi/optimization_result.hpp`;
  - `orbslam3_multi/include/orbslam3_multi/optimization_manager.hpp`;
  - `orbslam3_multi/src/optimization_manager.cpp`;
  - `orbslam3_server/src/global_map_server.cpp`;
  - documentacion de contexto, paquetes, subfase, pipeline e historial.
- cambios realizados:
  - `OptimizationDryRunResult` añade `partial_candidate` y `partial_reason`;
  - `OptimizationManagerConfig` añade `dryrun_partial_min_improvement_ratio=0.70` y `dryrun_partial_max_allowed_delta_t=35.0`;
  - `OptimizationManager::RunDryRun` marca `partial_candidate=true` con `reason=large_error_reduced_but_above_threshold` cuando baja mucho el error pero el resultado no es aceptable como `useful=true`;
  - `global_map_server` declara parametros `f1j_dryrun_partial_min_improvement_ratio` y `f1j_dryrun_partial_max_allowed_delta_t`;
  - `[F1J-OPT-DRYRUN-DECISION]` loggea `partial_candidate`, `partial_reason`, `before_t`, `after_t` e `improvement_ratio`.
- YAMLs de prueba:
  - se reutilizo `codex/archivos_auxiliares/trayectorias/tray_prueba_1.yaml`;
  - se reutilizo `codex/archivos_auxiliares/trayectorias/tray_prueba_2.yaml`;
  - no hizo falta modificarlos porque ya generan revisits fiduciales live y replay/debug.
- paquetes compilados:
  ```bash
  ./codex/herramientas/build_selected_packages.sh orbslam3_multi
  ./codex/herramientas/build_selected_packages.sh orbslam3_server
  ./codex/herramientas/build_selected_packages.sh simulacion_dron
  ```
- resultado de build:
  - los tres builds finales terminaron con `BUILD-EXIT-CODE 0`;
  - no hizo falta ejecutar `reduce_build_log.sh`.
- pruebas ejecutadas:
  - prueba `1`: live larga con `ros2 launch simulacion_dron multi_dron.launch.py rawdb_record_enabled:=false f1g_debug_mark_optimized_kf:=false f1j_export_debug_plot:=false`;
  - prueba `2`: replay/debug con `ros2 launch orbslam3_server global_orb_map_server.launch.py ... rawdb_replay_enabled:=true ... f1i_debug_force_task_enabled:=true f1j_dryrun_enabled:=true f1j_export_debug_plot:=false`.
- resultado de simulacion:
  - `prueba_1`: `SCENARIO-RUNNER-DONE ... success=true`, `SIM-DONE prueba=1 success=true`, `SIM-EXIT-CODE 0`;
  - `prueba_1`: `14` goals enviados y `14` resultados `success=true`;
  - `prueba_2`: `SCENARIO-RUNNER-DONE scenario='prueba_1i_pose_graph_replay_debug' success=true`, `SIM-DONE prueba=2 success=true`, `SIM-EXIT-CODE 0`.
- patrones usados para reducir logs:
  ```text
  SCENARIO-RUNNER-DONE|SIM-DONE|SIM-EXIT-CODE|F1J-OPT-CONFIG|F1H-FID-TASK-CREATED|F1I-GRAPH-BUILD-SUMMARY|F1J-OPT-*|partial_candidate|large_error_reduced_but_above_threshold|errores graves|apply prohibido
  ```
- logs reducidos:
  - `codex/archivos_auxiliares/logs/prueba_1.reduced.log`: `1654` lineas;
  - `codex/archivos_auxiliares/logs/prueba_2.reduced.log`: `391` lineas.
- evidencia positiva:
  - `prueba_1.log`: `107` dry-runs `[F1J-OPT-DRYRUN-RESULT]`;
  - `prueba_1.log`: `83` decisiones `useful=true`;
  - `prueba_1.log`: `24` decisiones `partial_candidate=true`;
  - `prueba_2.log`: `17` dry-runs, todos `useful=true`, incluyendo tarea debug `9000000001`;
  - `prueba_1.log` y `prueba_2.log`: `[F1J-OPT-NO-STATE-MUTATION] ... ok=true` para todos los dry-runs;
  - ejemplo parcial:
    ```text
    task_id=69 useful=false partial_candidate=true reason=large_error_reduced_but_above_threshold before_t=12.038334 after_t=0.710850 improvement_ratio=0.940951
    ```
  - `[F1J-OPT-CONFIG]` muestra `partial_min_improvement=0.700` y `partial_max_delta_t=35.000`;
  - no aparecen `OPT-APPLY`, `OPTIMIZATION-APPLIED`, `SET_OPTIMIZED_POSE`, `MAP_CORRECTION_PUBLISH` ni `CORRECTED_KEYFRAMES_PUBLISH`.
- evidencia negativa o ausente:
  - no se observo RViz2 manualmente;
  - aparece el cierre conocido `gazebo ... exit code 255` tras `SIM-DONE success=true`, no bloqueante;
  - no se genero SVG porque `f1j_export_debug_plot=false`, correcto para el camino normal.
- conclusion:
  - `CONSEGUIDA`.
- siguiente paso recomendado:
  - ejecutar `subfase_1K.md`: apply seguro en `GlobalPoseStore`, aplicando `useful=true` como camino normal y tratando `partial_candidate=true` como ruta controlada que no es exito normal.

## 2026-07-10 — Subfase 1J — Revalidacion dry-run con preservacion de anclajes

- objetivo intentado:
  - revalidar `OptimizationManager::RunDryRun` tras la reparacion de `1I`;
  - asegurar que los grafos con fiducial previo fijo llegan al dry-run con `fixed_kfs>=1`;
  - impedir que `useful=true` o `partial_candidate=true` se concedan si falla la preservacion de anclajes;
  - confirmar que `1J` no aplica poses ni modifica `RawMapDatabase`/`GlobalPoseStore`.
- archivos modificados:
  - `orbslam3_multi/include/orbslam3_multi/optimization_result.hpp`;
  - `orbslam3_multi/src/optimization_manager.cpp`;
  - `orbslam3_server/src/global_map_server.cpp`;
  - `codex/archivos_auxiliares/trayectorias/tray_prueba_2.yaml`;
  - `codex/contexto/00_LEER_PRIMERO.md`;
  - `codex/contexto/01_ESTADO_ACTUAL.md`;
  - `codex/contexto/06_MAPA_CODIGO.md`;
  - `codex/contexto/07_ULTIMA_SESION.md`;
  - `codex/contexto/paquetes/orbslam3_multi/optimization_manager.md`;
  - `codex/contexto/paquetes/orbslam3_multi/orbslam3_multi.md`;
  - `codex/contexto/paquetes/orbslam3_server/global_map_server.md`;
  - `codex/contexto/paquetes/orbslam3_server/orbslam3_server.md`;
  - `codex/pipeline/fase_1_sparse_global/pipeline_fase_1.md`;
  - `codex/pipeline/fase_1_sparse_global/subfases/subfase_1J.md`;
  - `codex/pipeline/fase_1_sparse_global/subfases/subfase_1K.md`;
  - `codex/pipeline/fase_1_sparse_global/historial/historial_fase_1.md`.
- cambios realizados:
  - `OptimizationDryRunResult` incorpora `anchor_preservation_required`, `anchor_preservation_graph_satisfied`, `anchor_preservation_ok`, `previous_fiducial_fixed_count`, `max_previous_fiducial_delta_t` y `max_previous_fiducial_delta_yaw`;
  - `OptimizationManager::RunDryRun` calcula esas metricas desde `PoseGraphProblem::anchor_preservation` y desde los vertices con `selection_reason=previous_fiducial_anchor`;
  - `OptimizationManager::RunDryRun` exige `anchor_preservation_ok=true` para clasificar un resultado como `useful=true` o `partial_candidate=true`;
  - `global_map_server` emite `[F1J-OPT-ANCHOR-PRESERVATION]`;
  - `tray_prueba_2.yaml` se renombro a `prueba_1j_rodeo_edificio_dryrun_revalidacion`.
- paquetes compilados:
  ```bash
  ./codex/herramientas/build_selected_packages.sh orbslam3_multi
  ./codex/herramientas/build_selected_packages.sh orbslam3_server
  ./codex/herramientas/build_selected_packages.sh simulacion_dron
  ```
- resultado de build:
  - los tres builds terminaron con salida `0`;
  - no hizo falta `reduce_build_log.sh`;
  - `orbslam3_multi` mantiene un warning no bloqueante de funcion no usada en `optimization_manager.cpp`.
- pruebas Gazebo ejecutadas:
  - prueba `1`:
    ```bash
    ./codex/herramientas/run_simulation.sh --prueba 1 --launch "ros2 launch simulacion_dron multi_dron.launch.py rawdb_record_enabled:=false f1j_dryrun_enabled:=true f1k_apply_enabled:=false f1l_validation_enabled:=false" --post-scenario-wait-sec 60 --startup-wait-sec 20 --timeout-sec 1600 --max-gazebo-retries 1
    ```
  - prueba `2`:
    ```bash
    ./codex/herramientas/run_simulation.sh --prueba 2 --launch "ros2 launch simulacion_dron multi_dron.launch.py rawdb_record_enabled:=false f1j_dryrun_enabled:=true f1k_apply_enabled:=false f1l_validation_enabled:=false" --post-scenario-wait-sec 60 --startup-wait-sec 20 --timeout-sec 1600 --max-gazebo-retries 1
    ```
- resultado de simulacion:
  - `prueba_1`: `14` goals `success=true`, `SCENARIO-RUNNER-DONE scenario='prueba_tipica_rodeo_edificio_dos_fiduciales' success=true`, `SIM-DONE prueba=1 success=true`, `SIM-EXIT-CODE 0`;
  - `prueba_2`: `14` goals `success=true`, `SCENARIO-RUNNER-DONE scenario='prueba_1j_rodeo_edificio_dryrun_revalidacion' success=true`, `SIM-DONE prueba=2 success=true`, `SIM-EXIT-CODE 0`.
- patrones usados para reducir logs:
  ```text
  SCENARIO-RUNNER-GOAL-RESULT|SCENARIO-RUNNER-DONE|SIM-DONE|SIM-EXIT-CODE|F1H-FID-TASK-CREATED|F1I-GRAPH|F1I-FID-CONNECTIVITY|F1J-|F1K-OPT-APPLY|F1K-OPT-PARTIAL|OPT-APPLY|OPTIMIZATION-APPLIED|SET_OPTIMIZED_POSE|MAP_CORRECTION_PUBLISH|CORRECTED_KEYFRAMES_PUBLISH|ERROR|FATAL|Segmentation fault|Killed|process has died
  ```
- logs reducidos:
  - `codex/archivos_auxiliares/logs/prueba_1.reduced.log`;
  - `codex/archivos_auxiliares/logs/prueba_2.reduced.log`.
- evidencia positiva:
  - `prueba_1.log`: `24` `[F1H-FID-TASK-CREATED]`, `24` `[F1I-GRAPH-BUILD-SUMMARY]`, `24` `[F1J-OPT-DRYRUN-RESULT] ... success=true`, `24` `[F1J-OPT-ANCHOR-PRESERVATION] ok=true`;
  - `prueba_1.log`: `24` decisiones `useful=true`;
  - `prueba_2.log`: `14` `[F1H-FID-TASK-CREATED]`, `14` `[F1I-GRAPH-BUILD-SUMMARY]`, `14` `[F1J-OPT-DRYRUN-RESULT] ... success=true`, `14` `[F1J-OPT-ANCHOR-PRESERVATION] ok=true`;
  - `prueba_2.log`: `14` decisiones `partial_candidate=true` con `reason=large_error_reduced_but_above_threshold`;
  - caso critico `prueba_2`, `task_id=1`: `before_t=23.188974`, `after_t=2.093630`, `max_delta_t=22.334223`, `fixed_kfs=1`, `hard_fixed_moved=false`, `max_previous_fiducial_delta_t=0.000000000`;
  - `[F1J-OPT-NO-STATE-MUTATION] ... ok=true` aparece en todos los dry-runs;
  - no aparecen `OPTIMIZATION-APPLIED`, `SET_OPTIMIZED_POSE`, `MAP_CORRECTION_PUBLISH` ni `CORRECTED_KEYFRAMES_PUBLISH`.
- evidencia negativa o notas:
  - `F1K-OPT-APPLY-SKIP` aparece para utiles porque `f1k_apply_enabled=false`;
  - `F1K-OPT-PARTIAL-PENDING` aparece para parciales de `prueba_2`, esperado porque quedan como deuda para `1L`;
  - `gazebo ... exit code 255` aparece despues de `SIM-DONE success=true`, conocido como cierre no bloqueante;
  - no se observo RViz2 manualmente; en `1J` la nube oficial no debe moverse.
- conclusion:
  - `CONSEGUIDA`.
- siguiente paso recomendado:
  - ejecutar `subfase_1K.md` para revalidar apply con precheck de preservacion de anclajes antes de escribir en `GlobalPoseStore`.

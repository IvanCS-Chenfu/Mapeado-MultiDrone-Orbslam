# Historial 1K

> Extraido mecanicamente de `historial_fase_1.md`. Leer este archivo antes de abrir otros historiales de subfase.

## 2026-07-10 — Subfase 1K — Apply seguro en `GlobalPoseStore`

- objetivo intentado:
  - aplicar resultados `OptimizationDryRunResult useful=true` en `GlobalPoseStore`;
  - mantener `RawMapDatabase` intacto;
  - marcar KFs optimizados y propagados;
  - guardar correcciones por KF y última corrección heredable por submapa;
  - publicar `/global_sparse_cloud` usando correcciones del servidor;
  - dejar `partial_candidate=true` como deuda obligatoria para `1L`, no como éxito normal.
- archivos modificados:
  - `orbslam3_multi/include/orbslam3_multi/optimization_result.hpp`;
  - `orbslam3_multi/include/orbslam3_multi/optimization_manager.hpp`;
  - `orbslam3_multi/src/optimization_manager.cpp`;
  - `orbslam3_multi/include/orbslam3_multi/global_pose_store.hpp`;
  - `orbslam3_multi/src/global_pose_store.cpp`;
  - `orbslam3_multi/include/orbslam3_multi/global_map_builder.hpp`;
  - `orbslam3_multi/src/global_map_builder.cpp`;
  - `orbslam3_server/src/global_map_server.cpp`;
  - documentación de contexto, paquetes, subfase, pipeline e historial.
- cambios realizados:
  - `OptimizationApplyResult` y registros por KF aplicado;
  - `OptimizationManager::ApplyUsefulResult`, limitado a `useful=true`;
  - `GlobalPoseStore::SetPropagatedKeyFramePose`;
  - consultas de corrección por KF y última corrección por submapa;
  - publicación de MapPoints con corrección del servidor en `GlobalMapBuilder`;
  - parámetro `f1k_apply_enabled`;
  - logs `[F1K-*]` de precheck, apply, raw intacto, propagación, herencia y publicación tras apply;
  - `partial_candidate=true/useful=false` queda como `[F1K-OPT-PARTIAL-PENDING] ... required_next=F1L_DECISION`.
- YAMLs de prueba:
  - se reutilizó `codex/archivos_auxiliares/trayectorias/tray_prueba_3.yaml` para la prueba larga live;
  - se reutilizó `codex/archivos_auxiliares/trayectorias/tray_prueba_2.yaml` para replay/debug;
  - no fue necesario crear YAML nuevo.
- paquetes compilados:
  ```bash
  ./codex/herramientas/build_selected_packages.sh orbslam3_multi
  ./codex/herramientas/build_selected_packages.sh orbslam3_server
  ./codex/herramientas/build_selected_packages.sh simulacion_dron
  ```
- resultado de build:
  - el primer build de `orbslam3_multi` falló por un error de tipos Eigen en `global_map_builder.cpp`;
  - se ejecutó `./codex/herramientas/reduce_build_log.sh`;
  - se corrigió solo el primer error real materializando `world_point` antes de aplicar la corrección opcional;
  - builds finales:
    - `orbslam3_multi`: `BUILD-EXIT-CODE 0`;
    - `orbslam3_server`: `BUILD-EXIT-CODE 0`;
    - `simulacion_dron`: `BUILD-EXIT-CODE 0`.
- pruebas ejecutadas:
  - prueba `3` live larga:
    ```bash
    ./codex/herramientas/run_simulation.sh --prueba 3 --launch "ros2 launch simulacion_dron multi_dron.launch.py rawdb_record_enabled:=false f1g_debug_mark_optimized_kf:=false f1j_export_debug_plot:=false f1k_apply_enabled:=true" --post-scenario-wait-sec 35 --startup-wait-sec 20 --timeout-sec 1400 --max-gazebo-retries 1
    ```
  - prueba `2` replay/debug:
    ```bash
    ./codex/herramientas/run_simulation.sh --prueba 2 --launch "ros2 launch orbslam3_server global_orb_map_server.launch.py n_drones:=2 namespace_base:=dron use_sim_time:=false rawdb_record_enabled:=false rawdb_replay_enabled:=true rawdb_replay_path:=src/codex/archivos_auxiliares/repeticiones/rawdb_prueba_1.record rawdb_replay_period_sec:=0.02 fiducial_sim_enabled:=false pose_store_debug_enabled:=false f1g_full_snapshot_enabled:=false f1g_debug_mark_optimized_kf:=false global_map_publish_period_sec:=1.0 f1i_debug_force_task_enabled:=true f1j_dryrun_enabled:=true f1j_export_debug_plot:=false f1k_apply_enabled:=true" --post-scenario-wait-sec 10 --startup-wait-sec 5 --timeout-sec 300 --max-gazebo-retries 0
    ```
- resultado de simulación:
  - `prueba_3`: `SCENARIO-RUNNER-DONE ... success=true`, `SIM-DONE prueba=3 success=true`, `SIM-EXIT-CODE 0`;
  - `prueba_3`: `14` goals enviados y `14` resultados `success=true`;
  - `prueba_2`: `SCENARIO-RUNNER-DONE scenario='prueba_1i_pose_graph_replay_debug' success=true`, `SIM-DONE prueba=2 success=true`, `SIM-EXIT-CODE 0`.
- patrones usados para reducir logs:
  - prueba `3`:
    ```text
    SCENARIO-RUNNER|GOAL|RESULT|success|F1K-|F1J-OPT|F1I-GRAPH|F1H-FID-TASK-CREATED|F1E-FID|F1F-GLOBALMAP|F1G-POSESTORE|ERROR|FATAL|Segmentation fault|Killed|RAWDB-POSE-OVERWRITE-BY-OPT|HARD-FIDUCIAL-MOVED|APPLY-USEFUL-FALSE
    ```
  - prueba `2`:
    ```text
    SCENARIO-RUNNER|GOAL|RESULT|success|F1K-|F1J-OPT|F1I-GRAPH|F1C-REPLAY|F1D-POSESTORE|F1F-GLOBALMAP|ERROR|FATAL|Segmentation fault|Killed|RAWDB-POSE-OVERWRITE-BY-OPT|HARD-FIDUCIAL-MOVED|APPLY-USEFUL-FALSE
    ```
- logs reducidos:
  - `codex/archivos_auxiliares/logs/prueba_3.reduced.log`: `2151` líneas;
  - `codex/archivos_auxiliares/logs/prueba_2.reduced.log`: `2149` líneas.
- evidencia positiva:
  - `prueba_3.log`: `5` `[F1K-OPT-APPLY-SUMMARY] ... applied=true`;
  - `prueba_2.log`: `3` `[F1K-OPT-APPLY-SUMMARY] ... applied=true`;
  - `prueba_3.log`: `17` `[F1K-POSESTORE-OPTIMIZED-POSE-SET]`;
  - `prueba_3.log`: `40` `[F1K-POSESTORE-PROPAGATED-POSE-SET]`;
  - `prueba_2.log`: `23` `[F1K-POSESTORE-OPTIMIZED-POSE-SET]`;
  - `prueba_2.log`: `54` `[F1K-POSESTORE-PROPAGATED-POSE-SET]`;
  - `prueba_3.log`: `309` `[F1K-POSESTORE-NEW-KF-INHERIT-CORRECTION]`;
  - `prueba_3.log`: `5` `[F1K-RAWDB-NOT-MODIFIED] ... ok=true`;
  - `prueba_2.log`: `3` `[F1K-RAWDB-NOT-MODIFIED] ... ok=true`;
  - `prueba_3.log`: `5` `[F1K-GLOBALMAP-PUBLISH-AFTER-APPLY]`;
  - `prueba_2.log`: `3` `[F1K-GLOBALMAP-PUBLISH-AFTER-APPLY]`;
  - `prueba_3.log` y `prueba_2.log`: `[F1F-GLOBALMAP-POINT-STATS]` con `server_corrected_points>0`;
  - `prueba_3.log`: múltiples `[F1K-OPT-PARTIAL-PENDING] ... required_next=F1L_DECISION`.
- evidencia negativa o ausente:
  - no se observó RViz2 manualmente;
  - no hubo KFs pendientes exactamente durante el apply (`pending_rebased=0`), porque el apply actual es síncrono y breve; sí se validó herencia posterior de corrección;
  - `prueba_3` conserva el cierre conocido de Gazebo `exit code 255` tras `SIM-DONE success=true`, no bloqueante;
  - no aparecieron `RAWDB-POSE-OVERWRITE-BY-OPT`, `HARD-FIDUCIAL-MOVED`, `APPLY-USEFUL-FALSE`, `raw_db_modified=true` ni prechecks fallidos.
- conclusión:
  - `CONSEGUIDA`.
- siguiente paso recomendado:
  - ejecutar `subfase_1L.md`: validar post-apply, backup/rollback y política formal para `partial_candidate=true` con salidas `ACCEPT`, `PARTIAL_KEEP_FOR_NEXT_PASS` o `REJECT_ROLLBACK`.

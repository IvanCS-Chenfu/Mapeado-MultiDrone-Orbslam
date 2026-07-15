## 2026-07-11 — Subfase 1K — Revalidacion apply con preservacion de anclajes

- objetivo intentado:
  - revalidar `1K` tras cerrar de nuevo `1I` y `1J`;
  - impedir que el apply escriba en `GlobalPoseStore` si una tarea que requiere preservar fiducial previo no trae evidencia de anclaje fijo;
  - comprobar que `RawMapDatabase` no cambia, que los KFs optimizados/propagados se registran y que la nube se publica tras apply.
- archivos modificados:
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
  - `codex/pipeline/fase_1_sparse_global/subfases/subfase_1K.md`;
  - `codex/pipeline/fase_1_sparse_global/subfases/subfase_1L.md`;
  - `codex/pipeline/fase_1_sparse_global/historial/historial_fase_1.md`.
- cambios realizados:
  - `OptimizationManager::ApplyCandidateResult` rechaza el apply si `anchor_preservation_required=true` y falta `anchor_preservation_graph_satisfied=true`, `anchor_preservation_ok=true`, `previous_fiducial_fixed_count>0` o `fixed_kfs>0`;
  - `global_map_server` emite `[F1K-OPT-APPLY-ANCHOR-PRESERVATION-PRECHECK]` antes de crear backup o escribir en `GlobalPoseStore`;
  - si el precheck falla, se emite `[F1K-OPT-APPLY-PRECHECK-FAIL]` y no se aplica;
  - `tray_prueba_2.yaml` queda etiquetado como `prueba_1k_rodeo_edificio_apply_revalidacion`.
- paquetes compilados:
  ```bash
  ./codex/herramientas/build_selected_packages.sh orbslam3_multi
  ./codex/herramientas/build_selected_packages.sh orbslam3_server
  ./codex/herramientas/build_selected_packages.sh simulacion_dron
  ```
- resultado de build:
  - los tres builds terminaron con salida `0`;
  - no hizo falta `reduce_build_log.sh`;
  - `orbslam3_multi` conserva un warning no bloqueante de funcion no usada en `optimization_manager.cpp`.
- pruebas Gazebo ejecutadas:
  - prueba `1`:
    ```bash
    ./codex/herramientas/run_simulation.sh --prueba 1 --launch "ros2 launch simulacion_dron multi_dron.launch.py rawdb_record_enabled:=false f1j_dryrun_enabled:=true f1k_apply_enabled:=true f1l_validation_enabled:=false" --post-scenario-wait-sec 60 --startup-wait-sec 20 --timeout-sec 1600 --max-gazebo-retries 1
    ```
  - prueba `2`:
    ```bash
    ./codex/herramientas/run_simulation.sh --prueba 2 --launch "ros2 launch simulacion_dron multi_dron.launch.py rawdb_record_enabled:=false f1j_dryrun_enabled:=true f1k_apply_enabled:=true f1l_validation_enabled:=false" --post-scenario-wait-sec 60 --startup-wait-sec 20 --timeout-sec 1600 --max-gazebo-retries 1
    ```
- resultado de simulacion:
  - `prueba_1`: `14` goals `success=true`, `SCENARIO-RUNNER-DONE scenario='prueba_tipica_rodeo_edificio_dos_fiduciales' success=true`, `SIM-DONE prueba=1 success=true`, `SIM-EXIT-CODE 0`;
  - `prueba_2`: `14` goals `success=true`, `SCENARIO-RUNNER-DONE scenario='prueba_1k_rodeo_edificio_apply_revalidacion' success=true`, `SIM-DONE prueba=2 success=true`, `SIM-EXIT-CODE 0`.
- patrones usados para reducir logs:
  ```text
  SCENARIO-RUNNER-GOAL-RESULT|SCENARIO-RUNNER-DONE|SIM-DONE|SIM-EXIT-CODE|F1H-FID-TASK-CREATED|F1I-GRAPH|F1I-FID-CONNECTIVITY|F1J-OPT|F1K-|F1L-POSESTORE-BACKUP|OPTIMIZATION-APPLIED|SET_OPTIMIZED_POSE|MAP_CORRECTION_PUBLISH|CORRECTED_KEYFRAMES_PUBLISH|RAWDB-POSE-OVERWRITE-BY-OPT|HARD-FIDUCIAL-MOVED|APPLY-USEFUL-FALSE|ERROR|FATAL|Segmentation fault|Killed|process has died
  ```
- logs reducidos:
  - `codex/archivos_auxiliares/logs/prueba_1.reduced.log`;
  - `codex/archivos_auxiliares/logs/prueba_2.reduced.log`.
- evidencia positiva:
  - `prueba_1`: `22` tareas/grafos/dry-runs, `7` `useful=true`, `15` `partial_candidate=true`, `7` applies;
  - `prueba_2`: `44` tareas/grafos/dry-runs, `10` `useful=true`, `15` `partial_candidate=true`, `10` applies;
  - todos los applies tienen `[F1K-OPT-APPLY-ANCHOR-PRESERVATION-PRECHECK] ... ok=true`, `previous_fiducial_fixed_count>=1`, `fixed_kfs>=1`, `hard_fixed_moved=false` y `max_previous_fiducial_delta_t=0.000000000`;
  - todos los applies tienen `[F1K-RAWDB-NOT-MODIFIED] ... ok=true` y `[F1K-OPT-APPLY-SUMMARY] ... raw_db_modified=false`;
  - `prueba_1`: `27` poses optimizadas, `157` propagadas, `184` correcciones por KF y `402` herencias de correccion en KFs nuevos;
  - `prueba_2`: `40` poses optimizadas, `439` propagadas, `479` correcciones por KF y `399` herencias de correccion en KFs nuevos;
  - los `partial_candidate=true/useful=false` quedan como `[F1K-OPT-PARTIAL-PENDING] ... required_next=F1L_DECISION`.
- evidencia negativa o ausente:
  - no aparecen `[F1K-OPT-APPLY-ANCHOR-PRESERVATION-PRECHECK] ok=false`, `fixed_kfs=0`, `hard_fixed_moved=true`, `raw_db_modified=true`, `RAWDB-POSE-OVERWRITE-BY-OPT`, `HARD-FIDUCIAL-MOVED`, `APPLY-USEFUL-FALSE`, `FATAL`, `Segmentation fault` ni `Killed`;
  - no hubo KFs pendientes exactamente durante el apply (`pending_rebased=0` en todos los summaries); si hubo herencia posterior para KFs nuevos;
  - no se observo RViz2 manualmente;
  - el cierre conocido de Gazebo `exit code 255` aparece despues de `SIM-DONE success=true` y se mantiene como cleanup no bloqueante.
- conclusion:
  - `CONSEGUIDA`.
- siguiente paso recomendado:
  - ejecutar `subfase_1L.md`: revalidar post-apply, backup/rollback y candidatos parciales con `[F1L-ANCHOR-PRESERVATION-CHECK]`.


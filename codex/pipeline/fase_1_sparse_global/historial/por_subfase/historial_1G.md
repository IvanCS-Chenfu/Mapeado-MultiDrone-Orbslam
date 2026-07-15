# Historial 1G

> Extraido mecanicamente de `historial_fase_1.md`. Leer este archivo antes de abrir otros historiales de subfase.

## 2026-07-08 — Subfase 1G — Full snapshots y reconciliación segura

- objetivo intentado: pedir full snapshots a los wrappers ORB-SLAM3, guardarlos en el journal raw con `arrival_id`, reconciliar KFs/MPs existentes y proteger el estado global de `GlobalPoseStore`.
- archivos modificados:
  - `orbslam3_multi/include/orbslam3_multi/raw_map_types.hpp`;
  - `orbslam3_multi/src/raw_map_database.cpp`;
  - `orbslam3_multi/include/orbslam3_multi/global_pose_store.hpp`;
  - `orbslam3_multi/src/global_pose_store.cpp`;
  - `orbslam3_server/src/global_map_server.cpp`;
  - `orbslam3_server/launch/global_orb_map_server.launch.py`;
  - `codex/archivos_auxiliares/trayectorias/tray_prueba_1.yaml`;
  - `codex/archivos_auxiliares/trayectorias/tray_prueba_2.yaml`;
  - documentación de contexto, paquetes, pipeline e historial relacionada con `1G`.
- cambios realizados:
  - `RawInsertResult` pasa a informar submapa, tipo de entrada, `arrival_id`, KFs/MPs nuevos, actualizados y bad;
  - `RawMapDatabase::InsertFullSnapshot` aplica política conservadora `insert_update_no_absent_delete`;
  - se detectan cambios de pose local raw en KFs existentes y se registran como `RawPoseChange`;
  - `GlobalPoseStore::ReconcileAfterRawIngestResult` recalcula poses derivadas de anchor y conserva poses optimizadas del servidor;
  - `global_map_server` crea clientes `/dron_X/orbslam/get_full_map`, solicita snapshots al arranque y periódicamente, procesa respuestas y reproduce snapshots desde `.record`;
  - se filtran snapshots vacíos/no inicializados antes de tratarlos como evidencia útil;
  - se borró el antiguo `codex/archivos_auxiliares/repeticiones/prueba_diff_anclaje.record` solicitado por el usuario.
- paquetes compilados:
  ```bash
  ./codex/herramientas/build_selected_packages.sh orbslam3_multi
  ./codex/herramientas/build_selected_packages.sh orbslam3_server
  ```
- resultado de build:
  - `orbslam3_multi`: `BUILD-EXIT-CODE 0`;
  - `orbslam3_server`: `BUILD-EXIT-CODE 0`;
  - no hizo falta ejecutar `reduce_build_log.sh`.
- pruebas Gazebo/replay ejecutadas:
  - prueba `1`: Gazebo live con launch oficial `ros2 launch simulacion_dron multi_dron.launch.py`;
  - prueba `2`: replay sin Gazebo visual usando `ros2 launch orbslam3_server global_orb_map_server.launch.py` con `rawdb_replay_enabled:=true`, `rawdb_replay_period_sec:=0.05`, `fiducial_sim_enabled:=false`, `pose_store_debug_enabled:=false`, `f1g_full_snapshot_enabled:=false` y `f1g_debug_mark_optimized_kf:=true`.
- patrones usados para reducir logs:
  - prueba 1:
    ```text
    SCENARIO-RUNNER|GOAL|RESULT|success|F1G-FULL-SNAPSHOT|F1G-RAWDB|F1G-SNAPSHOT|F1G-RAW-POSE|F1G-POSESTORE|F1G-GLOBALMAP|F1F-GLOBALMAP|ERROR|FATAL|Segmentation fault|Killed
    ```
  - prueba 2:
    ```text
    SCENARIO-RUNNER|GOAL|RESULT|success|F1G-REPLAY|F1G-FULL-SNAPSHOT|F1G-RAWDB|F1G-SNAPSHOT|F1G-DEBUG|F1G-POSESTORE|KEEP-OPTIMIZED|REBASE-ANCHOR|F1C-REPLAY|ERROR|FATAL|Segmentation fault|Killed
    ```
- evidencia positiva encontrada:
  - `prueba_1`: `SIM-DONE prueba=1 success=true`;
  - `prueba_1`: 5 resultados `SCENARIO-RUNNER-GOAL-RESULT ... success=true`;
  - `prueba_1`: 8 `[F1G-FULL-SNAPSHOT-RX]`, 8 `[F1G-RAWDB-INSERT-FULL]` y 8 `[F1G-POSESTORE-RECONCILE-SUMMARY]`;
  - `prueba_1`: snapshot de `drone_id=1 epoch=1` con `new_kfs=3 updated_kfs=24 new_mps=379 updated_mps=1611 raw_pose_changed=23`;
  - `prueba_1`: snapshot de `drone_id=2 epoch=1` con `new_kfs=0 updated_kfs=27 updated_mps=1863 raw_pose_changed=26`;
  - `prueba_1`: aparecen `F1G-POSESTORE-REBASE-ANCHOR` y `F1G-POSESTORE-KEEP-OPTIMIZED`;
  - el record nuevo queda en `codex/archivos_auxiliares/repeticiones/rawdb_prueba_1.record` con tamaño aproximado `472M`;
  - `prueba_2`: `SIM-DONE prueba=2 success=true`;
  - `prueba_2`: `[F1C-REPLAY-LOAD] ... entries=368 deltas=356 full=12 submaps=4 kfs=225 mps=26165 fiducial_observations=74`;
  - `prueba_2`: 12 `[F1G-REPLAY-FULL-SNAPSHOT]`, 12 `[F1G-RAWDB-INSERT-FULL]` y 12 `[F1G-POSESTORE-RECONCILE-SUMMARY]`;
  - `prueba_2`: `[F1C-REPLAY-DONE] entries=368 journal=368 deltas=356 full=12 fiducial_observations=74 submaps=4 kfs=225 mps=26165`.
- evidencia negativa o ausente:
  - la primera grabación intermedia de `1G` contenía un snapshot vacío/no inicializado y se descartó; el código quedó corregido para ignorar ese caso;
  - al primer intento de replay le faltó espacio de disco para escribir `prueba_2.log`; se limpió solo logs temporales antiguos/parciales dentro de `codex/archivos_auxiliares` y se repitió correctamente;
  - en `prueba_1` aparece `gazebo ... exit code 255` durante el cierre posterior a `SIM-DONE success=true`, patrón no bloqueante ya observado;
  - no aparecieron `FATAL`, `Segmentation fault` ni `Killed`;
  - `1G` no implementa optimización real, loops ni fused landmarks.
- documentación actualizada:
  - `codex/contexto/00_LEER_PRIMERO.md`;
  - `codex/contexto/01_ESTADO_ACTUAL.md`;
  - `codex/contexto/04_TOPICS_SERVICES_ACTIONS.md`;
  - `codex/contexto/06_MAPA_CODIGO.md`;
  - `codex/contexto/07_ULTIMA_SESION.md`;
  - `codex/contexto/paquetes/orbslam3_msgs/orbslam3_msgs.md`;
  - `codex/contexto/paquetes/orbslam3_multi/orbslam3_multi.md`;
  - `codex/contexto/paquetes/orbslam3_multi/raw_map_database.md`;
  - `codex/contexto/paquetes/orbslam3_multi/global_pose_store.md`;
  - `codex/contexto/paquetes/orbslam3_server/orbslam3_server.md`;
  - `codex/contexto/paquetes/orbslam3_server/global_map_server.md`;
  - `codex/contexto/paquetes/orbslam3_server/launches.md`;
  - `codex/pipeline/fase_1_sparse_global/pipeline_fase_1.md`;
  - `codex/pipeline/fase_1_sparse_global/subfases/subfase_1G.md`;
  - `codex/pipeline/fase_1_sparse_global/subfases/subfase_1H.md`.
- conclusión: `CONSEGUIDA`.
- siguiente paso recomendado: ejecutar `subfase_1H.md` para medir una segunda visita fiducial y crear una tarea de optimización fiducial sin optimizar todavía.


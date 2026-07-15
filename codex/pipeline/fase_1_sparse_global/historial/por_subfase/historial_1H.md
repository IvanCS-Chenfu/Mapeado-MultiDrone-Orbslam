# Historial 1H

> Extraido mecanicamente de `historial_fase_1.md`. Leer este archivo antes de abrir otros historiales de subfase.

## 2026-07-09 — Subfase 1H — Revisits fiduciales y tarea fiducial

- objetivo intentado: implementar la segunda visita a fiducial para submapas ya anclados, medir residual absoluto contra `GlobalPoseStore` y preparar una `FiducialOptimizationTask` si el residual supera umbrales, sin optimizar todavia.
- archivos modificados:
  - `orbslam3_multi/include/orbslam3_multi/fiducial_optimization_task.hpp`;
  - `orbslam3_multi/include/orbslam3_multi/fiducial_anchor_manager.hpp`;
  - `orbslam3_multi/src/fiducial_anchor_manager.cpp`;
  - `orbslam3_server/src/global_map_server.cpp`;
  - `orbslam3_server/launch/global_orb_map_server.launch.py`;
  - `codex/archivos_auxiliares/trayectorias/tray_prueba_1.yaml`;
  - `codex/archivos_auxiliares/trayectorias/tray_prueba_2.yaml`;
  - `codex/archivos_auxiliares/trayectorias/prueba_tipica_anclaje_diferencial.yaml`;
  - documentación de contexto, paquetes, pipeline, subfase e historial.
- cambios realizados:
  - se añadió `FiducialOptimizationTask` como estructura ligera de residual fiducial alto;
  - `FiducialAnchorManager` distingue primer anchor de revisit;
  - en revisit no se sobrescribe `world_T_local`;
  - se calcula `error_t_m`, `error_rot_rad` y `error_yaw_rad`;
  - se clasifica el residual con umbrales `fiducial_revisit_*`;
  - se emiten `[F1H-FID-REVISIT]`, `[F1H-FID-POSE-ERROR]`, `[F1H-FID-OK]`, `[F1H-FID-TASK-CREATED]`, `[F1H-FID-TASK-SKIP-DUPLICATE]` y `[F1H-FID-TASK-STATS]`;
  - `/global_sparse_cloud` pasa a QoS `KeepLast(1)`;
  - el timer de nube sparse no reconstruye desde raw, solo republica la ultima nube cacheada con `[F1F-GLOBALMAP-REPUBLISH]`.
- paquetes compilados:
  ```bash
  ./codex/herramientas/build_selected_packages.sh orbslam3_multi orbslam3_server
  ```
- resultado de build:
  - build inicial: `2 packages finished`, `BUILD-EXIT-CODE 0`;
  - build tras hotfix de publicación cacheada: `2 packages finished`, `BUILD-EXIT-CODE 0`;
  - no hizo falta ejecutar `reduce_build_log.sh`.
- pruebas Gazebo/replay ejecutadas:
  - prueba `1`: Gazebo live con launch oficial `ros2 launch simulacion_dron multi_dron.launch.py`;
  - prueba `2`: replay sin Gazebo visual usando `ros2 launch orbslam3_server global_orb_map_server.launch.py` con `rawdb_replay_enabled:=true`, `rawdb_replay_period_sec:=0.05`, `fiducial_sim_enabled:=false`, `pose_store_debug_enabled:=false`, `f1g_full_snapshot_enabled:=false`, `f1g_debug_mark_optimized_kf:=false` y `global_map_publish_period_sec:=1.0`.
- patrones usados para reducir logs:
  - prueba 1:
    ```text
    SCENARIO-RUNNER|SIM-DONE|success=true|F1C-|F1D-|F1E-|F1F-|F1G-|F1H-|FID|POSE-ERROR|TASK-CREATED|TASK-STATS|REPUBLISH|ERROR|FATAL|Segmentation fault|Killed
    ```
  - prueba 2:
    ```text
    SCENARIO-RUNNER|SIM-DONE|success=true|REPLAY|F1C-|F1D-|F1E-|F1G-|F1H-|FID|POSE-ERROR|TASK-CREATED|TASK-STATS|REPUBLISH|ERROR|FATAL|Segmentation fault|Killed
    ```
- evidencia positiva encontrada:
  - `prueba_1`: `SIM-DONE prueba=1 success=true`;
  - `prueba_1`: `SCENARIO-RUNNER-GOAL-RESULT ... success=true` en los goals visibles;
  - `prueba_1`: revisits con `source=SIM_GT_TEMPORAL_ASSOCIATION`;
  - `prueba_1`: `[F1H-FID-POSE-ERROR]` y `[F1H-FID-OK]` aparecen para varios KFs;
  - `prueba_1`: `[F1H-FID-TASK-STATS] ... confirmed_ok=32 high_error=0 pending=0` al menos en el tramo reducido;
  - `prueba_2`: `SIM-DONE prueba=2 success=true`;
  - `prueba_2`: revisits con `source=REPLAY_RECORDED_FIDUCIAL`;
  - `prueba_2`: `[F1H-FID-TASK-STATS] ... confirmed_ok=42 high_error=0 pending=0` al menos en el tramo reducido;
  - `prueba_2`: `[F1F-GLOBALMAP-REPUBLISH] reason=timer ... cached_points=24885`;
  - `rawdb_prueba_1.record` vigente queda en `codex/archivos_auxiliares/` con tamaño aproximado `586M`;
  - tras la validación no quedaron procesos `global_map_server` antiguos vivos.
- incidencia RViz2 diagnosticada:
  - durante la live se observó parpadeo entre nube nueva y nube antigua;
  - `ros2 topic info /global_sparse_cloud -v` mostró `Publisher count: 2`;
  - había un replay antiguo de `global_orb_map_server` todavía vivo con PIDs `478404/478434`;
  - se cerró ese replay y el topic quedó con `Publisher count: 1`;
  - el parpadeo principal venía de dos servidores publicando el mismo topic, no del `.record` vigente.
- evidencia negativa o ausente:
  - no se creó `FiducialOptimizationTask` porque el residual del dataset quedó bajo umbral; se considera correcto y suficiente para `1H`;
  - en `prueba_1` aparece `gazebo ... exit code 255` tras `SIM-DONE success=true`, durante cleanup; patrón no bloqueante ya observado;
  - no aparecieron `FATAL`, `Segmentation fault` ni `Killed`;
  - `1H` no implementa optimización real, loops ni fusión.
- documentación actualizada:
  - `codex/contexto/00_LEER_PRIMERO.md`;
  - `codex/contexto/01_ESTADO_ACTUAL.md`;
  - `codex/contexto/06_MAPA_CODIGO.md`;
  - `codex/contexto/07_ULTIMA_SESION.md`;
  - `codex/contexto/paquetes/orbslam3_multi/orbslam3_multi.md`;
  - `codex/contexto/paquetes/orbslam3_multi/fiducial_anchor_manager.md`;
  - `codex/contexto/paquetes/orbslam3_server/global_map_server.md`;
  - `codex/contexto/paquetes/orbslam3_server/launches.md`;
  - `codex/contexto/paquetes/simulacion_dron/simulacion_dron.md`;
  - `codex/pipeline/fase_1_sparse_global/pipeline_fase_1.md`;
  - `codex/pipeline/fase_1_sparse_global/subfases/subfase_1H.md`;
  - `codex/pipeline/fase_1_sparse_global/subfases/subfase_1I.md`;
  - `codex/pipeline/fase_1_sparse_global/historial/historial_fase_1.md`.
- conclusión: `CONSEGUIDA`.
- siguiente paso recomendado: ejecutar `subfase_1I.md` para construir el `PoseGraphBuilder` temporal que consumirá tareas fiduciales o de loop en subfases posteriores.


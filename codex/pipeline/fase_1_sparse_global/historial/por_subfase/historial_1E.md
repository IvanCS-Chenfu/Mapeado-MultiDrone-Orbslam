# Historial 1E

> Extraido mecanicamente de `historial_fase_1.md`. Leer este archivo antes de abrir otros historiales de subfase.

## 2026-07-08 — Subfase 1E — `FiducialAnchorManager` y record fiducial

- objetivo intentado: crear en `orbslam3_multi` una capa backend `FiducialAnchorManager` que reciba observaciones fiduciales ya asociadas a KeyFrames, calcule `world_T_local`, ancle submapas en `GlobalPoseStore`, marque KFs hard fiducial y persista esas observaciones en `.record` para replay sin GT vivo.
- acción previa solicitada por el usuario:
  - se borró `codex/archivos_auxiliares/repeticiones/rawdb_prueba_1.record` anterior para ahorrar espacio;
  - se generó un nuevo `.record` versión 2 desde la simulación viva de `1E`.
- archivos modificados:
  - `orbslam3_multi/include/orbslam3_multi/raw_map_types.hpp`;
  - `orbslam3_multi/include/orbslam3_multi/raw_map_database.hpp`;
  - `orbslam3_multi/src/raw_map_database.cpp`;
  - `orbslam3_multi/include/orbslam3_multi/global_pose_store.hpp`;
  - `orbslam3_multi/src/global_pose_store.cpp`;
  - `orbslam3_multi/include/orbslam3_multi/fiducial_anchor_manager.hpp`;
  - `orbslam3_multi/src/fiducial_anchor_manager.cpp`;
  - `orbslam3_multi/CMakeLists.txt`;
  - `orbslam3_server/src/global_map_server.cpp`;
  - `orbslam3_server/CMakeLists.txt`;
  - `orbslam3_server/launch/global_orb_map_server.launch.py`;
  - `codex/archivos_auxiliares/trayectorias/tray_prueba_1.yaml`;
  - `codex/archivos_auxiliares/trayectorias/tray_prueba_2.yaml`;
  - documentación de contexto, paquetes, pruebas clave y pipeline relacionada con `1E`/`1F`.
- cambios realizados:
  - `RawMapDatabase` pasa a guardar `.record` versión 2, compatible con versión 1, añadiendo `fiducial_observation_journal`;
  - se añade `RecordedFiducialObservation` con `arrival_id`, identidad de KF, `fiducial_id`, pose world del KF, timestamps, distancia y fuente;
  - `GlobalPoseStore` añade marca/consulta de KFs hard fiducial y estadísticas;
  - se crea `FiducialAnchorManager` sin dependencias ROS/Gazebo;
  - `global_map_server` añade subscripción a `/dron_X/sensor/GT/pose` solo en live con `fiducial_sim_enabled=true`;
  - el servidor asocia KFs a fiducial por timestamp y radio, llama al backend, guarda el journal fiducial y reproduce observaciones persistidas en replay;
  - `global_orb_map_server.launch.py` expone parámetros `fiducial_sim_enabled`, `fiducial_gt_max_dt_sec`, `fiducial_gt_buffer_max_samples` y configuración del fiducial 2.
- paquetes compilados:
  ```bash
  ./codex/herramientas/build_selected_packages.sh orbslam3_multi
  ./codex/herramientas/build_selected_packages.sh orbslam3_server
  ```
- resultado de build:
  - `orbslam3_multi`: `BUILD-EXIT-CODE 0`;
  - `orbslam3_server`: `BUILD-EXIT-CODE 0`;
  - no se compiló `simulacion_dron` porque solo se tocaron YAMLs auxiliares, no código/launch del paquete.
- pruebas Gazebo/replay ejecutadas:
  - prueba `1`: Gazebo live con launch oficial `ros2 launch simulacion_dron multi_dron.launch.py`;
  - prueba `2`: replay sin Gazebo visual usando `ros2 launch orbslam3_server global_orb_map_server.launch.py` con `rawdb_replay_enabled:=true`, `fiducial_sim_enabled:=false` y `pose_store_debug_enabled:=false`.
- patrones usados para reducir logs:
  - prueba 1:
    ```text
    SCENARIO-RUNNER|GOAL|RESULT|success|F1B-|F1C-|F1D-|F1E-|FID|POSESTORE|RAWDB|ORBMAP|WRAPPER|ERROR|FATAL|Segmentation fault|Killed
    ```
  - prueba 2:
    ```text
    SCENARIO-RUNNER|GOAL|RESULT|success|F1C-REPLAY|F1E-FID-REPLAY|F1E-FID|F1D-POSESTORE|RAWDB|ERROR|FATAL|Segmentation fault|Killed
    ```
- evidencia positiva encontrada:
  - `prueba_1`: `SIM-DONE prueba=1 success=true`;
  - `prueba_1`: `scenario_runner_node` terminó con `success=true`;
  - `prueba_1`: aparecen `[F1E-FID-KF-ASSOC]`, `[F1E-FID-OBS]`, `[F1E-FID-FIRST-ANCHOR]`, `[F1E-FID-WORLD-T-LOCAL]`, `[F1E-FID-KF-HARD]` y `[F1E-FID-JOURNAL-SAVE]`;
  - `prueba_1`: `[F1C-RAWDB-SAVE] reason=shutdown ... journal=364 deltas=364 full=0 fiducial_observations=60 submaps=3 kfs=175 mps=21095`;
  - `codex/archivos_auxiliares/repeticiones/rawdb_prueba_1.record` nuevo mide `398239640` bytes;
  - `prueba_2`: `SIM-DONE prueba=2 success=true`;
  - `prueba_2`: `[F1C-REPLAY-LOAD] ... fiducial_observations=60`;
  - `prueba_2`: `60` eventos `[F1E-FID-REPLAY-OBS]`;
  - `prueba_2`: `[F1E-FID-STATS] reason=REPLAY_RECORDED_FIDUCIAL observations=60 accepted=60 rejected=0 anchors_created=2 replay_observations=60 hard_fiducial_kfs=60`;
  - `prueba_2`: `[F1C-REPLAY-DONE] entries=364 journal=364 deltas=364 full=0 fiducial_observations=60 submaps=3 kfs=175 mps=21095`.
- evidencia negativa o ausente:
  - no se publica nube global visible todavía; eso queda para `1F`;
  - no hay segunda visita fiducial con optimización; queda para `1H`;
  - no hay loops, fusión real de landmarks ni score;
  - en `prueba_1` aparece `gazebo ... exit code 255` durante el cierre posterior a `SIM-DONE success=true`, patrón no bloqueante ya observado;
  - tras el `.record` nuevo el disco queda con margen muy bajo, aproximadamente `397M`, por lo que conviene liberar espacio antes de repetir simulaciones largas.
- documentación actualizada:
  - `codex/contexto/00_LEER_PRIMERO.md`;
  - `codex/contexto/01_ESTADO_ACTUAL.md`;
  - `codex/contexto/06_MAPA_CODIGO.md`;
  - `codex/contexto/07_ULTIMA_SESION.md`;
  - `codex/contexto/paquetes/orbslam3_multi/orbslam3_multi.md`;
  - `codex/contexto/paquetes/orbslam3_multi/raw_map_database.md`;
  - `codex/contexto/paquetes/orbslam3_multi/global_pose_store.md`;
  - `codex/contexto/paquetes/orbslam3_multi/fiducial_anchor_manager.md`;
  - `codex/contexto/paquetes/orbslam3_server/orbslam3_server.md`;
  - `codex/contexto/paquetes/orbslam3_server/global_map_server.md`;
  - `codex/contexto/paquetes/orbslam3_server/launches.md`;
  - `codex/contexto/pruebas_clave/fase_1C_rawdb_replay.md`;
  - `codex/contexto/pruebas_clave/fase_1E_fiducial_record_replay.md`;
  - `codex/pipeline/fase_1_sparse_global/pipeline_fase_1.md`;
  - `codex/pipeline/fase_1_sparse_global/subfases/subfase_1E.md`;
  - `codex/pipeline/fase_1_sparse_global/subfases/subfase_1F.md`.
- conclusión: `CONSEGUIDA`.
- siguiente paso recomendado: ejecutar `subfase_1F.md` para crear `LandmarkScoreManager` y `GlobalMapBuilder`, publicando la primera nube sparse global en `world` usando solo submapas anclados.


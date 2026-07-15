# Historial 1F

> Extraido mecanicamente de `historial_fase_1.md`. Leer este archivo antes de abrir otros historiales de subfase.

## 2026-07-08 — Subfase 1F — `GlobalMapBuilder`, `LandmarkScoreManager` y `/global_sparse_cloud`

- objetivo intentado: crear una salida sparse global publicable en `world`, con score inicial por punto y usando solo submapas anclados por `GlobalPoseStore`.
- archivos modificados:
  - `orbslam3_multi/include/orbslam3_multi/global_sparse_point.hpp`;
  - `orbslam3_multi/include/orbslam3_multi/landmark_score_manager.hpp`;
  - `orbslam3_multi/src/landmark_score_manager.cpp`;
  - `orbslam3_multi/include/orbslam3_multi/global_map_builder.hpp`;
  - `orbslam3_multi/src/global_map_builder.cpp`;
  - `orbslam3_multi/include/orbslam3_multi/global_pose_store.hpp`;
  - `orbslam3_multi/src/global_pose_store.cpp`;
  - `orbslam3_multi/CMakeLists.txt`;
  - `orbslam3_server/src/global_map_server.cpp`;
  - `orbslam3_server/CMakeLists.txt`;
  - `orbslam3_server/launch/global_orb_map_server.launch.py`;
  - `codex/archivos_auxiliares/trayectorias/tray_prueba_1.yaml`;
  - `codex/archivos_auxiliares/trayectorias/tray_prueba_2.yaml`;
  - documentación de contexto, paquetes, pruebas clave y pipeline relacionada con `1F`/`1G`.
- cambios realizados:
  - se crea `GlobalSparsePoint` como tipo de salida publicable;
  - se crea `LandmarkScoreManager` con eventos semánticos y score inicial raw:
    `0.55 * min(observations_count/8,1) + 0.35 * found_ratio + 0.10 * descriptor_valid`, con `is_bad => 0`;
  - se crea `GlobalMapBuilder`, que consulta `RawMapDatabase`, `GlobalPoseStore` y `LandmarkScoreManager`;
  - `GlobalPoseStore` expone `GetSubmapWorldTransform` para consumidores de solo lectura;
  - `global_map_server` actualiza scores al recibir deltas/replay, construye `PointCloud2` y publica `/global_sparse_cloud`;
  - el `PointCloud2` usa frame `world` y campos `x`, `y`, `z`, `score`, `drone_id` y `map_epoch`;
  - `global_orb_map_server.launch.py` expone `global_sparse_cloud_topic`, `global_map_min_score_to_publish` y `global_map_publish_period_sec`;
  - `tray_prueba_1.yaml` queda como prueba live con recorrido ida/vuelta al fiducial;
  - `tray_prueba_2.yaml` queda como replay lento a `1.0` s por delta.
- paquetes compilados:
  ```bash
  ./codex/herramientas/build_selected_packages.sh orbslam3_multi
  ./codex/herramientas/build_selected_packages.sh orbslam3_server
  ./codex/herramientas/build_selected_packages.sh orbslam3_multi orbslam3_server
  ```
- resultado de build:
  - `orbslam3_multi`: `BUILD-EXIT-CODE 0`;
  - `orbslam3_server`: `BUILD-EXIT-CODE 0`;
  - build final conjunto: `2 packages finished`, `BUILD-EXIT-CODE 0`;
  - no hizo falta ejecutar `reduce_build_log.sh`.
- pruebas Gazebo/replay ejecutadas:
  - prueba `1`: Gazebo live con launch oficial `ros2 launch simulacion_dron multi_dron.launch.py`;
  - prueba `2`: replay lento sin Gazebo visual usando `ros2 launch orbslam3_server global_orb_map_server.launch.py` con `rawdb_replay_period_sec:=1.0`, `fiducial_sim_enabled:=false`, `pose_store_debug_enabled:=false` y `global_map_publish_period_sec:=1.0`.
- patrones usados para reducir logs:
  - prueba 1:
    ```text
    SCENARIO-RUNNER|GOAL|RESULT|success|F1C-RAWDB|F1D-POSESTORE|F1E-FID|F1F-SCORE|F1F-GLOBALMAP|POINTCLOUD|global_sparse_cloud|ERROR|FATAL|Segmentation fault|Killed
    ```
  - prueba 2:
    ```text
    SCENARIO-RUNNER|GOAL|RESULT|success|F1C-REPLAY|F1C-RAWDB|F1D-POSESTORE|F1E-FID|F1E-FID-REPLAY|F1F-SCORE|F1F-GLOBALMAP|POINTCLOUD|global_sparse_cloud|ERROR|FATAL|Segmentation fault|Killed
    ```
- evidencia positiva encontrada:
  - `prueba_1`: `SIM-DONE prueba=1 success=true`;
  - `prueba_1`: `scenario_runner_node` terminó con `success=true`;
  - `prueba_1`: se guardó `.record` con `318` deltas, `72` observaciones fiduciales, `3` submapas, `179` KFs y `22885` MPs;
  - `prueba_1`: `[F1F-SCORE-UPDATE-ORBSLAM]` y `[F1F-SCORE-STATS]` aparecen durante la ingesta live;
  - `prueba_1`: antes de anclar no se publican puntos globales, con `anchored_submaps=0` y `returned_points=0`;
  - `prueba_1`: tras anclar aparecen publicaciones en `/global_sparse_cloud` con `points_published=22394`;
  - `prueba_2`: `SIM-DONE prueba=2 success=true`;
  - `prueba_2`: `[F1C-REPLAY-LOAD] ... entries=318 deltas=318 ... fiducial_observations=72`;
  - `prueba_2`: `[F1C-REPLAY-DONE] entries=318 journal=318 deltas=318 full=0 fiducial_observations=72 submaps=3 kfs=179 mps=22885`;
  - `prueba_2`: `[F1F-GLOBALMAP-PUBLISH] reason=timer topic=/global_sparse_cloud frame_id=world points_from_backend=22394 points_published=22394 min_score_to_publish=0.000 score_field=true drone_id_field=true map_epoch_field=true`;
  - en ambas pruebas el builder informa `anchored_submaps=2` y `skipped_unanchored=1`, por lo que no publica el submapa sin anchor.
- evidencia negativa o ausente:
  - no se realizó inspección manual en RViz2; queda lista para revisión del usuario;
  - no hay fusión real de landmarks en `1F`, por lo que `is_fused=false` y pueden existir duplicados raw entre submapas anclados;
  - no hay full snapshots, loops, optimización ni correcciones publicadas todavía;
  - en `prueba_1` aparece `gazebo ... exit code 255` durante el cierre posterior a `SIM-DONE success=true`, patrón no bloqueante ya observado;
  - no aparecen `FATAL`, `Segmentation fault` ni `Killed`.
- documentación actualizada:
  - `codex/contexto/00_LEER_PRIMERO.md`;
  - `codex/contexto/01_ESTADO_ACTUAL.md`;
  - `codex/contexto/04_TOPICS_SERVICES_ACTIONS.md`;
  - `codex/contexto/06_MAPA_CODIGO.md`;
  - `codex/contexto/07_ULTIMA_SESION.md`;
  - `codex/contexto/paquetes/orbslam3_multi/orbslam3_multi.md`;
  - `codex/contexto/paquetes/orbslam3_multi/global_pose_store.md`;
  - `codex/contexto/paquetes/orbslam3_multi/landmark_score_manager.md`;
  - `codex/contexto/paquetes/orbslam3_multi/global_map_builder.md`;
  - `codex/contexto/paquetes/orbslam3_server/orbslam3_server.md`;
  - `codex/contexto/paquetes/orbslam3_server/global_map_server.md`;
  - `codex/contexto/paquetes/orbslam3_server/launches.md`;
  - `codex/contexto/pruebas_clave/fase_1F_global_sparse_cloud.md`;
  - `codex/pipeline/fase_1_sparse_global/pipeline_fase_1.md`;
  - `codex/pipeline/fase_1_sparse_global/subfases/subfase_1F.md`;
  - `codex/pipeline/fase_1_sparse_global/subfases/subfase_1G.md`.
- conclusión: `CONSEGUIDA`.
- siguiente paso recomendado: ejecutar `subfase_1G.md` para pedir full snapshots de ORB-SLAM3 y reconciliarlos con `RawMapDatabase`/`GlobalPoseStore` sin destruir poses globales ancladas.

## 2026-07-08 — Hotfix Subfase 1F — `body_T_camera` para `/global_sparse_cloud`

- objetivo intentado: corregir la nube sparse global observada en RViz2 porque los anchors fiduciales estaban usando directamente la pose world del cuerpo del dron como pose de cámara/KF.
- diagnóstico:
  - el launch legacy ya contenía parámetros `body_T_camera_*`;
  - la ruta nueva `1E/1F` guardaba y reproducía observaciones fiduciales con pose world del cuerpo;
  - `FiducialAnchorManager` calculaba `world_T_local` sin pasar por la extrínseca cuerpo-cámara.
- archivos modificados:
  - `orbslam3_multi/include/orbslam3_multi/global_pose_store.hpp`;
  - `orbslam3_multi/src/global_pose_store.cpp`;
  - `orbslam3_multi/include/orbslam3_multi/fiducial_anchor_manager.hpp`;
  - `orbslam3_multi/src/fiducial_anchor_manager.cpp`;
  - `orbslam3_server/src/global_map_server.cpp`;
  - `orbslam3_server/launch/global_orb_map_server.launch.py`;
  - documentación de contexto, paquetes, prueba clave y subfase `1F`.
- cambios realizados:
  - `GlobalPoseStore` incorpora `BodyCameraTransformConfig`, `ConfigureBodyCameraTransform`, `GetBodyCameraTransform`, `TransformBodyPoseToCameraPose` y `GetBodyCameraTransformConfig`;
  - `body_T_camera` queda parametrizado con defaults `0.10/0.03/0.03`, `0/-90/90` y `use_camera_optical_frame_convention=true`;
  - `FiducialObservation` pasa a representar explícitamente `world_T_body_fiducial`;
  - `FiducialAnchorManager` calcula `world_T_camera_fiducial = world_T_body_fiducial * body_T_camera` antes de `world_T_local`;
  - el servidor declara `body_T_camera_*`, los pasa al backend y emite `[F1F-BODY-CAMERA-CONFIG]`;
  - el servidor loggea `[F1F-BODY-CAMERA-APPLY]` con `body_t` y `camera_t`;
  - el `.record` de la prueba diferencial se conserva renombrado como `codex/archivos_auxiliares/repeticiones/prueba_diff_anclaje.record`.
- paquetes compilados:
  ```bash
  ./codex/herramientas/build_selected_packages.sh orbslam3_multi
  ./codex/herramientas/build_selected_packages.sh orbslam3_server
  ```
- resultado de build:
  - `orbslam3_multi`: `BUILD-EXIT-CODE 0`;
  - `orbslam3_server`: `BUILD-EXIT-CODE 0`;
  - no hizo falta ejecutar `reduce_build_log.sh`.
- pruebas ejecutadas:
  - replay lento `prueba_2` con `rawdb_replay_period_sec:=1.0`, `fiducial_sim_enabled:=false`, `pose_store_debug_enabled:=false` y `global_map_publish_period_sec:=1.0`;
  - durante la ejecución el path usado fue `src/codex/archivos_auxiliares/repeticiones/rawdb_prueba_1.record`;
  - tras validar, ese archivo se renombró a `src/codex/archivos_auxiliares/repeticiones/prueba_diff_anclaje.record` para conservarlo sin duplicar espacio.
- patrones usados para reducir logs:
  ```text
  SCENARIO-RUNNER|GOAL|RESULT|success|F1C-REPLAY|F1C-RAWDB|F1D-POSESTORE|F1E-FID|F1E-FID-REPLAY|F1F-BODY-CAMERA|F1F-SCORE|F1F-GLOBALMAP|POINTCLOUD|global_sparse_cloud|ERROR|FATAL|Segmentation fault|Killed
  ```
- evidencia positiva encontrada:
  - `SIM-DONE prueba=2 success=true`;
  - `SCENARIO-RUNNER-DONE scenario='prueba_1f_global_sparse_cloud_replay_lento' success=true`;
  - `[F1F-BODY-CAMERA-CONFIG] source=launch use_optical=true body_T_camera_t=(0.100,0.030,0.030) rpy_deg=(0.000,-90.000,90.000)`;
  - `[F1F-BODY-CAMERA-APPLY]` aparece para observaciones fiduciales y muestra `body_t` distinto de `camera_t`;
  - `[F1C-REPLAY-DONE] entries=318 journal=318 deltas=318 full=0 fiducial_observations=72 submaps=3 kfs=179 mps=22885`;
  - `[F1F-GLOBALMAP-PUBLISH] ... topic=/global_sparse_cloud frame_id=world points_from_backend=22394 points_published=22394 ...`;
  - no aparecieron `FATAL`, `Segmentation fault` ni `Killed`.
- evidencia negativa o ausente:
  - la validación automática no decide si visualmente ya desapareció todo el problema; esa confirmación queda en manos del usuario en RViz2;
  - siguen siendo esperables doble pared, deriva entre anchors y ruido porque `1F` aún no optimiza ni fusiona landmarks.
- conclusión: `CONSEGUIDA`.
- siguiente paso recomendado: si la inspección visual del usuario confirma la nube corregida, pasar a `subfase_1G.md`; si no, revisar primero la convención exacta de `body_T_camera` contra el corrector legacy y el frame óptico del wrapper.


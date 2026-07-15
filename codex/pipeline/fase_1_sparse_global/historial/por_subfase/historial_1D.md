# Historial 1D

> Extraido mecanicamente de `historial_fase_1.md`. Leer este archivo antes de abrir otros historiales de subfase.

## 2026-07-08 — Subfase 1D — `GlobalPoseStore` ligero

- objetivo intentado: crear en `orbslam3_multi` una base ligera de poses globales de KeyFrames, separada de `RawMapDatabase`, capaz de guardar anchors de submapas, poses `world_T_kf`, poses optimizadas registradas y una corrección heredable por submapa.
- archivos modificados:
  - `orbslam3_multi/include/orbslam3_multi/raw_map_types.hpp`;
  - `orbslam3_multi/include/orbslam3_multi/raw_map_database.hpp`;
  - `orbslam3_multi/src/raw_map_database.cpp`;
  - `orbslam3_multi/include/orbslam3_multi/global_pose_store.hpp`;
  - `orbslam3_multi/src/global_pose_store.cpp`;
  - `orbslam3_multi/CMakeLists.txt`;
  - `orbslam3_server/src/global_map_server.cpp`;
  - `orbslam3_server/launch/global_orb_map_server.launch.py`;
  - `codex/archivos_auxiliares/trayectorias/tray_prueba_1.yaml`;
  - documentación de contexto, paquetes y pipeline relacionada con `1D`/`1E`.
- cambios realizados:
  - `RawKeyFrameId` y `RawMapPointId` pasan a ser comparables para poder indexar consultas ligeras;
  - `RawMapDatabase` añade getters constantes `GetKeyFrame` y `GetMapPoint`;
  - se crea `GlobalPoseStore` con `AnchorSubmap`, `RegisterNewKeyFrameIfAnchored`, `SetOptimizedKeyFramePose`, consultas de pose y estadísticas;
  - `GlobalPoseStore` calcula poses globales solo cuando recibe un anchor externo y no modifica el raw;
  - se añade una política temporal `1D` de corrección heredable para KFs nuevos posteriores a una pose optimizada;
  - `global_map_server` integra un modo debug `pose_store_debug_*` para validar con replay de `rawdb_prueba_1.record`;
  - `global_orb_map_server.launch.py` expone los parámetros debug, desactivados por defecto;
  - `tray_prueba_1.yaml` queda como escenario de espera para replay sin movimientos.
- paquetes compilados:
  ```bash
  ./codex/herramientas/build_selected_packages.sh orbslam3_multi orbslam3_server
  ./codex/herramientas/reduce_build_log.sh
  ./codex/herramientas/build_selected_packages.sh orbslam3_multi
  ./codex/herramientas/build_selected_packages.sh orbslam3_server
  ```
- resultado de build:
  - el primer build conjunto falló por falta de espacio: `No space left on device`;
  - se redujo el log con `reduce_build_log.sh`;
  - se hizo limpieza mínima de artefactos generados de `orbslam3_multi`, `orbslam3_server` y un log antiguo de build; también se retiraron logs auxiliares antiguos `prueba_2.log` y `prueba_2.reduced.log` dentro de `src`;
  - build final de `orbslam3_multi`: `BUILD-EXIT-CODE 0`;
  - build final de `orbslam3_server`: `BUILD-EXIT-CODE 0`;
  - quedó un warning no bloqueante ya existente en `global_pose_corrector.cpp` por `deg_to_rad` sin uso.
- pruebas Gazebo/replay ejecutadas:
  - prueba principal `1`: replay sin Gazebo visual, usando `run_simulation.sh` con launch `ros2 launch orbslam3_server global_orb_map_server.launch.py`;
  - parámetros relevantes: `rawdb_replay_enabled:=true`, `rawdb_replay_path:=src/codex/archivos_auxiliares/repeticiones/rawdb_prueba_1.record`, `pose_store_debug_enabled:=true`, anchor sintético tras 5 entradas y corrección sintética tras 10 entradas;
  - no se ejecutó la prueba opcional de generación de dataset porque `rawdb_prueba_1.record` ya existía desde `1C`.
- patrones usados para reducir logs:
  ```text
  SCENARIO-RUNNER|GOAL|RESULT|success|F1C-REPLAY|F1D-SERVER|F1D-POSESTORE|F1D-POSESTORE-ANCHOR|F1D-POSESTORE-OPT|F1D-POSESTORE-CORRECTION|F1D-POSESTORE-NEW-KF|F1D-POSESTORE-STATS|ERROR|FATAL|Segmentation fault|Killed
  ```
- evidencia positiva encontrada:
  - `SIM-DONE prueba=1 success=true`;
  - `[SCENARIO-RUNNER-DONE] scenario='prueba_1d_global_pose_store_replay' success=true`;
  - `[F1C-REPLAY-DONE] entries=284 journal=284 deltas=284 full=0 submaps=3 kfs=142 mps=17884`;
  - `[F1D-POSESTORE-INIT] anchors=0 world_poses=0 optimized_kfs=0 corrections=0`;
  - `[F1D-POSESTORE-ANCHOR-SUMMARY] drone_id=1 epoch=0 source=DEBUG_TEST anchored_kfs=1`;
  - `[F1D-POSESTORE-OPT-POSE-SET] drone_id=1 epoch=0 kf=0 source=DEBUG_TEST_OPT`;
  - `[F1D-POSESTORE-CORRECTION-SET] drone_id=1 epoch=0 kf=0 source=DEBUG_TEST_OPT dx=0.150 dy=-0.030 dz=0.000 dyaw=0.050`;
  - cuatro eventos `[F1D-POSESTORE-NEW-KF-CORRECTION-INHERIT]` para KFs nuevos del submapa anclado;
  - estadísticas finales con `anchors=1`, `world_poses=5`, `optimized_kfs=1` y `corrections=1`.
- evidencia negativa o ausente:
  - no hay anchor real todavía; el anchor de `1D` es sintético/debug y el primer anchor real queda para `1E`;
  - no se publica mapa global visible en RViz2 en `1D`;
  - no se tocaron fiduciales reales, loops, fused landmarks ni optimización local real;
  - no aparecen `FATAL`, `Segmentation fault`, `Killed` ni errores graves en los logs reducidos; la palabra `ERROR` aparece solo en la línea de patrones de reducción;
  - el disco quedó con margen bajo, aproximadamente `569M` tras el cierre documental, por lo que conviene vigilar espacio antes de simulaciones largas.
- documentación actualizada:
  - `codex/contexto/00_LEER_PRIMERO.md`;
  - `codex/contexto/01_ESTADO_ACTUAL.md`;
  - `codex/contexto/06_MAPA_CODIGO.md`;
  - `codex/contexto/07_ULTIMA_SESION.md`;
  - `codex/contexto/paquetes/orbslam3_multi/orbslam3_multi.md`;
  - `codex/contexto/paquetes/orbslam3_multi/raw_map_database.md`;
  - `codex/contexto/paquetes/orbslam3_multi/global_pose_store.md`;
  - `codex/contexto/paquetes/orbslam3_server/orbslam3_server.md`;
  - `codex/contexto/paquetes/orbslam3_server/global_map_server.md`;
  - `codex/contexto/paquetes/orbslam3_server/launches.md`;
  - `codex/pipeline/fase_1_sparse_global/pipeline_fase_1.md`;
  - `codex/pipeline/fase_1_sparse_global/subfases/subfase_1D.md`;
  - `codex/pipeline/fase_1_sparse_global/subfases/subfase_1E.md`.
- conclusión: `CONSEGUIDA`.
- siguiente paso recomendado: ejecutar `subfase_1E.md` para crear `FiducialAnchorManager` y convertir una observación fiducial simulada en el primer anclaje real de submapa, poblando `GlobalPoseStore` sin usar ground truth para construir el mapa general.


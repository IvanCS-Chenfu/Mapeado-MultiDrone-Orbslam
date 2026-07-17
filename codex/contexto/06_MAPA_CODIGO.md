# 06 — Mapa de código para Codex

Para ahorrar tokens, leer primero:

```text
codex/contexto/00_BOOTSTRAP_MINIMO.md
codex/contexto/01_ESTADO_ACTUAL_RESUMEN.md
codex/contexto/08_POLITICA_TOKENS_DOCUMENTACION.md
```

Usar este mapa cuando ya se sepa qué paquete/subfase toca revisar.

## Propósito

Este documento ayuda a localizar rápido zonas importantes. Si el código contradice este documento, priorizar siempre el código actual y actualizar la documentación tras validar.

Usar `rg` antes de abrir archivos grandes.

## Fase 1 activa

```text
Planificación activa: subfase_1A.md a subfase_1X.md
Subfase 1A: baseline capturada el 2026-07-03
Subfase 1B: servidor minimo F1B conseguido el 2026-07-06 con comentarios trazables
Subfase 1C: RawMapDatabase y replay conseguidos el 2026-07-06
Subfase 1D: GlobalPoseStore conseguido el 2026-07-08
Subfase 1E: FiducialAnchorManager y replay fiducial conseguidos el 2026-07-08
Subfase 1F: GlobalMapBuilder, LandmarkScoreManager, /global_sparse_cloud y hotfix body_T_camera conseguidos el 2026-07-08
Subfase 1G: full snapshots y reconciliacion segura conseguidos el 2026-07-08
Subfase 1H: revisit fiducial, medicion de residual y tareas pendientes conseguidos el 2026-07-09
Subfase 1I: PoseGraphBuilder temporal conseguido de nuevo el 2026-07-10 con preservacion de anclajes fiduciales previos
Subfase 1J: OptimizationManager dry-run y `partial_candidate` conseguidos de nuevo el 2026-07-10 con `[F1J-OPT-ANCHOR-PRESERVATION]`
Subfase 1K: apply seguro en `GlobalPoseStore` conseguido de nuevo el 2026-07-11 con precheck de preservacion de anclajes
Subfase 1L: PARCIAL; diagnostico post-apply, pendiente de revalidacion futura
Subfase 1M: actual/a probar en simulación; `CovisibilityDatabase` implementada
Subfase actual: 1N (por implementar; infraestructura sin BoW real)
```

No usar `12R-D4` ni otras subfases residuales como planificación activa. Se conservan como legacy.

## Servidor global legacy y servidor nuevo

Paquete:

```text
orbslam3_server
```

Archivo activo principal:

```text
orbslam3_server/src/global_map_server.cpp
```

Servidor heredado congelado:

```text
orbslam3_server/src/global_map_server_antiguo.cpp
```

Búsquedas útiles tras 1B:

```bash
rg -n "F1B|OnOrbMapDelta|CreateOrbMapSubscriptions|RawMapDatabase" orbslam3_server orbslam3_multi
rg -n "global_orb_map_server|global_map_server_antiguo" orbslam3_server/launch
```

Desde 1C, el servidor mínimo ya está conectado a `RawMapDatabase` sin reintroducir lógica legacy.

## Backend objetivo

Paquete:

```text
orbslam3_multi
```

Archivos activos actuales:

```text
orbslam3_multi/include/orbslam3_multi/raw_map_types.hpp
orbslam3_multi/include/orbslam3_multi/raw_map_database.hpp
orbslam3_multi/src/raw_map_database.cpp
orbslam3_multi/include/orbslam3_multi/global_pose_store.hpp
orbslam3_multi/src/global_pose_store.cpp
orbslam3_multi/include/orbslam3_multi/fiducial_anchor_manager.hpp
orbslam3_multi/src/fiducial_anchor_manager.cpp
orbslam3_multi/include/orbslam3_multi/landmark_score_manager.hpp
orbslam3_multi/src/landmark_score_manager.cpp
orbslam3_multi/include/orbslam3_multi/global_map_builder.hpp
orbslam3_multi/src/global_map_builder.cpp
orbslam3_multi/include/orbslam3_multi/global_sparse_point.hpp
orbslam3_multi/include/orbslam3_multi/pose_graph_problem.hpp
orbslam3_multi/include/orbslam3_multi/pose_graph_builder.hpp
orbslam3_multi/src/pose_graph_builder.cpp
orbslam3_multi/include/orbslam3_multi/optimization_result.hpp
orbslam3_multi/include/orbslam3_multi/optimization_manager.hpp
orbslam3_multi/src/optimization_manager.cpp
orbslam3_multi/include/orbslam3_multi/optimization_debug_exporter.hpp
orbslam3_multi/src/optimization_debug_exporter.cpp
orbslam3_multi/legacy/
```

Búsquedas útiles:

```bash
rg -n "RawMapDatabase|RawSubmapId|RawJournalEntry|InsertDelta|InsertFullSnapshot|RawPoseChange|SaveToPath|LoadFromPath|GetKeyFrame" orbslam3_multi
rg -n "GlobalPoseStore|ReconcileAfterRawIngestResult|BodyCameraTransformConfig|TransformBodyPoseToCameraPose|AnchorSubmap|RegisterNewKeyFrameIfAnchored|SetOptimizedKeyFramePose|FiducialAnchorManager|LoopDetector|SubcloudLoopVerifier|LandmarkScoreManager|GlobalMapBuilder|OptimizationManager|PoseGraphBuilder" orbslam3_multi
```

Los módulos antiguos `MultiDroneSystem`, `GlobalAtlas`, `GlobalKeyFrameDatabase`, `GlobalORBMatcher`, `GlobalGeometryVerifier`, `GlobalPoseGraphOptimizer`, `ImportedKeyFrame`, `ImportedMapPoint` y `GlobalPoseGraphTypes` solo existen como referencia en `orbslam3_multi/legacy/*_antiguo.*`.

Si una clase nueva aún no existe, la subfase correspondiente define cómo crearla.

## Mapa de subfases activas

| Subfase | Archivo | Código esperado |
|---|---|---|
| 1A | `subfase_1A.md` | Realizada: no corrigió código; capturó baseline del servidor actual. |
| 1B | `subfase_1B.md` | Realizada: servidor nuevo mínimo, legacy congelado y comentarios con etiquetas `F1B`. |
| 1C | `subfase_1C.md` | Realizada: `RawMapDatabase`, `arrival_id`, deltas, snapshots y replay. |
| 1D | `subfase_1D.md` | Realizada: `GlobalPoseStore`, replay debug, anchor/corrección sintéticos y herencia de corrección. |
| 1E | `subfase_1E.md` | Realizada: `FiducialAnchorManager`, anclaje inicial real por fiducial simulado y `.record` v2 con observaciones fiduciales. |
| 1F | `subfase_1F.md` | Realizada: `GlobalMapBuilder`, `LandmarkScoreManager` inicial, publicación `/global_sparse_cloud` y aplicación de `body_T_camera` para anclajes fiduciales. |
| 1G | `subfase_1G.md` | Realizada: reconciliación de full snapshots con poses globales, replay con `full=12` y protección de pose optimizada debug. |
| 1H | `subfase_1H.md` | Realizada: revisits fiduciales, medición de error absoluto, `FiducialOptimizationTask` preparada y hotfix de publicación cacheada para `/global_sparse_cloud`. |
| 1I | `subfase_1I.md` | Realizada de nuevo: `PoseGraphBuilder` temporal con preservacion de anclajes fiduciales previos. |
| 1J | `subfase_1J.md` | Realizada de nuevo: dry-run, dump/replay offline y HTML diagnóstico del grafo. |
| 1K | `subfase_1K.md` | Realizada/parcial: apply en `GlobalPoseStore`, propagación, publicación coherente y herencia de KFs futuros. |
| 1L | `subfase_1L.md` | Parcial: diagnóstico post-apply con logs/RViz2/GT debug; no es dueña del solver ni del apply. |
| 1M | `subfase_1M.md` | Actual/a probar en simulación: `CovisibilityDatabase` implementada para ORB-SLAM3 y loops geométricos. |
| 1N | `subfase_1N.md` | Sin hacer: `LoopDetector` BoW, consultando covisibilidad confirmada para saltar pares ya conocidos. |
| 1O | `subfase_1O.md` | `SubcloudLoopVerifier`. |
| 1P | `subfase_1P.md` | `LoopDecisionManager`, fusión y scoring multi-dron. |
| 1Q | `subfase_1Q.md` | Optimización por loop reutilizando 1I-1L. |
| 1R | `subfase_1R.md` | Reprocesado de KFs llegados durante optimización. |
| 1S | `subfase_1S.md` | Scoring centralizado y fused tracks. |
| 1T | `subfase_1T.md` | Contratos, IDs, frames e invariantes. |
| 1U | `subfase_1U.md` | Topics debug y observabilidad RViz2. |
| 1V | `subfase_1V.md` | Pruebas end-to-end y regresión. |
| 1W | `subfase_1W.md` | Rendimiento y límites. |
| 1X | `subfase_1X.md` | Cierre documental y handoff. |

## Wrapper ORB-SLAM3

Paquete:

```text
orbslam3_ros2
```

Búsqueda:

```bash
rg -n "BuildOrbMap|FillMapPointMsg|FillKeyFrameMsg|map_epoch|orb_map_delta|get_full_map|HashMapPoint|HashKeyFrame" orbslam3_ros2
```

Regla: no tocar salvo necesidad fuerte.

## Mensajes ROS

Paquete:

```text
orbslam3_msgs
```

Búsqueda:

```bash
find orbslam3_msgs -maxdepth 3 -type f
```

Regla: no rediseñar ni añadir score global durante Fase 1 salvo subfase explícita.

## Simulación y escenarios

Paquete:

```text
simulacion_dron
```

Launch oficial:

```bash
ros2 launch simulacion_dron multi_dron.launch.py
```

YAMLs automáticos:

```text
codex/archivos_auxiliares/trayectorias/tray_prueba_1.yaml
codex/archivos_auxiliares/trayectorias/tray_prueba_2.yaml
...
```

Pruebas tipicas reutilizables:

```text
codex/contexto/pruebas_clave/pruebas_tipicas.md
```

Regla: `tray_prueba_X.yaml` es el alias mecanico que consume `run_simulation.sh`; una trayectoria estable debe conservarse tambien con nombre semantico de prueba tipica y documentarse en historial.

Logs:

```text
SCENARIO-RUNNER
SIM-DONE
SIM-EXIT-CODE
```

## Herramientas Codex

```text
codex/herramientas/build_selected_packages.sh
codex/herramientas/reduce_build_log.sh
codex/herramientas/run_simulation.sh
codex/herramientas/reduce_simulation_log.sh
```

Build habitual de Fase 1:

```bash
./codex/herramientas/build_selected_packages.sh orbslam3_msgs orbslam3_multi orbslam3_server
```

Si se toca simulación:

```bash
./codex/herramientas/build_selected_packages.sh dron_individual simulacion_dron
```

Los logs completos terminan en `codex/archivos_auxiliares/logs/*.log`; los reducidos terminan en `*.reduced.log`.

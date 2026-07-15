# Subfase 1Q — Optimización por loop reutilizando el pipeline existente 1I–1L

## Estado

```text
sin hacer
```

---

## Objetivo tecnico

Validar y adaptar el pipeline de optimización ya creado para fiduciales para que funcione también con loops.

Esta subfase **no crea un sistema nuevo de optimización**. Debe reutilizar las clases y el flujo ya definidos en subfases anteriores:

```text
LoopVerificationResult con error alto
        ↓
LoopDecisionManager crea LoopOptimizationTask
        ↓
PoseGraphBuilder construye PoseGraphProblem
        ↓
PoseGraphBuilder añade aristas desde CovisibilityDatabase
        ↓
OptimizationManager ejecuta dry-run
        ↓
OptimizationManager aplica si useful=true
        ↓
GlobalPoseStore guarda poses/correcciones
        ↓
validación post-apply recalcula error real del loop
        ↓
si ACCEPT:
    FusionManager fusiona landmarks
si REJECT:
    rollback y no se fusiona
```

La idea es comprobar que lo creado para optimización por fiducial también sirve para optimización por loop, cambiando solo:

- tipo de tarea;
- constraints objetivo;
- selección de KeyFrames;
- pesos;
- métrica de error before/after;
- validación post-apply;
- fusión posterior si la optimización queda aceptada.
- restricciones extra de covisibilidad confirmada.

Regla principal:

```text
1Q no crea un nuevo optimizador.
1Q reutiliza PoseGraphBuilder, OptimizationManager, GlobalPoseStore,
PostApplyValidation y FusionManager para LoopOptimizationTask.
```

---

## Contexto obligatorio a leer

Antes de actuar, Codex debe leer:

- `AGENTS.md`
- `codex/contexto/01_ESTADO_ACTUAL.md`
- `codex/pipeline/PIPELINE_MAESTRO.md`
- `codex/pipeline/fase_1_sparse_global/pipeline_fase_1.md`
- `codex/pipeline/fase_1_sparse_global/subfases/subfase_1A.md`
- `codex/pipeline/fase_1_sparse_global/subfases/subfase_1B.md`
- `codex/pipeline/fase_1_sparse_global/subfases/subfase_1C.md`
- `codex/pipeline/fase_1_sparse_global/subfases/subfase_1D.md`
- `codex/pipeline/fase_1_sparse_global/subfases/subfase_1E.md`
- `codex/pipeline/fase_1_sparse_global/subfases/subfase_1F.md`
- `codex/pipeline/fase_1_sparse_global/subfases/subfase_1G.md`
- `codex/pipeline/fase_1_sparse_global/subfases/subfase_1H.md`
- `codex/pipeline/fase_1_sparse_global/subfases/subfase_1I.md`
- `codex/pipeline/fase_1_sparse_global/subfases/subfase_1J.md`
- `codex/pipeline/fase_1_sparse_global/subfases/subfase_1K.md`
- `codex/pipeline/fase_1_sparse_global/subfases/subfase_1L.md`
- `codex/pipeline/fase_1_sparse_global/subfases/subfase_1M.md`
- `codex/pipeline/fase_1_sparse_global/subfases/subfase_1N.md`
- `codex/pipeline/fase_1_sparse_global/subfases/subfase_1O.md`
- `codex/pipeline/fase_1_sparse_global/subfases/subfase_1P.md`
- historial reciente de Fase 1;
- documentación de paquetes afectados:
  - `codex/contexto/paquetes/orbslam3_multi/`
  - `codex/contexto/paquetes/orbslam3_server/`
  - `codex/contexto/paquetes/simulacion_dron/`
- ADRs relacionados con:
  - `RawMapDatabase`;
  - `GlobalPoseStore`;
  - `LoopDetector`;
  - `SubcloudLoopVerifier`;
  - `FusionManager`;
  - `FusedLandmarkManager`;
  - `LandmarkScoreManager`;
  - pose graph;
  - apply/rollback;
  - no usar GT para loops.

---

## Diagnostico de partida

Hasta esta subfase, el pipeline nuevo debe tener:

1. `LoopDetector` generando candidatos BoW.
2. `SubcloudLoopVerifier` construyendo subnubes, haciendo matching/RANSAC y calculando error.
3. `LoopDecisionManager` interpretando `LoopVerificationResult`.
4. `CovisibilityDatabase` con covisibilidad ORB-SLAM3 y loops confirmados.
5. `FusionManager` / `FusedLandmarkManager` fusionando landmarks cuando el error es bajo.
6. `PoseGraphBuilder` creando grafos temporales para tareas de optimización.
7. `OptimizationManager` ejecutando dry-run.
8. `OptimizationManager` o equivalente aplicando resultados útiles.
9. `GlobalPoseStore` guardando poses optimizadas, propagadas y correcciones heredables.
10. Validación post-apply/rollback definida en 1L.
11. `GlobalMapBuilder` publicando nube fused.

El problema actual es que la ruta 1I–1L se ha diseñado y validado principalmente con tareas de fiducial, donde el constraint objetivo es absoluto:

```text
un KeyFrame debe acercarse a una pose world de fiducial.
```

En loops, el constraint objetivo es relativo/geométrico:

```text
una zona query debe alinearse con una zona candidate según la transformación RANSAC.
```

La subfase 1Q debe comprobar que el mismo pipeline funciona para este segundo caso.

Además, el grafo de 1Q debe consultar `CovisibilityDatabase` para añadir
restricciones entre KeyFrames que ya representan la misma zona. Estas aristas
pueden venir de:

```text
ORBSLAM3_NATIVE
SERVER_LOOP_GEOMETRIC
```

La finalidad es evitar que la optimización separe KFs covisibles y crear dobles
paredes, sin aumentar obligatoriamente el número de vértices.

---

## Archivos permitidos a modificar

### `orbslam3_multi`

Se pueden modificar archivos ya creados en fases anteriores para soportar `LoopOptimizationTask`:

- `orbslam3_multi/include/orbslam3_multi/loop_optimization_task.hpp`
- `orbslam3_multi/include/orbslam3_multi/optimization_task.hpp`
- `orbslam3_multi/include/orbslam3_multi/loop_decision_manager.hpp`
- `orbslam3_multi/include/orbslam3_multi/covisibility_database.hpp`
- `orbslam3_multi/src/covisibility_database.cpp`
- `orbslam3_multi/src/loop_decision_manager.cpp`
- `orbslam3_multi/include/orbslam3_multi/pose_graph_builder.hpp`
- `orbslam3_multi/src/pose_graph_builder.cpp`
- `orbslam3_multi/include/orbslam3_multi/pose_graph_problem.hpp`
- `orbslam3_multi/include/orbslam3_multi/optimization_manager.hpp`
- `orbslam3_multi/src/optimization_manager.cpp`
- `orbslam3_multi/include/orbslam3_multi/optimization_result.hpp`
- `orbslam3_multi/include/orbslam3_multi/global_pose_store.hpp`
- `orbslam3_multi/src/global_pose_store.cpp`
- `orbslam3_multi/include/orbslam3_multi/subcloud_loop_verifier.hpp`
- `orbslam3_multi/src/subcloud_loop_verifier.cpp`
- `orbslam3_multi/include/orbslam3_multi/fusion_manager.hpp`
- `orbslam3_multi/src/fusion_manager.cpp`
- `orbslam3_multi/include/orbslam3_multi/fused_landmark_manager.hpp`
- `orbslam3_multi/src/fused_landmark_manager.cpp`
- `orbslam3_multi/include/orbslam3_multi/global_map_builder.hpp`
- `orbslam3_multi/src/global_map_builder.cpp`
- `orbslam3_multi/include/orbslam3_multi/raw_map_database.hpp`
- `orbslam3_multi/src/raw_map_database.cpp`
- `orbslam3_multi/CMakeLists.txt`
- `orbslam3_multi/package.xml` solo si fuese necesario

Solo crear estructuras mínimas si faltan. No crear un optimizador paralelo.

### `orbslam3_server`

Cambios mínimos para ejecutar la ruta de loop optimization:

- `orbslam3_server/src/global_map_server.cpp`
- clases auxiliares del servidor nuevo si existen
- launch nuevo del servidor si necesita parámetros debug

### `codex/archivos_auxiliares`

- `codex/archivos_auxiliares/trayectorias/tray_prueba_1.yaml`
- `codex/archivos_auxiliares/trayectorias/tray_prueba_2.yaml`
- `codex/archivos_auxiliares/trayectorias/tray_prueba_3.yaml`
- datasets/replay creados en 1C/1N/1O/1P
- salidas debug:
  - `codex/archivos_auxiliares/f1p_loop_graph_<task_id>.svg`
  - `codex/archivos_auxiliares/f1p_loop_dryrun_<task_id>.svg`
  - `codex/archivos_auxiliares/f1p_loop_post_apply_<task_id>.svg`
  - o formatos equivalentes documentados

### Documentación

- `codex/contexto/paquetes/orbslam3_multi/pose_graph_builder.md`
- `codex/contexto/paquetes/orbslam3_multi/optimization_manager.md`
- `codex/contexto/paquetes/orbslam3_multi/global_pose_store.md`
- `codex/contexto/paquetes/orbslam3_multi/loop_decision_manager.md`
- `codex/contexto/paquetes/orbslam3_multi/fusion_manager.md`
- `codex/contexto/paquetes/orbslam3_multi/subcloud_loop_verifier.md`
- `codex/contexto/paquetes/orbslam3_server/global_map_server.md`
- historial de Fase 1 / subfase 1Q

---

# Subfase 1P — Decisión de loop, fusión de landmarks y scoring multi-dron

## Estado

```text
sin hacer
```

---

## Objetivo tecnico

Crear la primera versión de la fase de decisión posterior a la verificación geométrica de loops.

La subfase 1O produce un `LoopVerificationResult` con una decisión preliminar:

```text
REJECT
HOLD_INSUFFICIENT_EVIDENCE
FUSION_CANDIDATE
LOOP_OPTIMIZATION_CANDIDATE
```

La subfase 1P debe interpretar ese resultado y orquestar la acción correspondiente:

```text
LoopVerificationResult
        ↓
LoopDecisionManager
        ↓
REJECT / HOLD / FUSION / LOOP_OPTIMIZATION_TASK
```

Objetivos principales:

1. Crear una clase de decisión de loops que no mezcle geometría, fusión y optimización en una sola clase.
2. Crear `FusionManager` / `FusedLandmarkManager` para fusionar landmarks equivalentes cuando el error sea bajo.
3. Integrar la fusión con `LandmarkScoreManager`.
4. Publicar preferentemente landmarks fusionados en `GlobalMapBuilder`.
5. Crear `LoopOptimizationTask` cuando las subnubes sean geométricamente consistentes pero estén separadas por error alto.
6. Asegurar que, si hay optimización por loop, la fusión solo se hace después de que el pipeline 1I–1L acepte la optimización post-apply.
7. Insertar en `CovisibilityDatabase` solo loops confirmados geométricamente,
   con pose relativa medida y soporte de inliers.

Regla principal:

```text
Si el error es bajo:
    insertar covisibilidad confirmada.
    fusionar landmarks equivalentes.

Si el error es alto pero la geometría confirma que las nubes son la misma zona:
    insertar covisibilidad confirmada como restricción geométrica.
    crear LoopOptimizationTask.
    No fusionar todavía.
    Fusionar solo después de optimización aceptada por 1L.

Si no hay evidencia suficiente:
    no insertar covisibilidad.
    no fusionar ni optimizar.
```

Esta fase profundiza en scoring de landmarks: los puntos fusionados deben subir score y los puntos esperados pero no asociados pueden bajar score de forma prudente.

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
- historial reciente de Fase 1;
- documentación de paquetes afectados:
  - `codex/contexto/paquetes/orbslam3_multi/`
  - `codex/contexto/paquetes/orbslam3_server/`
  - `codex/contexto/paquetes/orbslam3_msgs/`
  - `codex/contexto/paquetes/simulacion_dron/`
- ADRs relacionados con:
  - `RawMapDatabase`;
  - `GlobalPoseStore`;
  - `LandmarkScoreManager`;
  - `GlobalMapBuilder`;
  - fusión de landmarks;
  - loops;
  - pose graph;
  - scoring;
  - no usar GT para loops.

---

## Diagnostico de partida

Hasta esta subfase, el sistema nuevo debe tener:

1. `RawMapDatabase`, que conserva KeyFrames/MapPoints raw de ORB-SLAM3.
2. `GlobalPoseStore`, que conserva poses globales, anchors, optimizaciones y correcciones.
3. `LandmarkScoreManager`, creado en 1F, con score inicial basado en señales raw de ORB-SLAM3.
4. `GlobalMapBuilder`, que publica puntos con score.
5. `CovisibilityDatabase`, que guarda relaciones confirmadas.
6. `LoopDetector`, que genera candidatos BoW y evita pares ya confirmados.
7. `SubcloudLoopVerifier`, que construye subnubes, hace matching/RANSAC y calcula error geométrico.
8. Pipeline de optimización fiducial ya existente:
   - 1I `PoseGraphBuilder`;
   - 1J dry-run;
   - 1K apply;
   - 1L post-apply/rollback.

El problema actual es que 1O solo devuelve una decisión preliminar. Todavía no existe la lógica que:

- fusiona landmarks cuando dos subnubes ya están en el mismo sitio;
- crea una tarea de optimización por loop cuando las subnubes son la misma zona pero están separadas;
- registra covisibilidad confirmada para que optimizaciones futuras no separen
  KFs que representan la misma zona;
- actualiza scores por fusión multi-dron;
- baja score de puntos que deberían haber sido asociados pero no lo fueron;
- evita publicar duplicados después de fusionar;
- retrasa la fusión hasta después de una optimización aceptada cuando el error de loop es alto.

Punto de diseño clave:

```text
RawMapDatabase no borra ni fusiona MapPoints originales.
FusedLandmarkManager crea tracks globales.
GlobalMapBuilder publica el track fusionado en vez de publicar todos sus miembros raw.
```

---

## Archivos permitidos a modificar

### `orbslam3_multi`

Se pueden crear o modificar archivos relacionados con decisión de loop, fusión y scoring:

- `orbslam3_multi/include/orbslam3_multi/loop_decision_manager.hpp`
- `orbslam3_multi/src/loop_decision_manager.cpp`
- `orbslam3_multi/include/orbslam3_multi/fusion_manager.hpp`
- `orbslam3_multi/src/fusion_manager.cpp`
- `orbslam3_multi/include/orbslam3_multi/fused_landmark_manager.hpp`
- `orbslam3_multi/src/fused_landmark_manager.cpp`
- `orbslam3_multi/include/orbslam3_multi/fused_landmark_track.hpp`
- `orbslam3_multi/include/orbslam3_multi/loop_optimization_task.hpp`
- `orbslam3_multi/include/orbslam3_multi/covisibility_database.hpp`
- `orbslam3_multi/src/covisibility_database.cpp`
- `orbslam3_multi/include/orbslam3_multi/loop_verification_result.hpp`
- `orbslam3_multi/include/orbslam3_multi/subcloud_loop_verifier.hpp`
- `orbslam3_multi/src/subcloud_loop_verifier.cpp`
- `orbslam3_multi/include/orbslam3_multi/landmark_score_manager.hpp`
- `orbslam3_multi/src/landmark_score_manager.cpp`
- `orbslam3_multi/include/orbslam3_multi/global_map_builder.hpp`
- `orbslam3_multi/src/global_map_builder.cpp`
- `orbslam3_multi/include/orbslam3_multi/raw_map_database.hpp`
- `orbslam3_multi/src/raw_map_database.cpp`
- `orbslam3_multi/include/orbslam3_multi/global_pose_store.hpp`
- `orbslam3_multi/src/global_pose_store.cpp`
- `orbslam3_multi/CMakeLists.txt`
- `orbslam3_multi/package.xml` solo si fuese necesario

Si existen clases legacy de fusión o matching, pueden consultarse o adaptarse de forma controlada, sin reactivar rutas antiguas peligrosas.

### `orbslam3_server`

Cambios mínimos para invocar el orquestador de loops:

- `orbslam3_server/src/global_map_server.cpp`
- clases auxiliares del servidor nuevo si existen
- launch nuevo del servidor si necesita parámetros debug

### `codex/archivos_auxiliares`

- `codex/archivos_auxiliares/trayectorias/tray_prueba_1.yaml`
- `codex/archivos_auxiliares/trayectorias/tray_prueba_2.yaml`
- datasets/replay creados en 1C/1N/1O
- salidas debug opcionales:
  - `codex/archivos_auxiliares/f1o_fusion_tracks_<id>.csv`
  - `codex/archivos_auxiliares/f1o_score_events_<id>.csv`
  - `codex/archivos_auxiliares/f1o_loop_decision_<id>.txt`

### Documentación

- `codex/contexto/paquetes/orbslam3_multi/loop_decision_manager.md`
- `codex/contexto/paquetes/orbslam3_multi/fusion_manager.md`
- `codex/contexto/paquetes/orbslam3_multi/fused_landmark_manager.md`
- `codex/contexto/paquetes/orbslam3_multi/landmark_score_manager.md`
- `codex/contexto/paquetes/orbslam3_multi/global_map_builder.md`
- `codex/contexto/paquetes/orbslam3_server/global_map_server.md`
- historial de Fase 1 / subfase 1P

---

# Subfase 1S — Consolidación de scoring de landmarks y fused tracks

## Estado

```text
sin hacer
```

---

## Objetivo tecnico

Consolidar de forma explícita toda la política de scores de MapPoints, landmarks fusionados y publicación de nube global.

Hasta ahora el scoring aparece repartido entre varias subfases:

```text
1F:
    crea LandmarkScoreManager y score inicial/raw desde ORB-SLAM3.

1P:
    sube score por fusión confirmada.
    baja score de puntos esperados pero no asociados.
    publica fused tracks con score.

1Q:
    fusiona después de optimización por loop aceptada.

1R:
    reinyecta KFs rebasados al pipeline de loops.
```

La subfase 1S no debe crear una lógica nueva de SLAM, loops u optimización. Su objetivo es cerrar y validar una política única de score:

```text
Ninguna clase externa modifica scores directamente.
Todas las clases emiten eventos semánticos.
LandmarkScoreManager decide cómo esos eventos cambian el score.
```

Esta subfase debe dejar claro:

- qué eventos de score existen;
- qué datos raw de ORB-SLAM3 afectan al score;
- cómo se calcula score inicial;
- cómo se actualiza score de MapPoints raw;
- cómo se calcula score de `FusedLandmarkTrack`;
- cuándo sube score por fusión;
- cuándo baja score por falta de asociación;
- cómo se revierten cambios de score si una optimización/fusión se rechaza;
- cómo `GlobalMapBuilder` usa scores para publicar;
- qué logs permiten validar el scoring.

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
- `codex/pipeline/fase_1_sparse_global/subfases/subfase_1N.md`
- `codex/pipeline/fase_1_sparse_global/subfases/subfase_1O.md`
- `codex/pipeline/fase_1_sparse_global/subfases/subfase_1P.md`
- `codex/pipeline/fase_1_sparse_global/subfases/subfase_1Q.md`
- `codex/pipeline/fase_1_sparse_global/subfases/subfase_1R.md`
- historial reciente de Fase 1;
- documentación de paquetes afectados:
  - `codex/contexto/paquetes/orbslam3_multi/`
  - `codex/contexto/paquetes/orbslam3_server/`
  - `codex/contexto/paquetes/simulacion_dron/`
- ADRs relacionados con:
  - `RawMapDatabase`;
  - `LandmarkScoreManager`;
  - `FusedLandmarkManager`;
  - `GlobalMapBuilder`;
  - loops;
  - fusión;
  - rollback;
  - publicación de nube global.

---

## Diagnostico de partida

El sistema ya debe tener o estar en proceso de tener:

1. `RawMapDatabase` con KeyFrames, MapPoints, observaciones y datos raw procedentes de ORB-SLAM3.
2. `LandmarkScoreManager`, creado inicialmente en 1F.
3. `FusionManager` / `FusedLandmarkManager`, creados en 1P.
4. `GlobalMapBuilder`, que publica nube sparse con score.
5. `LoopDecisionManager`, que decide si fusionar o crear tarea de optimización.
6. Optimización y rollback definidos en 1I–1L.
7. Optimización por loop validada en 1Q.
8. Reprocesado post-optimización de KFs en 1R.

El problema es que el score es transversal. Si no se consolida, puede acabar modificado desde varias clases de forma incoherente:

```text
RawMapDatabase
FusionManager
SubcloudLoopVerifier
LoopDecisionManager
GlobalMapBuilder
FusedLandmarkManager
OptimizationManager
```

Eso haría difícil depurar por qué un punto se publica, desaparece, sube o baja de confianza.

La subfase 1S debe centralizar y documentar la política.

---

## Archivos permitidos a modificar

### `orbslam3_multi`

Se pueden modificar o crear archivos relacionados con scoring:

- `orbslam3_multi/include/orbslam3_multi/landmark_score_manager.hpp`
- `orbslam3_multi/src/landmark_score_manager.cpp`
- `orbslam3_multi/include/orbslam3_multi/landmark_score_event.hpp`
- `orbslam3_multi/include/orbslam3_multi/landmark_score_policy.hpp`
- `orbslam3_multi/include/orbslam3_multi/fused_landmark_manager.hpp`
- `orbslam3_multi/src/fused_landmark_manager.cpp`
- `orbslam3_multi/include/orbslam3_multi/fused_landmark_track.hpp`
- `orbslam3_multi/include/orbslam3_multi/fusion_manager.hpp`
- `orbslam3_multi/src/fusion_manager.cpp`
- `orbslam3_multi/include/orbslam3_multi/global_map_builder.hpp`
- `orbslam3_multi/src/global_map_builder.cpp`
- `orbslam3_multi/include/orbslam3_multi/raw_map_database.hpp`
- `orbslam3_multi/src/raw_map_database.cpp`
- `orbslam3_multi/include/orbslam3_multi/loop_decision_manager.hpp`
- `orbslam3_multi/src/loop_decision_manager.cpp`
- `orbslam3_multi/include/orbslam3_multi/optimization_result.hpp`
- `orbslam3_multi/include/orbslam3_multi/optimization_manager.hpp`
- `orbslam3_multi/src/optimization_manager.cpp`
- `orbslam3_multi/CMakeLists.txt`
- `orbslam3_multi/package.xml` solo si fuese necesario

### `orbslam3_server`

Cambios mínimos para exponer logs, parámetros y publicación de nube:

- `orbslam3_server/src/global_map_server.cpp`
- clases auxiliares del servidor nuevo si existen
- launch/config si necesita parámetros de score

### `codex/archivos_auxiliares`

- `codex/archivos_auxiliares/trayectorias/tray_prueba_1.yaml`
- `codex/archivos_auxiliares/trayectorias/tray_prueba_2.yaml`
- `codex/archivos_auxiliares/trayectorias/tray_prueba_3.yaml`
- datasets/replay creados en fases anteriores
- salidas debug opcionales:
  - `codex/archivos_auxiliares/f1r_score_events_<id>.csv`
  - `codex/archivos_auxiliares/f1r_score_snapshot_<id>.csv`
  - `codex/archivos_auxiliares/f1r_fused_scores_<id>.csv`

### Documentación

- `codex/contexto/paquetes/orbslam3_multi/landmark_score_manager.md`
- `codex/contexto/paquetes/orbslam3_multi/fused_landmark_manager.md`
- `codex/contexto/paquetes/orbslam3_multi/fusion_manager.md`
- `codex/contexto/paquetes/orbslam3_multi/global_map_builder.md`
- `codex/contexto/paquetes/orbslam3_multi/raw_map_database.md`
- `codex/contexto/paquetes/orbslam3_server/global_map_server.md`
- historial de Fase 1 / subfase 1S

---


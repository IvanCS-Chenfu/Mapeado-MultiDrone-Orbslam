# Subfase 1R — Reprocesado post-optimización de KeyFrames llegados durante optimización

## Estado

```text
sin hacer
```

---

## Objetivo tecnico

Cerrar el ciclo de los KeyFrames que llegan mientras una optimización está en curso.

En subfases anteriores se definió que, si llega un KeyFrame nuevo durante una optimización, el sistema no debe descartarlo ni colocarlo definitivamente con una pose antigua. Debe:

```text
1. guardarlo en RawMapDatabase;
2. asignarle una pose provisional en GlobalPoseStore si es posible;
3. marcarlo como pendiente de rebase;
4. tras la optimización aceptada, aplicarle la corrección heredable;
5. después de rebasarlo, volver a procesarlo por el pipeline de loops.
```

La subfase 1R implementa y valida esa última parte:

```text
KF llegado durante optimización
        ↓
rebase tras optimización aceptada
        ↓
cola post-optimización
        ↓
LoopDetector
        ↓
SubcloudLoopVerifier
        ↓
LoopDecisionManager
        ↓
fusión u optimización si procede
```

Esta subfase no crea un nuevo detector de loops ni un nuevo optimizador. Debe reutilizar lo ya creado:

- 1N: `LoopDetector`;
- 1O: `SubcloudLoopVerifier`;
- 1P: `LoopDecisionManager`, `FusionManager`, `FusedLandmarkManager`, `LandmarkScoreManager`;
- 1Q: optimización por loop usando el pipeline 1I–1L;
- 1K: rebase de KFs llegados durante optimización;
- 1L: validación post-apply y rollback.

Regla principal:

```text
Un KF que llegó durante una optimización no queda completo hasta que,
si la optimización se acepta y se rebasa,
se vuelve a someter al pipeline de loops.
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
- `codex/pipeline/fase_1_sparse_global/subfases/subfase_1N.md`
- `codex/pipeline/fase_1_sparse_global/subfases/subfase_1O.md`
- `codex/pipeline/fase_1_sparse_global/subfases/subfase_1P.md`
- `codex/pipeline/fase_1_sparse_global/subfases/subfase_1Q.md`
- historial reciente de Fase 1;
- documentación de paquetes afectados:
  - `codex/contexto/paquetes/orbslam3_multi/`
  - `codex/contexto/paquetes/orbslam3_server/`
  - `codex/contexto/paquetes/simulacion_dron/`
- ADRs relacionados con:
  - `RawMapDatabase`;
  - `GlobalPoseStore`;
  - optimización;
  - rebase de KeyFrames;
  - loops;
  - fusión;
  - scoring;
  - colas/eventos si existen.

---

## Diagnostico de partida

Hasta esta subfase, el sistema debe tener:

1. `RawMapDatabase` insertando deltas/snapshots y guardando KFs nuevos.
2. `GlobalPoseStore` asignando poses globales a KFs de submapas anclados.
3. `GlobalPoseStore` guardando poses optimizadas, poses propagadas y correcciones heredables.
4. 1K definiendo que KFs llegados durante optimización se marcan como pendientes y se rebasan tras el apply.
5. 1L aceptando o rechazando optimizaciones.
6. 1N–1P procesando loops para KFs nuevos normales.
7. 1Q validando optimización por loop con el pipeline común.

El problema pendiente es:

```text
Un KF puede llegar mientras el servidor está optimizando.
Puede ser guardado y rebasado correctamente,
pero si no se vuelve a procesar por LoopDetector,
se pierden posibles loops/fusiones/optimizaciones asociadas a ese KF.
```

Además, aunque el KF haya sido procesado antes con una pose provisional, ese resultado puede haber quedado obsoleto después de aplicar la optimización.

Por tanto, tras aceptar una optimización y rebasar KFs pendientes, esos KFs deben entrar en una cola post-optimización y ser procesados como KFs nuevos para loops.

---

## Archivos permitidos a modificar

### `orbslam3_multi`

Se pueden modificar o crear archivos auxiliares relacionados con cola post-optimización y estados de KeyFrames:

- `orbslam3_multi/include/orbslam3_multi/global_pose_store.hpp`
- `orbslam3_multi/src/global_pose_store.cpp`
- `orbslam3_multi/include/orbslam3_multi/optimization_manager.hpp`
- `orbslam3_multi/src/optimization_manager.cpp`
- `orbslam3_multi/include/orbslam3_multi/optimization_result.hpp`
- `orbslam3_multi/include/orbslam3_multi/post_optimization_kf_queue.hpp`
- `orbslam3_multi/src/post_optimization_kf_queue.cpp`
- `orbslam3_multi/include/orbslam3_multi/loop_detector.hpp`
- `orbslam3_multi/src/loop_detector.cpp`
- `orbslam3_multi/include/orbslam3_multi/subcloud_loop_verifier.hpp`
- `orbslam3_multi/src/subcloud_loop_verifier.cpp`
- `orbslam3_multi/include/orbslam3_multi/loop_decision_manager.hpp`
- `orbslam3_multi/src/loop_decision_manager.cpp`
- `orbslam3_multi/include/orbslam3_multi/raw_map_database.hpp`
- `orbslam3_multi/src/raw_map_database.cpp`
- `orbslam3_multi/CMakeLists.txt`
- `orbslam3_multi/package.xml` solo si fuese necesario

No es obligatorio crear `PostOptimizationKeyFrameQueue` si se puede implementar con una estructura clara ya existente, pero debe quedar documentado.

### `orbslam3_server`

Cambios mínimos para orquestar la cola post-optimización:

- `orbslam3_server/src/global_map_server.cpp`
- clases auxiliares del servidor nuevo si existen
- launch nuevo del servidor si necesita parámetros debug

### `codex/archivos_auxiliares`

- `codex/archivos_auxiliares/trayectorias/tray_prueba_1.yaml`
- `codex/archivos_auxiliares/trayectorias/tray_prueba_2.yaml`
- `codex/archivos_auxiliares/trayectorias/tray_prueba_3.yaml`
- datasets/replay creados en fases anteriores
- logs/export debug opcionales:
  - `codex/archivos_auxiliares/f1q_post_opt_queue_<id>.csv`
  - `codex/archivos_auxiliares/f1q_loop_reprocess_<id>.txt`

### Documentación

- `codex/contexto/paquetes/orbslam3_multi/global_pose_store.md`
- `codex/contexto/paquetes/orbslam3_multi/optimization_manager.md`
- `codex/contexto/paquetes/orbslam3_multi/loop_detector.md`
- `codex/contexto/paquetes/orbslam3_multi/post_optimization_kf_queue.md` si se crea
- `codex/contexto/paquetes/orbslam3_server/global_map_server.md`
- historial de Fase 1 / subfase 1R

---


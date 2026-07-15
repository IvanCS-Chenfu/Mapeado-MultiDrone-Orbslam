# Subfase 1O — `SubcloudLoopVerifier`: subnubes, matching ORB, reducción espacial y RANSAC

## Estado

```text
sin hacer
```

---

## Objetivo tecnico

Crear la primera versión de verificación geométrica de candidatos de loop.

La subfase 1N genera candidatos visuales por BoW:

```text
query_kf_id
candidate_seed_kf_id
bow_score
```

La subfase 1O debe tomar esos candidatos y comprobar si realmente representan la misma zona del mapa mediante subnubes 3D, matching de descriptores ORB y RANSAC.

La idea central es:

```text
LoopCandidate(query_kf, candidate_seed_kf)
        ↓
consultar CovisibilityDatabase por seguridad
        ↓
SubcloudLoopVerifier
        ↓
query_subcloud
candidate_subcloud
        ↓
matching ORB
        ↓
reducción robusta de candidate_subcloud usando zona de matches
        ↓
matching refinado
        ↓
RANSAC 3D-3D
        ↓
LoopVerificationResult
```

La subfase 1O debe calcular el error geométrico del loop y devolver una decisión preliminar:

```text
REJECT
HOLD_INSUFFICIENT_EVIDENCE
FUSION_CANDIDATE
LOOP_OPTIMIZATION_CANDIDATE
```

En esta subfase **no se llama todavía a `FusionManager` ni a `OptimizationManager`**. La llamada real a fusión u optimización se hará en la fase siguiente.

Si el par `query_kf`/`candidate_seed_kf` ya existe en `CovisibilityDatabase`,
1O debe saltar la verificación costosa y devolver un resultado diagnóstico tipo
`ALREADY_CONFIRMED_COVISIBILITY` o equivalente. No debe repetir subnubes/RANSAC.

Si la geometría confirma el loop, el `LoopVerificationResult` debe conservar
evidencia suficiente para que 1P pueda añadir la relación confirmada a
`CovisibilityDatabase`:

```text
relative_pose_measured = transformación RANSAC/subnube
inlier_mappoint_pairs
ransac_inliers
mean_residual
loop_confidence
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
  - BoW/FeatureVector;
  - loops;
  - submapas `(drone_id, map_epoch)`;
  - separación fiducial/loop;
  - no usar GT para loops.

---

## Diagnostico de partida

Hasta esta subfase, el sistema nuevo debe tener:

1. `RawMapDatabase` con KeyFrames, MapPoints, BoW, descriptores, observaciones y poses locales originales.
2. `GlobalPoseStore` con poses globales para KeyFrames anclados/corregidos.
3. `LandmarkScoreManager` con score inicial de landmarks basado en datos raw de ORB-SLAM3.
4. `GlobalMapBuilder` publicando nube básica con score.
5. `CovisibilityDatabase` de 1M con covisibilidad confirmada.
6. `LoopDetector` de 1N generando candidatos BoW y saltando pares ya confirmados.

El problema actual es que un candidato BoW todavía no confirma un loop. Dos KeyFrames pueden parecerse visualmente pero:

- no estar en el mismo sitio;
- compartir apariencia repetida;
- mirar zonas cercanas pero no idénticas;
- tener pocos puntos en común;
- estar desplazados en el mapa por deriva;
- ser falsos positivos.

La subfase 1O debe añadir la verificación geométrica.

La verificación geométrica no debe insertar directamente en la base de
covisibilidad confirmada. 1O produce evidencia; 1P decide e inserta.

Punto de diseño clave:

```text
La candidate_subcloud NO debe ser simplemente:
    "los MapPoints vistos por el candidate_seed_kf".

Debe ser:
    "una subnube local del mapa global alrededor de la zona donde probablemente está el candidato".
```

El `candidate_seed_kf` del BoW es solo una semilla visual para construir una subnube global más robusta.

---

## Archivos permitidos a modificar

### `orbslam3_multi`

Se pueden crear o modificar archivos relacionados con subnubes y verificación de loop:

- `orbslam3_multi/include/orbslam3_multi/subcloud_loop_verifier.hpp`
- `orbslam3_multi/src/subcloud_loop_verifier.cpp`
- `orbslam3_multi/include/orbslam3_multi/loop_verification_result.hpp`
- `orbslam3_multi/include/orbslam3_multi/subcloud.hpp`
- `orbslam3_multi/include/orbslam3_multi/loop_detector.hpp`
- `orbslam3_multi/src/loop_detector.cpp`
- `orbslam3_multi/include/orbslam3_multi/loop_candidate.hpp`
- `orbslam3_multi/include/orbslam3_multi/covisibility_database.hpp`
- `orbslam3_multi/include/orbslam3_multi/raw_map_database.hpp`
- `orbslam3_multi/src/raw_map_database.cpp`
- `orbslam3_multi/include/orbslam3_multi/global_pose_store.hpp`
- `orbslam3_multi/src/global_pose_store.cpp`
- `orbslam3_multi/include/orbslam3_multi/landmark_score_manager.hpp`
- `orbslam3_multi/src/landmark_score_manager.cpp`
- `orbslam3_multi/CMakeLists.txt`
- `orbslam3_multi/package.xml` solo si fuese necesario

Si existen clases legacy de matching/verificación geométrica, pueden consultarse o adaptarse de forma controlada, sin reactivar rutas antiguas peligrosas.

### `orbslam3_server`

Cambios mínimos para invocar `SubcloudLoopVerifier` tras `LoopDetector`:

- `orbslam3_server/src/global_map_server.cpp`
- clases auxiliares del servidor nuevo si existen
- launch nuevo del servidor si necesita parámetros debug

### `codex/archivos_auxiliares`

- `codex/archivos_auxiliares/trayectorias/tray_prueba_1.yaml`
- `codex/archivos_auxiliares/trayectorias/tray_prueba_2.yaml`
- datasets/replay creados en 1C/1E/1N
- salidas debug opcionales:
  - `codex/archivos_auxiliares/f1n_subcloud_query_<id>.csv`
  - `codex/archivos_auxiliares/f1n_subcloud_candidate_<id>.csv`
  - `codex/archivos_auxiliares/f1n_matches_<id>.csv`
  - `codex/archivos_auxiliares/f1n_debug_<id>.svg`

### Documentación

- `codex/contexto/paquetes/orbslam3_multi/subcloud_loop_verifier.md`
- `codex/contexto/paquetes/orbslam3_multi/loop_detector.md`
- `codex/contexto/paquetes/orbslam3_multi/raw_map_database.md`
- `codex/contexto/paquetes/orbslam3_multi/global_pose_store.md`
- `codex/contexto/paquetes/orbslam3_server/global_map_server.md`
- historial de Fase 1 / subfase 1O

---

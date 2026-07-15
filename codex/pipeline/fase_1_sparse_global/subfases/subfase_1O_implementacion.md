## Archivos prohibidos

No modificar:

- `ORB_SLAM3/`, salvo permiso explícito.
- `orbslam3_ros2` / wrapper, salvo bug crítico demostrado.
- `orbslam3_msgs`, salvo necesidad justificada y aprobada.
- `global_map_server_antiguo.cpp`, salvo consulta.
- archivos legacy de `orbslam3_multi/legacy/`, salvo consulta.
- lógica de fusión de landmarks.
- lógica de aplicación de optimización.
- `GlobalPoseStore` para mover poses por loops en esta subfase.
- `RawMapDatabase` para corregir poses.
- rutas de fiducial ya validadas salvo integración mínima.
- `build/`, `install/`, `log/` como código fuente.

No borrar código legacy de forma masiva.

---

## Funciones, clases o nodos que hay que localizar

`planificador_fase` debe localizar o crear, según exista o no:

### En `orbslam3_multi`

- `RawMapDatabase`
- `GlobalPoseStore`
- `LandmarkScoreManager`
- `LoopDetector`
- `LoopCandidate`
- `CovisibilityDatabase`
- `SubcloudLoopVerifier`
- `Subcloud`
- `SubcloudPoint`
- `DescriptorMatch`
- `Ransac3D3DResult`
- `LoopVerificationResult`

Métodos o datos necesarios:

- obtener KeyFrame por ID;
- obtener MapPoints observados por un KeyFrame;
- obtener descriptores asociados a observaciones KF-MP;
- obtener BoW/FeatureVector si se necesita;
- obtener covisibilidad de un KF;
- obtener parent/children/spanning tree si existe;
- obtener vecinos temporales;
- obtener pose global de un KF;
- obtener score de un MapPoint;
- transformar un MapPoint local/raw a `world`.
- consultar si un par de KFs ya tiene covisibilidad confirmada.

### En `orbslam3_server`

- punto donde se procesan resultados de `LoopDetector`;
- punto donde se invoca `SubcloudLoopVerifier`;
- logs de loop;
- parámetros debug de 1O;
- replay si existe.

---

## Cambios requeridos

### 1. Crear estructura `SubcloudPoint`

Cada punto de subnube debe contener información útil para matching, RANSAC y logging.

Campos recomendados:

```text
mappoint_id
source_kf_id
submap_id
position_world
descriptor
score
num_observations
is_bad
keypoint_index opcional
source = QUERY_KF | CANDIDATE_WINDOW
```

Condición de seguridad:

- no incluir puntos sin posición válida;
- no incluir puntos sin descriptor válido;
- no incluir puntos marcados bad salvo modo debug explícito.

---

### 2. Crear estructura `Subcloud`

Campos recomendados:

```text
cloud_id
type = QUERY | CANDIDATE_INITIAL | CANDIDATE_REDUCED
center_kf_id
seed_kf_id si aplica
submap_ids
source_kf_ids
points
bbox
stats
```

Stats mínimos:

```text
raw_points
filtered_points
bad_removed
no_descriptor_removed
low_score_removed
duplicates_removed
final_points
```

---

### 3. Crear `LoopVerificationResult`

Debe representar el resultado completo de verificar un candidato BoW.

Campos recomendados:

```text
query_kf_id
candidate_seed_kf_id
query_submap_id
candidate_submap_id
bow_score
already_confirmed_covisibility
confirmed_covisibility_source opcional
query_points
candidate_initial_points
candidate_reduced_points
initial_matches
refined_matches
ransac_inliers
inlier_ratio
mean_residual
max_residual
error_t
error_yaw
error_rot
estimated_candidate_T_query
relative_pose_measured
inlier_mappoint_pairs
loop_confidence
decision
reason
```

Decisiones permitidas:

```text
REJECT
HOLD_INSUFFICIENT_EVIDENCE
FUSION_CANDIDATE
LOOP_OPTIMIZATION_CANDIDATE
ALREADY_CONFIRMED_COVISIBILITY opcional
```

Esta decisión es preliminar. La fase siguiente decidirá si llama a fusión u optimización.

Logs:

```text
[F1N-SUBCLOUD-DECISION]
```

---

### 4. Crear `SubcloudLoopVerifier`

Nueva clase en `orbslam3_multi`.

Responsabilidad:

```text
Tomar un LoopCandidate BoW,
descartar temprano si el par ya está en CovisibilityDatabase,
construir query_subcloud y candidate_subcloud,
hacer matching ORB,
reducir candidate_subcloud con una región robusta basada en matches,
rehacer matching,
ejecutar RANSAC 3D-3D,
calcular error geométrico y devolver LoopVerificationResult.
```

Método mínimo recomendado:

```cpp
LoopVerificationResult VerifyCandidate(
    const LoopCandidate& candidate,
    const RawMapDatabase& raw_db,
    const GlobalPoseStore& pose_store,
    const CovisibilityDatabase* covis_db_optional,
    const LandmarkScoreManager* score_manager_optional);
```

Condiciones de seguridad:

- no fusionar;
- no crear `LoopOptimizationTask`;
- no optimizar;
- no modificar `GlobalPoseStore`;
- no insertar en `CovisibilityDatabase`; eso pertenece a 1P;
- no usar GT.

Logs:

```text
[F1N-SUBCLOUD-VERIFY-START]
[F1O-SUBCLOUD-SKIP-CONFIRMED-COVIS]
```

### 4.1. Consultar `CovisibilityDatabase` antes de construir subnubes

Si el candidato llega a 1O aunque ya esté confirmado, saltar todo el proceso
caro:

```text
if covis_db.HasConfirmedEdge(query_kf, candidate_seed_kf):
    result.decision = ALREADY_CONFIRMED_COVISIBILITY
    result.already_confirmed_covisibility = true
    return result
```

Esta salida no rechaza el loop; indica que la relación ya es conocida y usable
por optimización/fusión según corresponda.

---

### 5. Construir `query_subcloud`

La `query_subcloud` se construye con los MapPoints observados por el KeyFrame nuevo `query_kf_id`.

Proceso:

1. obtener `query_kf` desde `RawMapDatabase`;
2. obtener sus MapPoints observados;
3. filtrar MapPoints inválidos;
4. obtener descriptor asociado;
5. transformar cada punto a `world` usando `GlobalPoseStore`;
6. asociar score desde `LandmarkScoreManager` si existe;
7. deduplicar si procede.

Filtros mínimos:

```text
MapPoint no bad
posición válida
descriptor válido
observación válida desde query_kf
score >= query_subcloud_min_score si se habilita
profundidad válida si aplica
```

Si el query KF no tiene pose global, la verificación debe devolver:

```text
REJECT o HOLD_INSUFFICIENT_EVIDENCE
reason=query_no_world_pose
```

Logs:

```text
[F1N-SUBCLOUD-QUERY-BUILD]
```

Ejemplo:

```text
[F1N-SUBCLOUD-QUERY-BUILD] query_kf=120 raw_points=180 filtered_points=132 no_descriptor=12 bad=5 final_points=132
```

---

### 6. Usar `candidate_seed_kf` solo como semilla

El KeyFrame candidato BoW no debe usarse directamente como candidate subcloud completa.

Regla obligatoria:

```text
candidate_seed_kf es semilla visual.
candidate_subcloud representa la zona global alrededor de esa semilla.
```

Antes de construir la ventana, validar:

```text
candidate_seed_kf existe
candidate_seed_kf no bad
candidate_seed_kf tiene pose global
candidate_seed_kf tiene suficientes MapPoints
candidate_seed_kf pertenece a submapa usable
```

Logs:

```text
[F1N-SUBCLOUD-CANDIDATE-SEED]
```

---

### 7. Construir ventana local de KeyFrames candidate

Construir `candidate_kf_window` alrededor de `candidate_seed_kf`.

Orden de expansión recomendado:

1. incluir `candidate_seed_kf`;
2. añadir KFs covisibles fuertes;
3. añadir parent/children del spanning tree;
4. añadir KFs temporalmente cercanos dentro del mismo submapa;
5. añadir KFs cercanos espacialmente en `world`, si tienen pose global;
6. en fases futuras, añadir KFs por fused tracks.

La expansión debe limitarse por parámetros.

Parámetros recomendados:

```text
candidate_window_max_kfs
candidate_window_covisibility_min_weight
candidate_window_max_graph_depth
candidate_window_spatial_radius_m
candidate_window_temporal_kf_radius
candidate_window_max_points
candidate_window_anchor_stop_enabled
```

Condiciones de parada:

```text
max_kfs alcanzado
max_points alcanzado
radio espacial superado
se cruza a zona de fiducial fuerte no deseada
se cruza a submapa sin relación clara
KF sin pose global
KF bad
```

Logs:

```text
[F1N-SUBCLOUD-CANDIDATE-WINDOW]
```

Ejemplo:

```text
[F1N-SUBCLOUD-CANDIDATE-WINDOW] query_kf=120 seed_kf=34 window_kfs=14 covisible=8 tree=3 temporal=3 spatial=0
```

---

### 8. Construir `candidate_subcloud_initial`

Con la ventana candidate:

```text
candidate_subcloud_initial =
    unión filtrada de MapPoints observados por candidate_kf_window
```

Filtros mínimos:

```text
MapPoint no bad
posición global válida
descriptor válido
score mínimo si se habilita
observado por al menos N KFs si se habilita
no duplicado
dentro de radio/bbox de candidate window
```

Dedupe mínimo:

```text
deduplicar por mappoint_id
```

Futuro:

```text
deduplicar por FusedLandmarkTrack cuando exista FusedLandmarkManager
```

Si hay demasiados puntos, muestrear por:

```text
score alto
más observaciones
distribución espacial
cercanía a candidate_seed
descriptor válido
```

Parámetros recomendados:

```text
candidate_subcloud_min_points
candidate_subcloud_max_points
candidate_subcloud_min_score
candidate_subcloud_voxel_size
candidate_subcloud_min_observations
```

Logs:

```text
[F1N-SUBCLOUD-CANDIDATE-BUILD]
```

Ejemplo:

```text
[F1N-SUBCLOUD-CANDIDATE-BUILD] seed_kf=34 raw_points=1840 filtered_points=620 score_min=0.25
```

---

### 9. Matching ORB inicial

Hacer matching de descriptores ORB entre:

```text
query_subcloud
candidate_subcloud_initial
```

Aplicar:

```text
distancia Hamming máxima
ratio test si procede
cross-check si procede
evitar matches duplicados
```

Parámetros recomendados:

```text
orb_match_max_hamming
orb_match_ratio_test
orb_match_cross_check
min_initial_matches
```

La salida es una lista de correspondencias 3D-3D:

```text
p_query_world
p_candidate_world
descriptor_distance
```

Logs:

```text
[F1N-SUBCLOUD-MATCH]
```

---

### 10. Reducir `candidate_subcloud` usando zona espacial de matches

Si hay suficientes matches iniciales, reducir la candidate subcloud usando la distribución espacial de los puntos candidate matcheados.

Objetivo:

```text
Reducir la candidate_subcloud a la zona global donde parecen estar los matches,
sin depender solo del candidate_seed_kf.
```

Proceso recomendado:

1. tomar posiciones `world` de los puntos candidate que han matcheado;
2. calcular región robusta:
   - bounding box robusta por percentiles; o
   - centro mediano + radio robusto; o
   - voxels con matches y margen;
3. ampliar con margen;
4. conservar solo puntos candidate dentro de esa región;
5. si quedan pocos puntos, hacer fallback.

Regla:

```text
No usar min/max de todos los matches sin robustez,
porque outliers pueden agrandar demasiado la caja.
```

Primera implementación recomendada:

```text
bounding box robusta:
    percentiles 10–90 en x,y,z
    + margen configurable
```

Parámetros:

```text
candidate_reduce_enabled
candidate_reduce_min_initial_matches
candidate_reduce_percentile_low
candidate_reduce_percentile_high
candidate_reduce_margin_m
candidate_reduce_min_points_after
candidate_reduce_fallback_to_initial
```

Logs:

```text
[F1N-SUBCLOUD-CANDIDATE-REDUCE-MATCH-BOX]
[F1N-SUBCLOUD-CANDIDATE-REDUCE-STATS]
```

Ejemplo:

```text
[F1N-SUBCLOUD-CANDIDATE-REDUCE-STATS] initial_points=1840 initial_matches=96 robust_box_points=430 margin=0.75 fallback=false
```

Fallback:

```text
[F1N-SUBCLOUD-CANDIDATE-REDUCE-STATS] initial_points=1840 initial_matches=12 fallback=true reason=not_enough_matches
```

---

### 11. Matching ORB refinado

Después de reducir `candidate_subcloud`, repetir matching:

```text
query_subcloud
candidate_subcloud_reduced
```

Esto limpia matches y reduce coste de RANSAC.

Logs:

```text
[F1N-SUBCLOUD-MATCH-REFINED]
```

Debe mostrar:

```text
refined_matches
duplicates_removed
mean_descriptor_distance
```

Si no hay suficientes matches refinados:

```text
decision=REJECT o HOLD_INSUFFICIENT_EVIDENCE
reason=not_enough_refined_matches
```

---

### 12. RANSAC 3D-3D

Con correspondencias 3D-3D refinadas, estimar una transformación rígida:

```text
candidate_T_query_ransac
```

o convención equivalente documentada.

Esta transformación representa:

```text
cómo mover la query_subcloud para alinearla con candidate_subcloud.
```

RANSAC debe devolver:

```text
success
inliers
inlier_ratio
mean_residual
max_residual
estimated_transform
degenerate
```

Parámetros recomendados:

```text
ransac_min_matches
ransac_max_iterations
ransac_inlier_threshold_m
ransac_min_inliers
ransac_min_inlier_ratio
ransac_degeneracy_check_enabled
```

Logs:

```text
[F1N-SUBCLOUD-RANSAC]
```

---

### 13. Calcular error de loop

Distinguir dos errores:

#### A. Error de alineación

Mide si las nubes realmente parecen la misma zona:

```text
mean_residual
max_residual
inliers
inlier_ratio
```

Si es malo:

```text
decision=REJECT
```

#### B. Error de pose global actual

Si RANSAC encontró buena alineación, medir cuánto habría que mover la query para coincidir con candidate:

```text
error_t
error_yaw
error_rot
```

Interpretación:

```text
error bajo:
    las nubes ya están casi en el mismo sitio → FUSION_CANDIDATE

error alto:
    las nubes parecen la misma zona pero están separadas → LOOP_OPTIMIZATION_CANDIDATE
```

Logs:

```text
[F1N-SUBCLOUD-ERROR]
```

Ejemplo:

```text
[F1N-SUBCLOUD-ERROR] query_kf=120 seed_kf=34 matches=81 inliers=52 ratio=0.64 mean_residual=0.07 error_t=0.18 error_yaw=0.04 decision=FUSION_CANDIDATE
```

---

### 14. Decisión preliminar

Reglas iniciales:

#### `REJECT`

Usar si:

```text
pocos puntos
pocos matches
pocos inliers
inlier_ratio bajo
mean_residual alto
max_residual alto
transformación degenerada
candidate_seed no usable
query sin pose global
```

#### `HOLD_INSUFFICIENT_EVIDENCE`

Usar si:

```text
BoW alto pero subnubes pequeñas
matches justos
inlier_ratio medio
resultado prometedor pero insuficiente
```

#### `FUSION_CANDIDATE`

Usar si:

```text
inliers buenos
residual bajo
error_t bajo
error_yaw bajo
```

Significa:

```text
las nubes ya están casi en el mismo sitio.
```

#### `LOOP_OPTIMIZATION_CANDIDATE`

Usar si:

```text
inliers buenos
residual bajo
pero error_t/error_yaw alto
```

Significa:

```text
parecen la misma zona,
pero el mapa las tiene separadas.
```

La fase siguiente decidirá si llamar a fusión o crear `LoopOptimizationTask`.

Logs:

```text
[F1N-SUBCLOUD-DECISION]
```

---

### 15. Integración con servidor y `LoopDetector`

El servidor/backend debe tomar candidatos de 1N y pasarlos a `SubcloudLoopVerifier`.

Flujo:

```text
LoopDetector → LoopCandidate(s)
        ↓
SubcloudLoopVerifier.VerifyCandidate(...)
        ↓
LoopVerificationResult
        ↓
logs
```

En 1O no debe ejecutarse ninguna acción posterior salvo loggear y opcionalmente guardar resultados debug.

Logs del servidor:

```text
[F1N-SERVER-VERIFY-CANDIDATE-CALL]
[F1N-SERVER-VERIFY-CANDIDATE-RESULT]
```

---

### 16. Export debug opcional

Si es viable, exportar subnubes/matches a archivos para inspección.

Opciones:

```text
CSV query_subcloud
CSV candidate_subcloud_initial
CSV candidate_subcloud_reduced
CSV matches/refined matches
SVG/PNG 2D con puntos y matches
```

Logs:

```text
[F1N-SUBCLOUD-DEBUG-EXPORT]
```

No es obligatorio si los logs son suficientes, pero es recomendable para depuración.

---

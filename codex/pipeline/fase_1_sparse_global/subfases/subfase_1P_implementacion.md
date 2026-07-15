## Archivos prohibidos

No modificar:

- `ORB_SLAM3/`, salvo permiso explícito.
- `orbslam3_ros2` / wrapper, salvo bug crítico demostrado.
- `orbslam3_msgs`, salvo necesidad justificada y aprobada.
- `global_map_server_antiguo.cpp`, salvo consulta.
- archivos legacy de `orbslam3_multi/legacy/`, salvo consulta.
- `RawMapDatabase` para borrar o fusionar MapPoints originales.
- `GlobalPoseStore` para mover poses por fusión.
- rutas de fiducial ya validadas salvo integración mínima.
- aplicación real de optimización por loop si no pasa por 1I–1L.
- `build/`, `install/`, `log/` como código fuente.

No borrar código legacy de forma masiva.

---

## Funciones, clases o nodos que hay que localizar

`planificador_fase` debe localizar o crear, según exista o no:

### En `orbslam3_multi`

- `LoopVerificationResult`
- `SubcloudLoopVerifier`
- `CovisibilityDatabase`
- `LoopDecisionManager`
- `FusionManager`
- `FusedLandmarkManager`
- `FusedLandmarkTrack`
- `LandmarkScoreManager`
- `GlobalMapBuilder`
- `RawMapDatabase`
- `GlobalPoseStore`
- `LoopOptimizationTask`
- `PoseGraphBuilder`
- `OptimizationManager`

Métodos o datos necesarios:

- pares de inliers de RANSAC de 1O;
- MapPoint IDs asociados a cada correspondencia;
- posiciones world actuales;
- descriptors ORB de MapPoints;
- scores actuales;
- número de observaciones;
- submap/drone/epoch de cada MapPoint;
- estado bad/replaced si existe;
- resultado post-apply de 1L para loops futuros.
- API para insertar/consultar aristas confirmadas de covisibilidad.

### En `orbslam3_server`

- punto donde llega `LoopVerificationResult`;
- punto donde se invoca `LoopDecisionManager`;
- punto donde se publican estadísticas de fusión;
- punto donde `GlobalMapBuilder` publica nube fused/raw;
- parámetros debug para forzar decisiones controladas.

---

## Cambios requeridos

### 1. Crear `LoopDecisionManager`

Nueva clase en `orbslam3_multi`.

Responsabilidad:

```text
Tomar un LoopVerificationResult y decidir qué acción ejecutar:
REJECT, HOLD, FUSION, LOOP_OPTIMIZATION_TASK.
```

Método mínimo recomendado:

```cpp
LoopDecisionResult ProcessVerificationResult(
    const LoopVerificationResult& result,
    const RawMapDatabase& raw_db,
    const GlobalPoseStore& pose_store,
    CovisibilityDatabase& covis_db,
    FusionManager& fusion_manager,
    LandmarkScoreManager& score_manager);
```

o una interfaz equivalente.

Reglas:

```text
REJECT:
    no insertar covisibilidad.
    no fusionar, no optimizar.
    opcionalmente bajar score solo si hay evidencia fuerte de inconsistencia.

HOLD:
    no insertar covisibilidad.
    guardar diagnóstico.
    no tocar score salvo eventos suaves.

FUSION_CANDIDATE:
    insertar covisibilidad confirmada en CovisibilityDatabase.
    llamar a FusionManager.
    actualizar scores por eventos semánticos.
    reconstruir/publicar mapa.

LOOP_OPTIMIZATION_CANDIDATE:
    insertar covisibilidad confirmada en CovisibilityDatabase.
    crear LoopOptimizationTask.
    no fusionar todavía.
    después de optimización aceptada por 1L, permitir fusión.
```

Logs:

```text
[F1O-LOOP-DECISION]
[F1P-COVIS-EDGE-CONFIRMED]
```

---

### 2. Crear `LoopDecisionResult`

Campos recomendados:

```text
loop_result_id
query_kf_id
candidate_seed_kf_id
decision
action_taken
created_loop_optimization_task
fusion_executed
fusion_deferred
reason
num_fused_pairs
num_unmatched_penalized
score_events
covisibility_edge_inserted
covisibility_edge_id opcional
```

Decisiones/acciones:

```text
REJECT_NO_ACTION
HOLD_NO_ACTION
FUSION_APPLIED
LOOP_OPT_TASK_CREATED
FUSION_DEFERRED_UNTIL_OPT_ACCEPT
```

Logs:

```text
[F1O-LOOP-DECISION-SUMMARY]
```

---

### 3. Crear `FusionManager` / `FusedLandmarkManager`

Separar responsabilidades:

```text
FusionManager:
    recibe pares de MapPoints candidatos a fusionar y decide/ejecuta la fusión.

FusedLandmarkManager:
    mantiene tracks fusionados persistentes.
```

Estructura recomendada:

```text
FusedLandmarkTrack:
    fused_id
    member_mappoint_ids
    source_drone_ids
    source_submap_ids
    observing_keyframes
    fused_position_world
    representative_descriptor
    score
    confidence
    last_update_time
    creation_source
```

Regla crítica:

```text
No borrar MapPoints originales de RawMapDatabase.
```

Los MapPoints raw siguen existiendo. El track fused es una capa global encima.

Logs:

```text
[F1O-FUSION-EVENT]
[F1O-FUSED-TRACK-CREATE]
[F1O-FUSED-TRACK-UPDATE]
```

---

### 3.1. Insertar covisibilidad confirmada

Cuando 1O devuelva `FUSION_CANDIDATE` o `LOOP_OPTIMIZATION_CANDIDATE`, 1P debe
añadir una arista confirmada:

```text
source = SERVER_LOOP_GEOMETRIC
kf_a = query_kf
kf_b = candidate_seed_kf
weight = ransac_inliers o soporte equivalente
relative_pose_measured = result.relative_pose_measured
relative_pose_current = result.relative_pose_measured inicialmente
information_weight = función conservadora de inliers/residual/confianza
```

No insertar en `REJECT`, `HOLD` ni por BoW solo.

Si la arista ya existe, no crear duplicado; actualizar soporte o
`relative_pose_current` solo si la nueva evidencia es más fuerte y queda
documentado.

Logs:

```text
[F1P-COVIS-EDGE-CONFIRMED]
[F1P-COVIS-EDGE-ALREADY-KNOWN]
```

---

### 4. Fusionar inlier pairs

Cuando 1O devuelve `FUSION_CANDIDATE`, usar los pares inlier de RANSAC.

Cada inlier debe mapear:

```text
query_mappoint_id ↔ candidate_mappoint_id
```

Para cada par:

1. validar que ambos MapPoints existen;
2. validar que no están bad;
3. validar que tienen posición world válida;
4. validar que la distancia post-alineación es baja;
5. validar descriptor razonable;
6. añadir ambos miembros a un `FusedLandmarkTrack`.

Si alguno de los MapPoints ya pertenece a un track fused, actualizar ese track en vez de crear otro duplicado.

Logs:

```text
[F1O-FUSION-PAIR]
[F1O-FUSION-PAIR-REJECT]
```

---

### 5. Calcular posición fused por media ponderada

Para cada `FusedLandmarkTrack`, calcular:

```text
p_fused = sum(w_i * p_i) / sum(w_i)
```

Pesos iniciales permitidos:

```text
w_i = max(score_i, epsilon)
```

o:

```text
w_i = score_i * max(num_observations_i, 1)
```

Usar una política simple en 1P y documentarla.

Factores futuros:

```text
residual geométrico
confianza de submapa
si viene de KF optimizado
soporte multi-dron
soporte multi-fiducial
```

Logs:

```text
[F1O-FUSED-POSITION-UPDATE]
```

---

### 6. Descriptor fused

No hacer media aritmética normal de descriptores ORB.

Opciones permitidas:

#### Opción inicial recomendada: descriptor representativo

Elegir el descriptor del miembro con mayor confianza:

```text
mayor score
mayor número de observaciones
menor residual de fusión
```

#### Opción alternativa: descriptor medoid

Elegir el descriptor miembro que minimiza la distancia Hamming media al resto.

#### Mejora futura: mayoría ponderada de bits

Para cada bit del ORB descriptor, elegir el bit mayoritario ponderado.

En 1P se recomienda usar `representative_descriptor` o `medoid`, no media float.

Logs:

```text
[F1O-FUSED-DESCRIPTOR-UPDATE]
```

Debe indicar la política usada:

```text
policy=representative
policy=medoid
```

---

### 7. Actualizar scores por eventos semánticos

No permitir que clases externas modifiquen scores con sumas/restas arbitrarias.

Regla:

```text
Las clases externas informan eventos.
LandmarkScoreManager decide cómo cambia el score.
```

Eventos mínimos para 1P:

```text
FusionConfirmed
FusedTrackConfirmed
ExpectedButUnmatchedInConfirmedOverlap
LoopRejectedGeometryInconsistent
OrbSlamQualityUpdate
MarkedBad
```

Cuando se fusionan puntos:

```text
score_manager.ApplyScoreEvent(mp_id, FusionConfirmed)
score_manager.ApplyScoreEvent(fused_id, FusedTrackConfirmed)
```

Cuando un punto debería haber tenido asociación y no la tuvo:

```text
score_manager.ApplyScoreEvent(mp_id, ExpectedButUnmatchedInConfirmedOverlap)
```

Logs:

```text
[F1O-SCORE-EVENT]
[F1O-SCORE-UPDATE]
```

---

### 8. Bajar score solo de puntos esperados pero no asociados

No bajar score de todos los puntos sin match.

Solo penalizar puntos que razonablemente deberían haber tenido asociación.

Condiciones recomendadas para penalizar:

```text
la verificación geométrica fue buena;
el punto está dentro de la zona de solape confirmada;
el punto está dentro de la región candidate/query comparable;
el punto tiene descriptor válido;
el punto no está bad;
el punto tiene score suficiente para ser esperado;
el punto no quedó fuera por filtrado geométrico justificado;
opcionalmente, el punto falla varias veces.
```

Evento:

```text
ExpectedButUnmatchedInConfirmedOverlap
```

No penalizar por:

```text
oclusiones no modeladas
puntos fuera del solape
subnube pequeña
HOLD
REJECT por falta de evidencia
```

Logs:

```text
[F1O-SCORE-UNMATCHED-CANDIDATE]
[F1O-SCORE-UNMATCHED-SKIP]
```

---

### 9. Integrar señales raw de ORB-SLAM3 con scoring

ORB-SLAM3 puede actualizar indirectamente calidad de MapPoints mediante señales como:

```text
número de observaciones
found ratio
visible/found counters si se exportan
isBad
replaced/merged local si se exporta
```

`RawMapDatabase` debe seguir detectando actualizaciones raw y emitir eventos o permitir que `LandmarkScoreManager` consulte esos datos:

```text
NewMapPoint
OrbSlamQualityUpdate
ObservationCountIncreased
ObservationCountDecreased
MarkedBad
```

En 1P se debe documentar claramente qué señales reales exporta el wrapper y cuáles no.

Logs:

```text
[F1O-SCORE-ORBSLAM-QUALITY-UPDATE]
```

---

### 10. Publicar preferentemente fused tracks

Actualizar `GlobalMapBuilder` para publicar:

```text
1. FusedLandmarkTrack si existe.
2. MapPoints raw no fusionados con score suficiente.
3. No publicar miembros raw de un fused track en la nube fused principal.
```

Esto evita duplicados:

```text
mp_10 + mp_25 → fused_3

Publicar:
    fused_3

No publicar en nube principal:
    mp_10
    mp_25
```

Se permite topic debug opcional:

```text
/global_sparse_cloud_raw_debug
/global_sparse_cloud_fused
```

Logs:

```text
[F1O-GLOBALMAP-FUSED-BUILD]
[F1O-GLOBALMAP-FUSED-PUBLISH]
[F1O-GLOBALMAP-SKIP-FUSED-MEMBER]
```

---

### 11. Crear `LoopOptimizationTask` cuando el error es alto

Si `LoopVerificationResult.decision == LOOP_OPTIMIZATION_CANDIDATE`, crear una tarea de optimización por loop.

La tarea debe contener:

```text
task_id
task_type = LOOP_ALIGNMENT_ERROR
query_kf_id
candidate_seed_kf_id
query_submap_id
candidate_submap_id
estimated_candidate_T_query
error_t
error_yaw
error_rot
inliers
inlier_ratio
mean_residual
loop_confidence
source = SUBCLOUD_RANSAC
```

Debe incluir suficiente información para que `PoseGraphBuilder` cree el grafo igual que con fiducial, cambiando constraints/pesos.

Condición:

```text
No fusionar antes de optimizar.
```

Logs:

```text
[F1O-LOOP-OPT-TASK-CREATED]
[F1O-FUSION-DEFERRED-UNTIL-OPT-ACCEPT]
```

---

### 12. Fusionar después de optimización aceptada

Si un loop necesitó optimización:

1. crear `LoopOptimizationTask`;
2. ejecutar pipeline 1I–1L;
3. si 1L devuelve ACCEPT:
   - revalidar o reutilizar inliers transformados si siguen siendo válidos;
   - fusionar landmarks;
   - actualizar scores;
   - publicar fused map.
4. si 1L devuelve PARTIAL:
   - no fusionar por defecto salvo política explícita muy conservadora;
   - dejar en HOLD/needs_second_pass.
5. si 1L devuelve REJECT:
   - no fusionar;
   - marcar loop candidate fallido.

Logs:

```text
[F1O-POST-OPT-FUSION-CHECK]
[F1O-POST-OPT-FUSION-APPLY]
[F1O-POST-OPT-FUSION-SKIP]
```

En 1P se puede implementar la estructura y una ruta debug; si no existe aún caso real de loop optimization, debe quedar documentado.

---

### 13. Integración con servidor

El servidor/backend debe orquestar:

```text
LoopDetector
    ↓
SubcloudLoopVerifier
    ↓
LoopDecisionManager
```

En `FUSION_CANDIDATE`, el servidor debe publicar mapa fused actualizado.

En `LOOP_OPTIMIZATION_CANDIDATE`, debe enviar la tarea al pipeline de optimización ya existente.

Logs del servidor:

```text
[F1O-SERVER-LOOP-DECISION-CALL]
[F1O-SERVER-LOOP-DECISION-RESULT]
```

---

### 14. Export debug opcional

Exportar:

```text
tracks fused
pares fusionados
scores antes/después
puntos penalizados
miembros raw omitidos en publicación
```

Archivos sugeridos:

```text
codex/archivos_auxiliares/f1o_fusion_tracks_<task_id>.csv
codex/archivos_auxiliares/f1o_score_events_<task_id>.csv
```

Logs:

```text
[F1O-DEBUG-EXPORT]
```

---

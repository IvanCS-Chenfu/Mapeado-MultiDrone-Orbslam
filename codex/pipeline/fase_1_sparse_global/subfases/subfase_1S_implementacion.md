## Archivos prohibidos

No modificar:

- `ORB_SLAM3/`, salvo permiso explícito.
- `orbslam3_ros2` / wrapper, salvo bug crítico demostrado.
- `orbslam3_msgs`, salvo necesidad justificada y aprobada.
- `global_map_server_antiguo.cpp`, salvo consulta.
- archivos legacy de `orbslam3_multi/legacy/`, salvo consulta.
- `RawMapDatabase` para borrar MapPoints raw.
- `GlobalPoseStore` para mover poses.
- optimización de poses.
- detección BoW.
- RANSAC/subnubes.
- fusión geométrica nueva no prevista en 1P.
- uso de GT para loops o scoring.
- `build/`, `install/`, `log/` como código fuente.

No borrar código legacy de forma masiva.

---

## Funciones, clases o nodos que hay que localizar

`planificador_fase` debe localizar:

### En `orbslam3_multi`

- `LandmarkScoreManager`
- `LandmarkScoreEvent`
- `LandmarkScorePolicy` o equivalente
- `RawMapDatabase`
- `FusionManager`
- `FusedLandmarkManager`
- `FusedLandmarkTrack`
- `GlobalMapBuilder`
- `LoopDecisionManager`
- `OptimizationManager`
- estructuras de apply/rollback si afectan a score
- eventos raw de MapPoints si existen

Métodos o equivalentes:

- registrar MapPoint nuevo;
- actualizar score por datos ORB-SLAM3;
- aplicar evento semántico;
- obtener score de MapPoint raw;
- obtener score de fused track;
- rollback de eventos de score;
- commit de eventos de score;
- exportar estadísticas de score;
- publicar nube con score.

### En `orbslam3_server`

- punto donde se insertan deltas;
- punto donde `RawMapDatabase` detecta MapPoints nuevos/actualizados;
- punto donde se aplican fusiones;
- punto donde se publica nube global;
- parámetros de scoring.

---

## Cambios requeridos

### 1. Definir eventos oficiales de score

Crear o consolidar un enum/lista central de eventos.

Eventos mínimos recomendados:

```text
NewMapPoint
OrbSlamQualityUpdate
ObservationCountIncreased
ObservationCountDecreased
MarkedBad
DescriptorValid
DescriptorInvalid
FusionConfirmed
FusedTrackConfirmed
ExpectedButUnmatchedInConfirmedOverlap
ReobservedInConfirmedOverlap
LoopRejectedGeometryInconsistent
OptimizationAccepted
OptimizationRejectedRollback
FusedTrackMemberAdded
FusedTrackMemberRemovedByRollback
```

Regla obligatoria:

```text
Ninguna clase externa puede hacer score += X o score -= Y.
Las clases externas solo emiten LandmarkScoreEvent.
LandmarkScoreManager traduce eventos a cambios de score.
```

Logs:

```text
[F1R-SCORE-EVENT]
```

---

### 2. Crear política única de cambios de score

`LandmarkScoreManager` debe tener una política explícita y documentada.

Recomendado:

```text
score en rango [0.0, 1.0]
clamp tras cada actualización
score inicial configurable
incrementos/decrementos configurables
```

Parámetros sugeridos:

```text
score_initial_default
score_min
score_max
score_new_mappoint_base
score_orbslam_quality_weight
score_fusion_confirmed_bonus
score_fused_track_confirmed_bonus
score_reobserved_bonus
score_expected_unmatched_penalty
score_marked_bad_value
score_decay_enabled
```

No hace falta que la política sea perfecta. Debe ser simple, trazable y modificable.

Logs:

```text
[F1R-SCORE-POLICY]
[F1R-SCORE-UPDATE]
```

Ejemplo:

```text
[F1R-SCORE-UPDATE] target=mp:123 event=FusionConfirmed old=0.54 new=0.71 delta=0.17 reason=fusion_confirmed
```

---

### 3. Consolidar score inicial/raw desde ORB-SLAM3

Cuando `RawMapDatabase` detecta MapPoints nuevos o actualizados, debe notificar a `LandmarkScoreManager` mediante eventos.

Señales raw posibles:

```text
número de observaciones
found ratio si se exporta
visible/found counters si se exportan
isBad
replaced si se exporta
descriptor válido
posición válida
MapPoint nuevo
MapPoint actualizado
```

Si el wrapper no exporta alguna métrica, documentar fallback.

Ejemplo de fallback:

```text
si no hay found_ratio:
    usar observaciones + isBad + descriptor válido + posición válida
```

Eventos esperados:

```text
NewMapPoint
OrbSlamQualityUpdate
ObservationCountIncreased
ObservationCountDecreased
MarkedBad
DescriptorInvalid
```

Logs:

```text
[F1R-SCORE-ORBSLAM-RAW]
[F1R-SCORE-ORBSLAM-QUALITY-UPDATE]
```

---

### 4. Definir score de MapPoint raw

Cada MapPoint raw debe tener score consultable.

Campos recomendados:

```text
mappoint_id
score
last_event
last_update_arrival_id
num_events
is_publishable
debug_reason
```

Métodos sugeridos:

```text
RegisterNewMapPoint(mp_id, raw_data)
ApplyScoreEvent(mp_id, event)
GetMapPointScore(mp_id)
GetMapPointScoreDebug(mp_id)
```

Regla:

```text
RawMapDatabase conserva datos raw.
LandmarkScoreManager conserva el score.
```

---

### 5. Definir score de `FusedLandmarkTrack`

Cada track fused debe tener score propio.

Score recomendado:

```text
fused_score =
    combinación de:
        scores de miembros
        número de miembros
        número de drones/submapas distintos
        número total de observaciones
        residual de fusión
        estabilidad del track
```

Primera política simple permitida:

```text
fused_score = clamp(
    media ponderada de scores de miembros
    + bonus por soporte multi-miembro
    + bonus por soporte multi-dron/submapa
    - penalización por residual alto
)
```

No inventar datos que no existan. Si no hay residual, documentar que no se usa.

Métodos sugeridos:

```text
UpdateFusedTrackScore(fused_id)
GetFusedTrackScore(fused_id)
```

Logs:

```text
[F1R-FUSED-SCORE-UPDATE]
```

---

### 6. Subir score por fusión confirmada

Cuando `FusionManager` confirme fusión:

```text
mp_a + mp_b → fused_track
```

debe emitir eventos:

```text
FusionConfirmed para cada miembro
FusedTrackMemberAdded para el track
FusedTrackConfirmed para el track
```

`FusionManager` no debe decidir el incremento numérico.

Logs:

```text
[F1R-SCORE-FUSION-CONFIRMED]
```

---

### 7. Bajar score de unmatched solo cuando sea razonable

No bajar score de todos los puntos no asociados.

Solo penalizar puntos que cumplan condiciones:

```text
la verificación geométrica fue buena;
el punto está dentro de zona de solape confirmada;
el punto debería ser visible/asociable;
tiene descriptor válido;
no está bad;
no fue descartado por falta de evidencia;
no es caso HOLD;
no viene de un loop rechazado;
opcionalmente ya falló varias veces.
```

Evento:

```text
ExpectedButUnmatchedInConfirmedOverlap
```

Logs:

```text
[F1R-SCORE-UNMATCHED-PENALIZE]
[F1R-SCORE-UNMATCHED-SKIP]
```

Ejemplo:

```text
[F1R-SCORE-UNMATCHED-SKIP] mp=432 reason=outside_confirmed_overlap
```

---

### 8. Score rollback / journal de eventos

Crear o consolidar un journal de eventos de score.

Campos recomendados:

```text
event_id
task_id
source
target_type = RAW_MP | FUSED_TRACK
target_id
event_type
old_score
new_score
committed
rolled_back
arrival_id
reason
```

Debe permitir:

```text
commit por task_id
rollback por task_id
```

Casos importantes:

- una optimización se aplica pero 1L la rechaza;
- una fusión asociada a una optimización aceptada luego debe revertirse por fallo posterior;
- una prueba debug fuerza rollback.

Métodos sugeridos:

```text
BeginScoreTransaction(task_id)
ApplyScoreEvent(..., task_id)
CommitScoreTransaction(task_id)
RollbackScoreTransaction(task_id)
```

Si no se implementan transacciones completas, debe existir al menos una forma de restaurar scores afectados por `task_id`.

Logs:

```text
[F1R-SCORE-JOURNAL-BEGIN]
[F1R-SCORE-JOURNAL-COMMIT]
[F1R-SCORE-JOURNAL-ROLLBACK]
```

---

### 9. Integrar rollback de score con rollback de optimización/fusión

Cuando 1L o 1Q hagan rollback:

```text
poses vuelven al estado anterior;
correcciones se restauran;
score events asociados al task deben revertirse;
fused tracks creados por ese task deben revertirse o marcarse no publicados.
```

Regla:

```text
Un loop rechazado no debe dejar scores altos ni tracks fusionados activos.
```

Logs:

```text
[F1R-SCORE-ROLLBACK-FROM-OPT]
[F1R-FUSED-TRACK-ROLLBACK]
```

---

### 10. Publicación por score en `GlobalMapBuilder`

`GlobalMapBuilder` debe usar los scores de forma coherente.

Reglas:

```text
1. Publicar FusedLandmarkTrack si existe y score >= umbral.
2. Publicar MapPoints raw no fusionados si score >= umbral.
3. No publicar miembros raw de tracks fused en la nube principal.
4. Permitir topic/debug de raw completo si está habilitado.
5. Incluir campo score en PointCloud2 si el formato lo permite.
```

Parámetros:

```text
global_map_min_score_to_publish
global_map_publish_fused_tracks
global_map_publish_raw_unfused
global_map_publish_raw_debug
global_map_publish_score_field
```

Logs:

```text
[F1R-GLOBALMAP-SCORE-FILTER]
[F1R-GLOBALMAP-FUSED-SCORE-PUBLISH]
[F1R-GLOBALMAP-RAW-SCORE-PUBLISH]
[F1R-GLOBALMAP-SKIP-LOW-SCORE]
```

---

### 11. Estadísticas de score

Añadir estadísticas para depuración:

```text
raw_mappoints_total
raw_mappoints_publishable
fused_tracks_total
fused_tracks_publishable
score_min
score_mean
score_max
score_histogram opcional
num_score_events
num_score_rollbacks
```

Logs:

```text
[F1R-SCORE-STATS]
```

Ejemplo:

```text
[F1R-SCORE-STATS] raw_total=5400 raw_pub=3910 fused_total=220 fused_pub=198 min=0.12 mean=0.58 max=0.97 events=845 rollbacks=2
```

---

### 12. Export debug opcional

Exportar snapshots de score:

```text
codex/archivos_auxiliares/f1r_score_snapshot_<run_id>.csv
codex/archivos_auxiliares/f1r_score_events_<run_id>.csv
codex/archivos_auxiliares/f1r_fused_scores_<run_id>.csv
```

Logs:

```text
[F1R-SCORE-DEBUG-EXPORT]
```

No es obligatorio si los logs son suficientes, pero es muy útil para depuración.

---


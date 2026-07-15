## Archivos prohibidos

No modificar:

- `ORB_SLAM3/`, salvo permiso explícito.
- `orbslam3_ros2` / wrapper, salvo bug crítico demostrado.
- `orbslam3_msgs`, salvo necesidad justificada y aprobada.
- `global_map_server_antiguo.cpp`, salvo consulta.
- archivos legacy de `orbslam3_multi/legacy/`, salvo consulta.
- `RawMapDatabase` para corregir o fusionar poses/puntos.
- clases nuevas que dupliquen el optimizador ya creado.
- un segundo `PoseGraphBuilder` específico de loops.
- un segundo `GlobalPoseStore`.
- fusión antes de optimización cuando el error del loop es alto.
- fusión después de rollback.
- uso de GT para loops.
- `build/`, `install/`, `log/` como código fuente.

No borrar código legacy de forma masiva.

---

## Funciones, clases o nodos que hay que localizar

`planificador_fase` debe localizar:

### En `orbslam3_multi`

- `LoopVerificationResult`
- `LoopDecisionManager`
- `LoopOptimizationTask`
- `CovisibilityDatabase`
- `PoseGraphBuilder`
- `PoseGraphProblem`
- `PoseGraphEdge`
- `PoseGraphPrior`
- `PropagationPlan`
- `OptimizationManager`
- `OptimizationDryRunResult`
- `OptimizationApplyResult`
- `PostApplyValidationResult`
- `GlobalPoseStore`
- `RawMapDatabase`
- `FusionManager`
- `FusedLandmarkManager`
- `GlobalMapBuilder`

Métodos o equivalentes:

- creación de `LoopOptimizationTask`;
- `PoseGraphBuilder::Build(...)` para task genérico;
- rama interna `BuildForLoopTask(...)` si ya existe;
- dry-run;
- apply;
- rollback;
- post-apply validation;
- fusion after accept;
- skip fusion after reject.
- consulta de aristas confirmadas para una ventana de KeyFrames;
- actualización de `relative_pose_current` tras optimización aceptada.

### En `orbslam3_server`

- punto donde `LoopDecisionManager` recibe `LOOP_OPTIMIZATION_CANDIDATE`;
- punto donde se envía la tarea a `PoseGraphBuilder`;
- punto donde se ejecuta dry-run/apply/post-apply;
- punto donde se llama a fusión después de ACCEPT;
- logs de servidor.

---

## Cambios requeridos

### 1. Confirmar o completar `LoopOptimizationTask`

Debe existir una tarea que represente un loop geométricamente confirmado con error alto.

Campos mínimos:

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
inlier_mappoint_pairs
created_from_loop_result_id
```

Condición:

```text
LoopOptimizationTask solo se crea si SubcloudLoopVerifier confirmó geometría
y el error de pose es alto.
```

Logs:

```text
[F1P-LOOP-OPT-TASK-RX]
```

---

### 2. Reutilizar `PoseGraphBuilder` para loop

No crear otro builder.

`PoseGraphBuilder` debe aceptar una `LoopOptimizationTask` y construir un `PoseGraphProblem`.

Diferencia con fiducial:

```text
FiducialOptimizationTask:
    prior absoluto fuerte sobre un KF o zona.

LoopOptimizationTask:
    edge/constraint relativa entre zona query y zona candidate.
```

El grafo para loop debe incluir:

- KFs alrededor de `query_kf_id`;
- KFs alrededor de `candidate_seed_kf_id`;
- fiduciales/hard anchors relevantes;
- edges internas en la zona query;
- edges internas en la zona candidate;
- soft priors;
- constraint principal de loop basado en `estimated_candidate_T_query`;
- aristas de covisibilidad confirmada desde `CovisibilityDatabase`;
- KFs variables/fijos;
- `PropagationPlan` para KFs afectados no variables.

Logs:

```text
[F1P-LOOP-GRAPH-BUILD]
[F1P-LOOP-GRAPH-SUMMARY]
[F1Q-COVIS-GRAPH-EDGES]
```

Debe incluir:

```text
vertices
query_vertices
candidate_vertices
fixed_vertices
variable_vertices
edges
loop_edges
covisibility_edges
priors
propagation_size
```

### 2.1. Añadir aristas desde `CovisibilityDatabase`

`PoseGraphBuilder` debe consultar la base confirmada al construir un grafo de
loop y, cuando aplique, también al reutilizar la ruta fiducial.

Consulta recomendada:

```text
covis_edges = covis_db.GetEdgesForWindow(selected_kfs, min_weight)
```

Reglas:

- no añadir vértices nuevos solo por una arista de covisibilidad salvo que la
  subfase lo justifique explícitamente;
- si ambos KFs ya son vértices del grafo, añadir una edge relativa;
- si un KF no es vértice, usar la relación como evidencia para propagación o
  para seleccionar controles en una futura ampliación;
- usar `relative_pose_current` como medición activa;
- conservar `relative_pose_measured` intacta;
- escalar el peso con `information_weight`, `source` y soporte.

Logs:

```text
[F1Q-COVIS-GRAPH-EDGES]
[F1Q-COVIS-GRAPH-EDGE]
[F1Q-COVIS-GRAPH-SKIP]
```

---

### 3. Constraint principal de loop

El constraint objetivo del loop debe representar:

```text
la relación query ↔ candidate estimada por RANSAC.
```

Ejemplo conceptual:

```text
T_candidate_query_measured = estimated_candidate_T_query
```

El residual debe penalizar que la relación actual entre ambas zonas difiera de esa medición.

No imponer que solo se mueva el query. El grafo debe permitir que cada lado se mueva según pesos y constraints:

```text
query side se mueve más si tiene menor confianza;
candidate/global side se mueve menos si está cerca de fiduciales o ya optimizada;
ambos pueden moverse si ninguno está fuertemente fijado.
```

Logs:

```text
[F1P-LOOP-CONSTRAINT]
```

---

### 4. Reutilizar `OptimizationManager` dry-run

No crear un dry-run especial.

Ejecutar:

```text
OptimizationManager::RunDryRun(PoseGraphProblem)
```

Para loop, el dry-run debe calcular:

```text
before_loop_error
predicted_after_loop_error
initial_cost
final_cost
moved_kfs
max_delta_t
hard_fixed_moved
```

Métricas específicas:

```text
before_error_t
predicted_after_error_t
before_error_yaw
predicted_after_error_yaw
mean_residual_before
mean_residual_predicted_after
```

Logs:

```text
[F1P-LOOP-DRYRUN-RESULT]
```

---

### 5. Aplicar solo si dry-run es útil

Reutilizar apply de 1K.

Condiciones mínimas:

```text
dryrun.success=true
dryrun.useful=true
hard_fixed_moved=false
final_cost < initial_cost
predicted_after_error < before_error
```

Aplicar en `GlobalPoseStore`:

- poses optimizadas;
- poses propagadas;
- correcciones por KF;
- última corrección heredable de submapa;
- KFs pendientes/rebasados si existen.

No modificar `RawMapDatabase`.

Logs:

```text
[F1P-LOOP-APPLY]
```

Debe aparecer también la familia de logs de 1K:

```text
[F1K-OPT-APPLY-PRECHECK]
[F1K-OPT-APPLY-SUMMARY]
```

---

### 6. Validar post-apply para loop

Reutilizar la validación de 1L, ampliándola para loops si falta.

Después del apply, recalcular el error real del loop usando el estado actualizado de `GlobalPoseStore`.

Proceso recomendado:

1. reconstruir o actualizar `query_subcloud` y `candidate_subcloud` con poses actuales;
2. usar los inliers de la verificación inicial si siguen siendo válidos o rehacer matching/RANSAC si es necesario;
3. calcular:
   - `real_after_error_t`;
   - `real_after_error_yaw`;
   - `real_after_error_rot`;
   - `mean_residual_after`;
   - `inlier_ratio_after`;
4. comparar con:
   - `before_error`;
   - `predicted_after_error`.

Logs:

```text
[F1P-LOOP-POST-APPLY-ERROR]
```

Debe aparecer también la familia de logs de 1L:

```text
[F1L-POST-APPLY-ERROR]
[F1L-POST-APPLY-INTERNAL-ERROR]
[F1L-POST-APPLY-FIXED-CHECK]
```

---

### 7. Aceptar, parcializar o rechazar

Usar las decisiones de 1L:

```text
ACCEPT
PARTIAL
REJECT
```

#### ACCEPT

Si el error real de loop baja y no se rompe coherencia:

```text
mantener poses;
mantener correcciones;
actualizar `relative_pose_current` de las aristas usadas en CovisibilityDatabase;
permitir fusión posterior.
```

Logs:

```text
[F1P-LOOP-POST-APPLY-ACCEPT]
[F1Q-COVIS-EDGE-REFINED]
```

#### PARTIAL

Si mejora pero sigue alto:

```text
marcar needs_second_pass;
no fusionar por defecto;
guardar retry suggestion.
```

Logs:

```text
[F1P-LOOP-POST-APPLY-PARTIAL]
```

#### REJECT

Si no mejora, empeora, rompe edges internas o mueve hard-fixed:

```text
rollback;
no fusionar;
marcar loop task fallido;
diagnóstico y sugerencia de reintento.
```

Logs:

```text
[F1P-LOOP-POST-APPLY-REJECT]
[F1P-LOOP-ROLLBACK]
[F1P-LOOP-FUSION-SKIP]
```

---

### 8. Fusionar solo después de ACCEPT

Si la optimización por loop queda aceptada por 1L/1Q:

```text
FusionManager fusiona landmarks usando los inlier_mappoint_pairs
del LoopVerificationResult/LoopOptimizationTask.
```

Antes de fusionar, debe comprobar:

- la optimización fue aceptada;
- no hubo rollback;
- los inliers siguen siendo geométricamente válidos;
- los MapPoints existen;
- no están bad;
- no se duplican tracks fused existentes.

Logs:

```text
[F1P-LOOP-FUSION-AFTER-ACCEPT]
```

Si la optimización fue parcial o rechazada:

```text
no fusionar
```

Logs:

```text
[F1P-LOOP-FUSION-SKIP]
```

---

### 9. Publicar mapa fused actualizado

Después de una optimización de loop aceptada y fusión aplicada:

- `GlobalMapBuilder` debe publicar la nube usando:
  - poses actualizadas de `GlobalPoseStore`;
  - tracks fused de `FusedLandmarkManager`;
  - scores actualizados de `LandmarkScoreManager`.

Logs:

```text
[F1P-LOOP-GLOBALMAP-PUBLISH]
```

RViz2 debería mostrar:

```text
reducción de doble pared;
zonas query/candidate más alineadas;
puntos duplicados reducidos por fusión;
fiduciales/hard-fixed sin movimiento indebido.
```

---

### 10. Diagnóstico y retry en caso de fallo

Si se rechaza la optimización por loop, reutilizar política de 1L.

Sugerencias posibles:

```text
WIDER_WINDOW
STRONGER_INTERNAL_EDGES
WEAKER_LOOP_OBJECTIVE
STRONGER_FIDUCIAL_PRIORS
DIFFERENT_VERTEX_SELECTION
RECOMPUTE_LOOP_SUBCLOUDS
REJECT_CANDIDATE
```

Reglas:

```text
si error no mejora:
    WIDER_WINDOW o DIFFERENT_VERTEX_SELECTION

si mejora pero rompe edges internas:
    STRONGER_INTERNAL_EDGES o WEAKER_LOOP_OBJECTIVE

si hard-fixed se mueve:
    BUG_FIXED_SELECTION

si loop parece falso:
    RECOMPUTE_LOOP_SUBCLOUDS o REJECT_CANDIDATE
```

No reintentar automáticamente en bucle en esta subfase.

Logs:

```text
[F1P-LOOP-ROLLBACK-DIAGNOSTIC]
[F1P-LOOP-RETRY-SUGGESTION]
```

---

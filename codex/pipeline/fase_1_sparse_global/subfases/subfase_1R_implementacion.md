## Archivos prohibidos

No modificar:

- `ORB_SLAM3/`, salvo permiso explícito.
- `orbslam3_ros2` / wrapper, salvo bug crítico demostrado.
- `orbslam3_msgs`, salvo necesidad justificada y aprobada.
- `global_map_server_antiguo.cpp`, salvo consulta.
- archivos legacy de `orbslam3_multi/legacy/`, salvo consulta.
- `RawMapDatabase` para corregir poses.
- `GlobalPoseStore` para guardar datos raw.
- crear un segundo pipeline de loops.
- crear un segundo pipeline de optimización.
- usar GT para loops.
- fusionar u optimizar sin pasar por 1N–1Q.
- reintentar loops en cascada infinita.
- `build/`, `install/`, `log/` como código fuente.

No borrar código legacy de forma masiva.

---

## Funciones, clases o nodos que hay que localizar

`planificador_fase` debe localizar:

### En `orbslam3_multi`

- `RawMapDatabase`
- `GlobalPoseStore`
- estados de pose/KF usados en 1K:
  - `PENDING_DURING_OPTIMIZATION`
  - `REBASED_AFTER_SERVER_OPT`
  - nombres equivalentes si ya existen
- `OptimizationManager`
- `OptimizationApplyResult`
- `PostApplyValidationResult`
- `LoopDetector`
- `SubcloudLoopVerifier`
- `LoopDecisionManager`
- `FusionManager`
- `FusedLandmarkManager`
- `LoopOptimizationTask`
- estructura de cola si existe

### En `orbslam3_server`

- punto donde llegan deltas durante optimización;
- punto donde se marca un KF como pending;
- punto donde termina una optimización;
- punto donde 1L devuelve ACCEPT/PARTIAL/REJECT;
- punto donde se invoca `LoopDetector` para KFs normales;
- control de timers/replay;
- parámetros debug de cascada.

---

## Cambios requeridos

### 1. Registrar KFs que llegan durante una optimización

Cuando llega un delta mientras hay una optimización activa, los KFs nuevos deben:

1. insertarse en `RawMapDatabase`;
2. recibir pose provisional en `GlobalPoseStore` si el submapa está anclado;
3. marcarse con estado pendiente;
4. asociarse al `optimization_id` activo.

Estado recomendado:

```text
PENDING_DURING_OPTIMIZATION
```

Campos recomendados por KF pendiente:

```text
kf_id
submap_id
arrival_id
optimization_id
pose_status_before_rebase
provisional_world_pose
reason=arrived_during_optimization
```

Logs:

```text
[F1Q-KF-ARRIVED-DURING-OPT]
[F1Q-KF-MARK-PENDING]
```

Ejemplo:

```text
[F1Q-KF-MARK-PENDING] kf=143 submap=(1,0) optimization_id=7 arrival_id=381 reason=arrived_during_optimization
```

---

### 2. Rebasar KFs pendientes tras optimización aceptada

Cuando una optimización termina con `ACCEPT` en 1L, los KFs pendientes de los submapas afectados deben rebasarse usando la corrección heredable creada por 1K.

Regla:

```text
world_T_kf_base = world_T_local * local_T_kf
world_T_kf = correction_T_inherited * world_T_kf_base
```

La corrección heredada puede venir de:

```text
submap_last_server_correction
corrección del parent corregido
corrección del KF covisible corregido más cercano
```

En 1R basta con usar la política simple ya validada en 1K, normalmente `submap_last_server_correction`.

Estado tras rebase:

```text
REBASED_AFTER_OPTIMIZATION
```

Logs:

```text
[F1Q-OPT-FINISHED]
[F1Q-KF-REBASED]
```

Ejemplo:

```text
[F1Q-KF-REBASED] kf=143 submap=(1,0) optimization_id=7 inherited_correction=true delta_t=0.42
```

---

### 3. Encolar KFs rebasados para loop check

Después de rebasar un KF, debe añadirse a una cola post-optimización:

```text
post_optimization_loop_check_queue
```

Estado recomendado:

```text
QUEUED_FOR_LOOP_CHECK
```

Cada entrada debe contener:

```text
kf_id
submap_id
optimization_id
reason=rebased_after_optimization
queued_at_time
queued_at_arrival_id
retry_count
```

Logs:

```text
[F1Q-KF-QUEUE-LOOP-CHECK]
```

Ejemplo:

```text
[F1Q-KF-QUEUE-LOOP-CHECK] kf=143 submap=(1,0) reason=rebased_after_optimization optimization_id=7
```

---

### 4. Despachar cola hacia el pipeline 1N–1P

La cola debe procesarse llamando al pipeline normal de loops:

```text
LoopDetector
    ↓
SubcloudLoopVerifier
    ↓
LoopDecisionManager
```

Flujo recomendado:

```text
for each queued_kf:
    LoopDetector.ProcessNewKeyFrame(kf_id)
    if candidates:
        SubcloudLoopVerifier.VerifyCandidate(...)
        LoopDecisionManager.ProcessVerificationResult(...)
```

Si `LoopDecisionManager` produce `LoopOptimizationTask`, se envía al pipeline 1Q.

Estados finales:

```text
LOOP_CHECK_DONE
LOOP_CHECK_SKIPPED
LOOP_CHECK_CREATED_NEW_TASK
```

Logs:

```text
[F1Q-LOOP-CHECK-DISPATCH]
[F1Q-LOOP-CHECK-RESULT]
```

Ejemplo:

```text
[F1Q-LOOP-CHECK-DISPATCH] kf=143 route=1M_1N_1O source=post_optimization_queue
```

---

### 5. Evitar reprocesado duplicado

Un KF no debe procesarse infinitamente por la misma causa.

Guardar registro:

```text
kf_id
optimization_id
processed_after_rebase=true
```

Si vuelve a aparecer la misma combinación:

```text
skip reason=already_processed_for_optimization
```

Logs:

```text
[F1Q-LOOP-CHECK-SKIP]
```

---

### 6. Controlar cascadas de optimización

El reprocesado post-optimización puede producir nuevos loops y nuevas optimizaciones. Esto es correcto, pero debe controlarse.

Parámetros recomendados:

```text
post_opt_max_kfs_per_cycle
post_opt_max_loop_checks_per_cycle
post_opt_max_new_optimization_tasks_per_cycle
post_opt_max_cascade_depth
post_opt_cooldown_sec
post_opt_candidate_pair_cooldown
```

Si se alcanza un límite:

```text
no procesar más en este ciclo;
dejar cola pendiente o marcar como skipped temporal;
emitir log.
```

Logs:

```text
[F1Q-CASCADE-LIMIT]
[F1Q-QUEUE-PAUSED]
```

Ejemplo:

```text
[F1Q-CASCADE-LIMIT] optimization_id=7 cascade_depth=2 max=2 action=queue_paused
```

---

### 7. Comportamiento si la optimización se rechaza o hace rollback

Si 1L devuelve `REJECT` y se ejecuta rollback:

```text
no rebasar KFs pendientes con una corrección rechazada;
no encolar KFs por esa optimización aceptada;
restaurar o mantener pose coherente anterior según GlobalPoseStore;
marcar KFs como LOOP_CHECK_SKIPPED o PENDING_AFTER_ROLLBACK según política.
```

Decisión recomendada en 1R:

```text
si rollback:
    no reinyectar KFs por esa optimización;
    dejarlos para el flujo normal cuando llegue nuevo delta/snapshot,
    o encolar con reason=after_rollback_recheck solo en modo debug.
```

Logs:

```text
[F1Q-OPT-REJECTED-NO-REBASE]
[F1Q-LOOP-CHECK-SKIP]
```

---

### 8. Comportamiento si la optimización queda PARTIAL

Si 1L devuelve `PARTIAL`, actuar de forma conservadora.

Política recomendada:

```text
si partial_accept mantiene poses:
    rebasar KFs pendientes;
    encolar loop check con prioridad baja o cooldown;

si partial no confirma suficientemente:
    no disparar nuevas optimizaciones inmediatamente;
    marcar needs_second_pass.
```

Logs:

```text
[F1Q-OPT-PARTIAL-DEFER-LOOP-CHECK]
```

---

### 9. Integración con replay

El comportamiento debe funcionar en replay.

Al reproducir deltas:

1. si hay optimización activa, KFs nuevos van a pending;
2. cuando el evento de optimización termina con ACCEPT, se rebasan;
3. se encolan;
4. se procesan por 1N–1P con el mismo orden determinista.

Logs:

```text
[F1Q-REPLAY-PENDING-KF]
[F1Q-REPLAY-QUEUE-DISPATCH]
```

---

### 10. Export debug opcional

Se recomienda exportar un resumen de la cola:

```text
codex/archivos_auxiliares/f1q_post_opt_queue_<optimization_id>.csv
```

Campos:

```text
optimization_id
kf_id
submap_id
arrival_id
status
queued
dispatched
loop_candidates
verification_decision
loop_task_created
```

Logs:

```text
[F1Q-DEBUG-EXPORT]
```

---


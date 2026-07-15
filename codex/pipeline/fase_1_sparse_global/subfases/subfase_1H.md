# Subfase 1H — Revisit fiducial y creación/coalescencia de tareas de optimización

## Estado

```text
actual
```

Estado anterior: `realizado`.

Esta subfase se reabre de forma limitada porque la detección del error fiducial ya funciona, pero la gestión actual de tareas usa `arrival_id` dentro de la clave de deduplicación y conserva las tareas indefinidamente. Eso provoca varias optimizaciones casi idénticas para una misma llegada al fiducial y dificulta distinguir una tarea pendiente, ejecutándose, sustituida o terminada.

La reapertura no cambia la semántica de los fiduciales ni introduce optimización. Su objetivo es entregar a `1I` una tarea única, vigente y trazable.

---

## Objetivo técnico

Cuando un submapa ya anclado observa de nuevo un fiducial:

1. obtener la pose global actual del `KeyFrame` asociado;
2. compararla con la pose absoluta de la observación fiducial;
3. confirmar la observación si el residual está dentro de umbral;
4. crear o actualizar una única tarea lógica de optimización si el residual es alto;
5. evitar tareas duplicadas durante una misma visita;
6. proporcionar un ciclo de vida explícito para que el servidor pueda consumir, sustituir y cerrar tareas;
7. volver a medir el residual justo antes de construir el grafo para descartar tareas obsoletas.

Esta subfase no construye el grafo, no optimiza, no aplica poses y no publica correcciones.

La segunda observación de un fiducial sigue siendo una observación absoluta. No es un loop.

---

## Decisiones de diseño obligatorias

### 1. No usar `arrival_id` como identidad lógica de la tarea

`arrival_id` identifica una llegada al journal, no un problema geométrico distinto. Dos observaciones consecutivas del mismo fiducial durante la misma visita no deben crear dos optimizaciones.

La agrupación lógica mínima debe ser equivalente a:

```text
TaskGroupKey = (submap_id, fiducial_id)
```

Mientras exista una tarea `PENDING` o `RUNNING` para esa clave:

- una observación más reciente actualiza o sustituye la candidata pendiente;
- no se crea una tarea paralela idéntica;
- una tarea ya `RUNNING` mantiene su snapshot inmutable;
- si llega evidencia nueva durante una tarea `RUNNING`, se guarda como sucesora pendiente solo si el residual sigue siendo alto al terminar la tarea actual.

Después de un `ACCEPT`, una nueva visita futura al mismo fiducial sí puede abrir una nueva generación.

### 2. Separar identidad, generación y procedencia

Cada tarea debe conservar:

```text
task_id                 # único y monotónico
task_generation         # generación dentro del TaskGroupKey
task_group_key          # (submap_id, fiducial_id)
latest_arrival_id       # procedencia más reciente
created_arrival_id      # primera observación que abrió la generación
source                  # LIVE / REPLAY_RECORDED_FIDUCIAL / equivalente
```

`arrival_id` no debe intervenir en la comparación de igualdad de la tarea.

### 3. Ciclo de vida explícito

Estados mínimos:

```text
PENDING
RUNNING
SUPERSEDED
COMPLETED_ACCEPTED
COMPLETED_REJECTED
CANCELLED_STALE
```

No es obligatorio exponer el enum fuera de `orbslam3_multi`, pero la semántica debe existir.

### 4. La tarea no congela ciegamente el residual

La tarea guarda el residual observado al crearse, pero antes de `1I` el servidor debe volver a consultar `GlobalPoseStore` y recalcular:

```text
current_estimated_world_T_kf
current_error_t
current_error_rot
current_error_yaw
```

Si el error ya está por debajo de umbral, la tarea se cierra como `CANCELLED_STALE` o equivalente y no se construye grafo.

### 5. El target fiducial no se marca hard antes de ser validado

El `KeyFrame` objetivo solo puede pasar a hard fiducial cuando:

- la observación ya era coherente sin optimización; o
- una optimización posterior fue aceptada y comprometida.

Una tarea pendiente no convierte el target en fijo.

---

## Contexto obligatorio a leer

Antes de modificar código:

- `AGENTS.md`
- `codex/contexto/01_ESTADO_ACTUAL.md`
- `codex/pipeline/PIPELINE_MAESTRO.md`
- `codex/pipeline/fase_1_sparse_global/pipeline_fase_1.md`
- `subfase_1E.md`
- `subfase_1F.md`
- `subfase_1G.md`
- esta subfase y las versiones revisadas de `1I` a `1L`
- historial reciente de Fase 1
- documentación de:
  - `orbslam3_multi/fiducial_anchor_manager`
  - `orbslam3_multi/global_pose_store`
  - `orbslam3_multi/raw_map_database`
  - `orbslam3_server/global_map_server`

---

## Diagnóstico de partida

La implementación actual ya:

- distingue primer anclaje y revisit;
- mide errores de traslación, rotación y yaw;
- crea `FiducialOptimizationTask`;
- no sobrescribe el anchor rígido del submapa durante un revisit;
- marca hard fiducials solo en primera observación o revisit ya coherente.

Problemas que debe corregir esta revisión:

1. `TaskKey` contiene `arrival_id`, por lo que observaciones consecutivas generan tareas distintas.
2. `pending_tasks_` no tiene consumo ni retirada explícita.
3. `task_ids_by_key_` crece indefinidamente.
4. El servidor puede procesar varias tareas equivalentes de una misma visita.
5. No existe una distinción clara entre tarea vigente y tarea obsoleta.
6. No se vuelve a medir el residual justo antes de construir el grafo.

---

## Archivos permitidos a modificar

### `orbslam3_multi`

- `include/orbslam3_multi/fiducial_optimization_task.hpp`
- `include/orbslam3_multi/fiducial_anchor_manager.hpp`
- `src/fiducial_anchor_manager.cpp`
- `include/orbslam3_multi/global_pose_store.hpp`, solo si se necesita una consulta/revisión de estado
- `src/global_pose_store.cpp`, solo si se necesita la consulta anterior
- `CMakeLists.txt` para tests

### `orbslam3_server`

- `src/global_map_server.cpp`
- `launch/global_orb_map_server.launch.py`, solo para parámetros reales

### Tests y documentación

- `orbslam3_multi/test/`
- `codex/archivos_auxiliares/trayectorias/tray_prueba_1.yaml`
- `codex/contexto/01_ESTADO_ACTUAL.md`
- documentación de paquetes afectados
- historial de Fase 1

---

## Archivos prohibidos

No modificar:

- `ORB_SLAM3/`;
- `orbslam3_ros2`;
- `orbslam3_msgs`;
- código legacy;
- lógica de solver de `1J`;
- aplicación/rollback de `1K-1L`.

---

## Cambios requeridos

### 1. Extender `FiducialOptimizationTask`

Campos mínimos nuevos o equivalentes:

```text
task_generation
created_arrival_id
latest_arrival_id
status
update_count
created_source
latest_source
```

Debe seguir conservando:

```text
submap_id
keyframe_id objetivo
fiducial_id
target_world_T_kf
estimated_world_T_kf al crear/actualizar
error_t_m
error_rot_rad
error_yaw_rad
umbrales
```

No añadir todavía información de covisibilidad ni vértices. Eso pertenece a `1I`.

### 2. Sustituir la deduplicación actual

Eliminar la identidad lógica basada en:

```text
(keyframe_id, fiducial_id, arrival_id, source)
```

Usar una tabla por `TaskGroupKey` y generación.

Comportamiento recomendado:

```text
si no hay tarea activa para (submap,fid):
    crear generación nueva PENDING

si hay PENDING:
    conservar task_id
    actualizar target KF, pose objetivo, residual y latest_arrival_id
    incrementar update_count

si hay RUNNING:
    no mutar la tarea en ejecución
    guardar latest_observation para una posible generación sucesora

si la tarea terminó ACCEPTED:
    permitir una generación nueva solo ante una visita posterior con residual alto
```

### 3. Añadir API de consumo y cierre

API orientativa, adaptada a los nombres reales del código:

```cpp
std::optional<FiducialOptimizationTask> TakeNextPendingTask();
bool MarkTaskRunning(uint64_t task_id);
bool MarkTaskCompleted(uint64_t task_id, TaskCompletion completion);
bool CancelTaskAsStale(uint64_t task_id, const std::string& reason);
```

No devolver continuamente una copia de todas las tareas para que el servidor las recorra en cada callback.

### 4. Revalidar residual antes de entregar a `1I`

El servidor, al tomar una tarea:

1. consulta la pose global actual del target;
2. recalcula el residual contra `target_world_T_kf`;
3. actualiza las métricas de ejecución de la tarea;
4. cancela si el residual ya es bajo;
5. solo entonces llama a `PoseGraphBuilder`.

Esto evita optimizar un error ya corregido por otra tarea o snapshot.

### 5. Limitar memoria histórica

Las tareas completadas pueden conservarse en un historial circular pequeño para diagnóstico, pero no en la cola activa.

Parámetros orientativos:

```text
fiducial_task_completed_history_limit
fiducial_task_max_pending_per_submap
```

El límite por submapa debe ser pequeño; el comportamiento normal esperado es una sola tarea activa por submapa.

### 6. Mantener replay determinista

Durante replay:

- las mismas observaciones deben producir la misma secuencia lógica de generaciones;
- el `arrival_id` se conserva como trazabilidad;
- no debe volver a convertirse en clave de igualdad;
- si varias observaciones del replay pertenecen a una misma visita, deben coalescerse igual que en live.

---

## Marcadores de log obligatorios

Mantener:

```text
[F1H-FID-REVISIT]
[F1H-FID-POSE-ERROR]
[F1H-FID-OK]
```

Sustituir o ampliar creación/deduplicación con:

```text
[F1H-TASK-CREATED] task_id=<id> generation=<g> group=(d,e,fid) target_kf=<id>
[F1H-TASK-UPDATED] task_id=<id> update_count=<n> old_kf=<id> new_kf=<id>
[F1H-TASK-TAKEN] task_id=<id> status=RUNNING
[F1H-TASK-STALE] task_id=<id> current_error_t=<m> reason=<...>
[F1H-TASK-COMPLETED] task_id=<id> result=<ACCEPTED|REJECTED|CANCELLED>
[F1H-TASK-SUCCESSOR-PENDING] parent_task_id=<id> successor_task_id=<id>
[F1H-TASK-STATS] pending=<n> running=<n> completed_history=<n> coalesced=<n>
```

No imprimir una línea `WARN` por cada actualización normal. Usar `INFO` o `DEBUG`.

---

## Tests unitarios obligatorios

Crear tests deterministas para `FiducialAnchorManager`:

1. Primera observación crea anchor y no crea tarea.
2. Revisit dentro de umbral marca fiducial coherente y no crea tarea.
3. Dos observaciones altas consecutivas del mismo `(submap,fid)` producen un solo `task_id` pendiente.
4. La segunda observación actualiza el target KF y `latest_arrival_id`.
5. Una observación durante `RUNNING` no muta el snapshot en ejecución.
6. Al completar una tarea, una visita futura puede crear una nueva generación.
7. Una tarea cuyo error actual ya bajó se cancela como stale.
8. Replay produce el mismo agrupamiento que live.
9. Las colas activas no crecen con tareas completadas.

---

## Pruebas de integración

### Prueba 1 — visita prolongada al mismo fiducial

Mantener al dron dentro del radio suficiente tiempo para recibir varias asociaciones KF-fiducial.

Éxito:

```text
muchas observaciones fiduciales
una sola tarea lógica PENDING/RUNNING para el grupo
cero optimizaciones paralelas equivalentes
```

### Prueba 2 — replay

Reproducir un `.record` que contenga varias observaciones de la misma visita.

Éxito:

```text
misma generación y mismo número de tareas lógicas que en live
```

---

## Paquetes a compilar

En llamadas separadas:

```bash
./codex/herramientas/build_selected_packages.sh orbslam3_multi
./codex/herramientas/build_selected_packages.sh orbslam3_server
```

No recompilar `orbslam3_msgs` si no se modifica.

---

## Criterio de éxito

La subfase queda `CONSEGUIDA` si:

- la detección de revisit sigue funcionando;
- el primer anchor no cambia;
- una visita prolongada no produce tareas duplicadas;
- existe ciclo de vida consumible;
- las tareas stale se cancelan antes de construir grafo;
- live y replay se comportan de forma equivalente;
- no se modifica ninguna pose por esta subfase;
- los tests unitarios y de integración pasan.

---

## Criterio de fallo o parcial

`PARCIAL` si la medición funciona, pero:

- siguen apareciendo varias tareas equivalentes;
- la cola activa crece sin límite;
- el servidor sigue recorriendo copias completas de tareas;
- no se puede cerrar una tarea;
- replay y live divergen.

`NO CONSEGUIDA` si se sobrescribe un anchor existente, se marca hard el target antes de validar o se aplica cualquier corrección desde `1H`.

---

## Documentación a actualizar

- `codex/contexto/01_ESTADO_ACTUAL.md`
- `codex/contexto/paquetes/orbslam3_multi/fiducial_anchor_manager.md`
- `codex/contexto/paquetes/orbslam3_multi/orbslam3_multi.md`
- `codex/contexto/paquetes/orbslam3_server/global_map_server.md`
- historial de Fase 1

La evidencia histórica extensa no debe volver a incrustarse al principio de esta subfase. Debe quedar en `historial_fase_1.md`.

---

## Entrega a la subfase 1I

`1H` entrega una única `FiducialOptimizationTask` vigente con:

```text
target fiducial absoluto
KF objetivo actual
residual actual validado
submapa y epoch correctos
generación y procedencia trazables
```

`1I` será responsable de localizar el fiducial previo, construir la ventana, consultar covisibilidad y seleccionar controles.
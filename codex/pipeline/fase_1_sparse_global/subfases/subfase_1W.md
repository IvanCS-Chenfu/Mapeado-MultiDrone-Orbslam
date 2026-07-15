# Subfase 1W — Rendimiento, límites y robustez del servidor sparse global

## Estado

```text
sin hacer
```

---

## Objetivo tecnico

Medir y ajustar límites operativos de la Fase 1 para que el servidor sparse global sea robusto con varios drones, muchos KeyFrames, subnubes grandes, full snapshots, optimizaciones y colas post-optimización.

Esta subfase no busca mejorar precisión SLAM. Busca garantizar:

```text
tiempos razonables
memoria acotada
colas controladas
debug pesado desactivable
degradación segura al alcanzar límites
```

---

## Contexto obligatorio a leer

Leer:

- `AGENTS.md`
- `codex/contexto/01_ESTADO_ACTUAL.md`
- pipeline completo 1A–1V
- documentación de paquetes
- logs de pruebas integrales 1V

---

## Diagnostico de partida

La fase 1 ya puede ejecutar muchas operaciones costosas:

```text
BoW amplio
subcloud candidate window
matching ORB
RANSAC
pose graph build
dry-run
apply
post-apply validation
fusión
score journal
debug markers
replay
```

Si no hay límites y métricas, el sistema puede degradarse con:

- demasiados candidatos BoW;
- demasiados puntos candidate;
- RANSAC lento;
- grafo demasiado grande;
- demasiados KFs pendientes;
- cascadas de optimizaciones;
- score journal sin límite;
- RViz markers muy pesados.

---

## Archivos permitidos a modificar

- parámetros/config del servidor y `orbslam3_multi`;
- logging de métricas;
- límites de cola;
- scripts de benchmark;
- documentación;
- cambios mínimos para cortar/degradar operaciones al alcanzar límites.

No modificar algoritmos de precisión salvo ajuste de límites.

---

## Archivos prohibidos

- `ORB_SLAM3/`
- wrapper salvo bug crítico
- rediseño de optimizador
- rediseño de fusión
- eliminación masiva de legacy
- cambios que oculten errores

---

## Funciones, clases o nodos que hay que localizar

- `RawMapDatabase`
- `LoopDetector`
- `SubcloudLoopVerifier`
- `PoseGraphBuilder`
- `OptimizationManager`
- `FusionManager`
- `LandmarkScoreManager`
- `GlobalMapBuilder`
- `PostOptimizationKeyFrameQueue`
- `global_map_server`

---

## Cambios requeridos

### 1. Centralizar parámetros de límites

Documentar y validar parámetros:

```text
rawdb_max_keyframes_debug
loop_bow_max_candidates
loop_bow_max_candidates_per_submap
candidate_window_max_kfs
candidate_subcloud_max_points
ransac_max_iterations
pose_graph_max_vertices
pose_graph_max_edges
propagation_max_kfs
post_opt_max_kfs_per_cycle
post_opt_max_cascade_depth
score_journal_max_events
debug_max_markers
publish_max_points_debug
```

Logs:

```text
[F1V-LIMITS-CONFIG]
```

---

### 2. Medir tiempos por etapa

Emitir métricas:

```text
delta_ingest_ms
full_snapshot_ms
bow_query_ms
subcloud_build_ms
matching_ms
ransac_ms
graph_build_ms
dryrun_ms
apply_ms
post_apply_ms
fusion_ms
score_update_ms
publish_ms
```

Logs:

```text
[F1V-PERF-STATS]
```

---

### 3. Medir tamaños de datos

Emitir:

```text
raw_keyframes
raw_mappoints
global_poses
fused_tracks
score_events
pending_kfs
loop_candidates
subcloud_points
graph_vertices
graph_edges
published_points
```

Logs:

```text
[F1V-SIZE-STATS]
```

---

### 4. Medir colas y cascadas

Para colas:

```text
pending_deltas
pending_post_opt_kfs
optimization_tasks
score_events_pending
```

Logs:

```text
[F1V-QUEUE-STATS]
[F1V-CASCADE-STATS]
```

---

### 5. Degradación segura al alcanzar límites

Si se alcanza un límite:

```text
reducir candidatos
usar top-K
hacer fallback
marcar HOLD
pausar cola
desactivar debug pesado
```

Nunca crashear ni aplicar resultados inseguros.

Logs:

```text
[F1V-LIMIT-HIT]
[F1V-SAFE-DEGRADE]
```

---

### 6. Prueba de estrés moderado

Ejecutar trayectorias largas/replay para simular muchos KFs.

---

## Cambios prohibidos

- No aumentar límites sin justificar.
- No ignorar límites alcanzados.
- No aplicar optimizaciones incompletas por timeout sin marcar.
- No bloquear el servidor por debug RViz.
- No ocultar out-of-memory/crashes.

---

## Paquetes a compilar

```bash
./codex/herramientas/build_selected_packages.sh orbslam3_multi orbslam3_server simulacion_dron
```

---

## Pruebas Gazebo requeridas

### Prueba 1 — rendimiento trayectoria larga

YAML:

```text
codex/archivos_auxiliares/trayectorias/tray_prueba_larga.yaml
```

Secuencia:

1. dos drones recorren edificio;
2. generar muchos KFs;
3. activar loops/fusión si aparecen;
4. medir tiempos y tamaños.

---

## Pruebas replay requeridas

### Prueba 2 — benchmark replay rápido

Usar dataset grande guardado.

### Prueba 3 — límites forzados

Ejecutar con límites bajos para comprobar `LIMIT-HIT` y `SAFE-DEGRADE`.

---

## Patrones de reduccion de logs

```text
SCENARIO-RUNNER|GOAL|RESULT|success|F1V-|PERF|LIMIT|QUEUE|SIZE|ERROR|FATAL|Segmentation fault|Killed
```

---

## Marcadores tecnicos obligatorios

```text
[F1V-LIMITS-CONFIG]
[F1V-PERF-STATS]
[F1V-SIZE-STATS]
[F1V-QUEUE-STATS]
```

Si se fuerzan límites:

```text
[F1V-LIMIT-HIT]
[F1V-SAFE-DEGRADE]
```

---

## Criterio de exito

`CONSEGUIDA` si:

1. compila;
2. métricas de tiempo/tamaño aparecen;
3. límites están documentados;
4. al forzar límites no hay crash;
5. colas no crecen sin control;
6. debug pesado es desactivable;
7. documentación actualizada.

---

## Criterio de fallo o parcial

`NO CONSEGUIDA` si:

- no compila;
- crashea por límites;
- no hay métricas;
- colas crecen sin límite;
- debug bloquea simulación.

`PARCIAL` si:

- métricas existen, pero falta prueba larga;
- límites funcionan en replay pero no en live.

---

## Documentacion a actualizar

Actualizar:

- `codex/contexto/paquetes/orbslam3_server/performance.md`
- `codex/contexto/paquetes/orbslam3_multi/performance_limits.md`
- historial 1W
- tabla de límites recomendados

# Subfase 1U — Observabilidad final en RViz2, topics debug y trazabilidad visual

## Estado

```text
sin hacer
```

---

## Objetivo tecnico

Consolidar la observabilidad visual y de debug del sistema sparse global multi-dron.

La Fase 1 ya debe tener ingesta, poses globales, fiduciales, loops, fusión, optimización, rollback y scoring. Esta subfase no debe cambiar la lógica de SLAM. Su objetivo es que el usuario pueda inspeccionar visualmente en RViz2 y por logs:

- qué KeyFrames existen;
- cuáles están raw/anclados/optimizados/propagados/rebasados;
- dónde están los fiduciales;
- qué candidatos BoW se detectaron;
- qué subnubes se compararon;
- qué grafo se optimizó;
- qué KFs se movieron;
- qué landmarks están fusionados;
- qué puntos se publican por score;
- qué rollback ocurrió.

---

## Contexto obligatorio a leer

Leer:

- `AGENTS.md`
- `codex/contexto/01_ESTADO_ACTUAL.md`
- `codex/pipeline/PIPELINE_MAESTRO.md`
- `codex/pipeline/fase_1_sparse_global/pipeline_fase_1.md`
- subfases `1A` a `1T`
- documentación de:
  - `orbslam3_multi`
  - `orbslam3_server`
  - `simulacion_dron`
- layouts RViz2 existentes si los hay.

---

## Diagnostico de partida

Las subfases anteriores usan muchos logs y algunos exports SVG/CSV. Sin una vista RViz2 coherente, el usuario puede no saber si:

```text
una optimización corrige o deforma;
una fusión elimina duplicados;
un loop candidate está en una zona razonable;
un fiducial se mantiene fijo;
un score bajo elimina ruido o elimina puntos buenos.
```

La subfase 1U debe crear o consolidar topics/markers de debug sin modificar la lógica principal.

---

## Archivos permitidos a modificar

### `orbslam3_multi`

- clases de export/debug visual si existen;
- `global_map_builder`;
- utilidades de MarkerArray;
- estructuras de debug no invasivas.

### `orbslam3_server`

- `orbslam3_server/src/global_map_server.cpp`
- publishers debug;
- parámetros debug;
- launch/config del servidor nuevo.

### `simulacion_dron`

Solo RViz/launch si procede:

- archivos `.rviz`;
- launch nuevo o argumentos para activar debug.

### Documentación

- `codex/contexto/paquetes/orbslam3_server/debug_topics.md`
- `codex/contexto/paquetes/orbslam3_multi/global_map_builder.md`
- `codex/contexto/paquetes/simulacion_dron/rviz.md`
- historial 1U

---

## Archivos prohibidos

No modificar:

- `ORB_SLAM3/`
- wrapper salvo necesidad justificada
- mensajes salvo decisión explícita
- lógica de optimización/fusión/scoring
- datos raw para “hacer bonito” en RViz2

---

## Funciones, clases o nodos que hay que localizar

- `global_map_server`
- publishers de nube global existentes
- `GlobalMapBuilder`
- `GlobalPoseStore`
- `FusedLandmarkManager`
- `LandmarkScoreManager`
- `LoopDetector`
- `SubcloudLoopVerifier`
- `PoseGraphBuilder`
- `OptimizationDebugExporter`
- layouts RViz2 existentes

---

## Cambios requeridos

### 1. Crear topics de nube global diferenciados

Publicar, si está habilitado:

```text
/global_sparse_cloud_fused
/global_sparse_cloud_scored
/global_sparse_cloud_raw_debug
```

Reglas:

```text
fused:
    fused tracks + raw no fusionados publishables.

scored:
    puntos con score como campo o color si se implementa.

raw_debug:
    puntos raw sin filtrar, solo debug.
```

Logs:

```text
[F1T-RVIZ-CLOUD-PUBLISH]
```

---

### 2. Publicar KeyFrames como MarkerArray

Topics sugeridos:

```text
/debug/keyframes_global
/debug/keyframes_raw
```

Estados visuales:

```text
RAW_ANCHORED
OPTIMIZED_BY_SERVER
PROPAGATED_AFTER_SERVER_OPT
REBASED_AFTER_OPTIMIZATION
FIDUCIAL_HARD
FIDUCIAL_SOFT
PENDING_DURING_OPTIMIZATION
```

No fijar colores si ya hay convención del proyecto; documentarlos si se añaden.

Logs:

```text
[F1T-RVIZ-KF-MARKERS]
```

---

### 3. Publicar fiduciales y anchors

Topic:

```text
/debug/fiducials
/debug/submap_anchors
```

Debe mostrar:

- fiducial id;
- pose world;
- KFs asociados;
- submapas anclados.

Logs:

```text
[F1T-RVIZ-FIDUCIAL-MARKERS]
```

---

### 4. Publicar candidatos y loops

Topics:

```text
/debug/loop_candidates
/debug/loop_verified
/debug/loop_rejected
```

Mostrar líneas entre:

```text
query_kf ↔ candidate_seed_kf
```

Con texto:

```text
bow_score
decision
error_t
inliers
```

Logs:

```text
[F1T-RVIZ-LOOP-MARKERS]
```

---

### 5. Publicar subnubes y matches de loop

Topics opcionales:

```text
/debug/subcloud_query
/debug/subcloud_candidate_initial
/debug/subcloud_candidate_reduced
/debug/subcloud_matches
```

Deben poder activarse/desactivarse por parámetros porque pueden ser pesados.

Logs:

```text
[F1T-RVIZ-SUBCLOUD-MARKERS]
```

---

### 6. Publicar pose graph debug

Topics:

```text
/debug/pose_graph_vertices
/debug/pose_graph_edges
/debug/pose_graph_priors
/debug/optimization_motion
```

Mostrar:

- vértices fijos;
- vértices variables;
- aristas internas;
- constraint loop/fiducial;
- movimiento before/after.

Debe reutilizar datos de `PoseGraphProblem` y resultados dry-run/apply.

Logs:

```text
[F1T-RVIZ-GRAPH-MARKERS]
```

---

### 7. Publicar fused landmarks y scores

Topics:

```text
/debug/fused_landmarks
/debug/score_colored_points
/debug/low_score_points
```

Mostrar:

- fused_id;
- miembros;
- score;
- puntos omitidos por score bajo.

Logs:

```text
[F1T-RVIZ-FUSED-MARKERS]
[F1T-RVIZ-SCORE-MARKERS]
```

---

### 8. Crear o actualizar layout RViz2

Crear o actualizar un `.rviz` que incluya:

- nubes globales;
- KeyFrames;
- fiduciales;
- loops;
- grafo;
- fused landmarks;
- TF si procede.

Ruta sugerida:

```text
simulacion_dron/rviz/sparse_global_debug.rviz
```

o ruta existente documentada.

---

### 9. Parámetros de activación

Parámetros recomendados:

```text
debug_publish_rviz
debug_publish_raw_cloud
debug_publish_keyframes
debug_publish_fiducials
debug_publish_loop_candidates
debug_publish_subclouds
debug_publish_pose_graph
debug_publish_fused_landmarks
debug_publish_score_markers
debug_max_debug_points
```

---

## Cambios prohibidos

- No cambiar poses ni scores por RViz.
- No publicar debug pesado siempre si afecta rendimiento.
- No mezclar nube raw debug con nube principal.
- No depender de RViz para criterios automáticos.
- No usar GT para loops.

---

## Paquetes a compilar

```bash
./codex/herramientas/build_selected_packages.sh orbslam3_multi orbslam3_server simulacion_dron
```

---

## Pruebas Gazebo requeridas

### Prueba 1 — visualización básica

YAML:

```text
codex/archivos_auxiliares/trayectorias/tray_prueba_1.yaml
```

Secuencia:

1. arrancar multi-dron;
2. anclar fiducial;
3. publicar nube;
4. mostrar KFs/fiduciales/nube en RViz2.

Comando:

```bash
./codex/herramientas/run_simulation.sh \
  --prueba 1 \
  --launch "ros2 launch simulacion_dron multi_dron.launch.py" \
  --post-scenario-wait-sec 20
```

---

### Prueba 2 — visualización de loop/optimización

YAML:

```text
codex/archivos_auxiliares/trayectorias/tray_prueba_2.yaml
```

Debe generar al menos candidatos BoW/subnubes o usar replay.

---

## Patrones de reduccion de logs

```text
SCENARIO-RUNNER|GOAL|RESULT|success|F1T-|RVIZ|DEBUG|MARKER|ERROR|FATAL|Segmentation fault|Killed
```

---

## Marcadores tecnicos obligatorios

```text
[F1T-RVIZ-CLOUD-PUBLISH]
[F1T-RVIZ-KF-MARKERS]
[F1T-RVIZ-FIDUCIAL-MARKERS]
[F1T-RVIZ-LOOP-MARKERS]
[F1T-RVIZ-FUSED-MARKERS]
```

Si se activa grafo/subnubes:

```text
[F1T-RVIZ-SUBCLOUD-MARKERS]
[F1T-RVIZ-GRAPH-MARKERS]
```

---

## Criterio de exito

La subfase se considera `CONSEGUIDA` si:

1. compila;
2. los topics debug se publican cuando están activados;
3. la nube principal no cambia por activar debug;
4. RViz2 puede abrir layout documentado;
5. logs muestran contadores de markers;
6. no hay errores graves;
7. documentación queda actualizada.

---

## Criterio de fallo o parcial

`NO CONSEGUIDA` si:

- no compila;
- no se publican markers básicos;
- debug modifica lógica;
- la simulación se vuelve inestable por debug.

`PARCIAL` si:

- nubes/KFs/fiduciales funcionan, pero falta grafo o subnubes;
- RViz2 no se pudo comprobar visualmente, pero logs son correctos.

---

## Documentacion a actualizar

Actualizar:

- `codex/contexto/paquetes/orbslam3_server/debug_topics.md`
- `codex/contexto/paquetes/simulacion_dron/rviz.md`
- `codex/contexto/paquetes/orbslam3_multi/global_map_builder.md`
- historial 1U
